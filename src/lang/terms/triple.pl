:- module(lang_triple,
		[ mng_triple_doc(t,-,t) ]).

:- use_module(library('semweb/rdf_db'),
		[ rdf_meta/1 ]).
:- use_module(library('lang/subgraph'),
		[ get_supgraphs/2 ]).
:- use_module(library('lang/scope'),
		[ time_scope/3, time_scope_data/2 ]).
:- use_module(library('db/mongo/client'),
		[ mng_get_db/3, mng_find/4, mng_query_value/2,
		  mng_strip_type/3, mng_strip_variable/2,
		  mng_strip_operator/3, mng_operator/2,
		  mng_typed_value/2 ]).
:- use_module(library('lang/compiler')).
:- use_module('intersect',
		[ mng_scope_intersect/5 ]).

:- rdf_meta(taxonomical_property(r)).
:- rdf_meta(must_propagate_tell(r)).
:- rdf_meta(pstar_value(r,t)).
:- rdf_meta(ostar_value(r,r,t)).
:- rdf_meta(lookup_parents_property(t,t)).

%%
% register the "triples" collection.
% This is needed for import/export.
% It also creates search indices.
%
:- setup_collection(triples, [
		['s'], ['p'], ['o'], ['p*'], ['o*'],
		['s','p'], ['s','o'], ['o','p'],
		['s','p*'], ['s','o*'], ['o','p*'], ['p','o*'],
		['s','o','p'], ['s','o','p*'], ['s','o*','p'] ]).

%% register query commands
:- query_compiler:add_command(triple).

%%
query_compiler:step_vars(triple(S,P,O), Ctx, Vars) :-
	bagof(Var,
		(	query_compiler:goal_var([S,P,O], Ctx, Var)
		;	query_compiler:context_var(Ctx, Var)
		),
		Vars).

%%
query_compiler:step_compile(triple(S,P,O), Ctx, Pipeline) :-
	(	option(mode(ask), Ctx)
	->	compile_ask( triple(S,P,O), Ctx, Pipeline)
	;	compile_tell(triple(S,P,O), Ctx, Pipeline)
	).

%%
% ask(triple(S,P,O)) uses $lookup to join input documents with
% the ones matching the triple pattern provided.
%
compile_ask(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% filter out documents that do not match the triple pattern.
		% this is done using $match or $lookup operators.
		(	lookup_triple(triple(S,P1,O), Ctx0, Step)
		% compute the intersection of scope so far with scope of next document
		;	mng_scope_intersect('v_scope',
				string('$next.scope.time.since'),
				string('$next.scope.time.until'),
				Ctx0, Step)
		% the triple doc contains parents of the P at p*
		% and parents of O at o*.
		% these can be unwinded to yield a solution for each parent.
		;	unwind_parents(Ctx0, Step)
		% project new variable groundings
		;	set_triple_vars(S,P1,O,Ctx0,Step)
		% remove next field again
		;	Step=['$unset',string('next')]
		),
		Pipeline
	).

%%
% tell(triple(S,P,O)) uses $lookup to find matching triples
% with overlapping scope which are toggled to be removed in next stage.
% then the union of their scopes is computed and used for output document.
%
compile_tell(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	option(graph(Graph), Ctx0, user),
	option(scope(Scope), Ctx0),
	time_scope_data(Scope, [Since,Until]),
	% input validation
	query_compiler:all_ground([S,O], Ctx),
	% strip the value, assert that operator must be $eq
	query_compiler:var_key_or_val(S, Ctx, S_query),
	query_compiler:var_key_or_val(O, Ctx, V_query),
	% special handling for RDFS semantic
	% FIXME: with below code P can not be inferred in query
	(	taxonomical_property(P1)
	->	( Pstar=array([string(P1)]), Ostar=string('$parents') )
	;	( Pstar=string('$parents'),  Ostar=array([V_query]) )
	),
	% build triple docuemnt
	TripleDoc=[
		['s', S_query], ['p', string(P1)], ['o', V_query],
		['p*', Pstar], ['o*', Ostar],
		['graph', string(Graph)],
		['scope', string('$v_scope')]
	],
	% configure the operation performed on scopes.
	% the default is to compute the union of scopes.
	(	option(intersect_scope, Ctx)
	->	(SinceOp='$max', UntilOp='$min')
	;	(SinceOp='$min', UntilOp='$max')
	),
	% compute steps of the aggregate pipeline
	% TODO: if just one document, update instead of delete
	findall(Step,
		% assign v_scope field. 
		(	Step=['$set', ['v_scope', [['time',[
					['since', double(Since)],
					['until', double(Until)]
			]]]]]
		% lookup documents that overlap with triple into 'next' field,
		% and toggle their delete flag to true
		;	delete_overlapping(triple(S,P,O), Ctx0, Step)
		% lookup parent documents into the 'parents' field
		;	lookup_parents(triple(S,P1,O), Ctx0, Step)
		% update v_scope.time.since
		;	reduce_num_array(string('$next'), SinceOp,
				'scope.time.since', 'v_scope.time.since', Step)
		% get max until of scopes in $next, update v_scope.time.until
		;	reduce_num_array(string('$next'), UntilOp,
				'scope.time.until', 'v_scope.time.until', Step)
		% add triples to triples array that have been queued to be removed
		;	array_concat('triples', string('$next'), Step)
		% add merged triple document to triples array
		;	array_concat('triples', array([TripleDoc]), Step)
		;	(	once(must_propagate_tell(P)),
				propagate_tell(S, Ctx0, Step)
			)
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LOOKUP triple documents
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_triple(triple(S,P,V), Ctx, Step) :-
	\+ memberchk(transitive, Ctx),
	memberchk(collection(Coll), Ctx),
	memberchk(outer_vars(QueryVars), Ctx),
	memberchk(step_vars(StepVars), Ctx),
	% FIXME: revise below
	mng_triple_doc(triple(S,P,V), QueryDoc, Ctx),
	(	memberchk(['s',_],QueryDoc)
	->	StartValue='$start.s'
	;	StartValue='$start.o'
	),
	%
	(	taxonomical_property(P)
	->	( Key_p='$p',  Key_o='$o*' )
	;	( Key_p='$p*', Key_o='$o' )
	),
	findall(MatchQuery,
		% first match what is grounded in compile context
		(	MatchQuery=QueryDoc
		% next match variables grounded in call context
		;	(	member([Arg,FieldValue],[[S,'$s'],[P,Key_p],[V,Key_o]]),
				triple_arg_var(Arg, ArgVar),
				query_compiler:var_key(ArgVar, Ctx, ArgKey),
				atom_concat('$$',ArgKey,ArgValue),
				%% TODO: need to check if key exists?
				%atom_concat('$',FieldKey,FieldValue),
				%['$and', array([[FieldKey, ['$exists', bool(true)]], ['$expr', ArgExpr]])]
				% variables are documents with a "type" field set to "var"
				atom_concat(ArgValue,'.type',ArgType),
				triple_arg_value(Arg, ArgValue, FieldValue, Ctx, ArgExpr),
				MatchQuery=['$expr', ['$or', array([
					% pass through if var is not grounded
					['$eq', array([string(ArgType), string('var')])],
					% else perform a match
					ArgExpr
				])]]
			)
		;	scope_match(Ctx, MatchQuery)
		;	graph_match(Ctx, MatchQuery)
		),
		MatchQueries
	),
	%
	findall(InnerStep,
		% match input triples with query pattern
		(	(	MatchQueries=[FirstMatch]
			->	InnerStep=['$match', FirstMatch]
			;	InnerStep=['$match', ['$and', array(MatchQueries)]]
			)
		% limit results if requested
		;	(	member(limit(Limit),Ctx),
				InnerStep=['$limit',int(Limit)]
			)
		),
		InnerPipeline
	),
	% pass input document values to lookup
	query_compiler:lookup_let_doc(StepVars, QueryVars, LetDoc),
	% lookup matching documents and store in 'next' field
    (	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',LetDoc],
			['pipeline', array(InnerPipeline)]
		]]
	% add additional results if P is a reflexive property
	;	(	memberchk(reflexive,Ctx),
			(	Step=['$unwind',string('$next')]
			;	Step=['$set', ['start', string('$next')]]
			;	Step=['$set', ['next', array([string('$next')])]]
			;	reflexivity(StartValue, Ctx, Step)
			)
		)
	% at this point 'next' field holds an array of matching documents
	% that is unwinded here.
	;	Step=['$unwind',string('$next')]
	).

%%
lookup_triple(triple(S,P,V), Ctx, Step) :-
	% read options
	option(transitive, Ctx),
	option(collection(Coll), Ctx),
%	option(graph(Graph), Ctx, user),
%	option(scope(Scope), Ctx),
	mng_one_db(_DB, OneColl),
	% infer lookup parameters
	query_value(P,Query_p),
	% TODO: can query operators be supported?
	mng_strip_variable(S, S0),
	mng_strip_variable(V, V0),
	query_compiler:var_key_or_val(S0, Ctx, S_val),
	query_compiler:var_key_or_val(V0, Ctx, V_val),
	
	% FIXME: a runtime condition is needed to cover the case where S was
	%        referred to in ignore'd goal that failed.
	(	has_value(S0,Ctx)
	->	(Start=S_val, To='s', From='o', StartValue='$start.s')
	;	(Start=V_val, To='o', From='s', StartValue='$start.o')
	),
	
	% match doc for restring the search
	findall(Restriction,
		(	Restriction=['p*',Query_p]
		% TODO: see how scope can be included
		%;	graph_doc(Graph,Restriction)
		%;	scope_doc(Scope,Restriction)
		),
		MatchDoc
	),
	% recursive lookup
	(	Step=['$graphLookup', [
			['from',                    string(Coll)],
			['startWith',               Start],
			['connectToField',          string(To)],
			['connectFromField',        string(From)],
			['as',                      string('t_paths')],
			['depthField',              string('depth')],
			['restrictSearchWithMatch', MatchDoc]
		]]
	% $graphLookup does not ensure order, so we need to order by recursion depth
	% in a separate step
	;	Step=['$lookup', [
			['from',     string(OneColl)],
			['as',       string('t_sorted')],
			['let',      [['t_paths', string('$t_paths')]]],
			['pipeline', array([
				['$set',         ['t_paths', string('$$t_paths')]],
				['$unwind',      string('$t_paths')],
				['$replaceRoot', ['newRoot', string('$t_paths')]],
				['$sort',        ['depth', integer(1)]]
			])]
		]]
	;	Step=['$set', ['next', string('$t_sorted')]]
	;	Step=['$set', ['start', ['$arrayElemAt',
			array([string('$next'), integer(0)])
		]]]
	;	Step=['$unset', string('t_paths')]
	;	Step=['$unset', string('t_sorted')]
	% add additional triple in next if P is a reflexive property
	;	reflexivity(StartValue, Ctx, Step)
	% iterate over results
	;	Step=['$unwind',string('$next')]
	% match values with expression given in query
	;	(	To=='s', has_value(V0,Ctx),
			Step=['$match', ['next.o', V_val]]
		)
	).

%%
% FIXME: need to do runtime check for ignored goals!
%
has_value(X, _Ctx) :-
	ground(X),!.
has_value(X, Ctx) :-
	term_variables(X,Vars),
	member(Var,Vars),
	query_compiler:is_referenced(Var, Ctx).

%%
reflexivity(StartValue, Ctx, Step) :-
	memberchk(reflexive,Ctx),
	(	Step=['$set', ['t_refl', [
			['s',string(StartValue)],
			['p',string('$start.p')],
			['p*',string('$start.p*')],
			['o',string(StartValue)],
			['o*',array([string(StartValue)])],
			['graph',string('$start.graph')],
			['scope',string('$start.scope')]
		]]]
	;	Step=['$set', ['next', ['$concatArrays',
			array([array([string('$t_refl')]), string('$next')])
		]]]
	;	Step=['$unset', array([string('t_refl'),string('start')])]
	).

%%
delete_overlapping(triple(S,P,V), Ctx,
		['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',LetDoc],
			['pipeline',array(Pipeline)]
		]]) :-
	memberchk(collection(Coll), Ctx),
	memberchk(outer_vars(QueryVars), Ctx),
	memberchk(step_vars(StepVars), Ctx),
	% read triple data
	query_compiler:var_key_or_val1(P, Ctx, P0),
	query_compiler:var_key_or_val1(S, Ctx, S0),
	query_compiler:var_key_or_val1(V, Ctx, V0),
	% read scope data
	option(scope(Scope), Ctx),
	time_scope_data(Scope,[Since,Until]),
	% set Since=Until in case of scope intersection.
	% this is to limit to results that do hold at Until timestamp.
	% FIXME: when overlap yields no results, zero is used as since
	%        by until. but then the new document could overlap
	%        with existing docs, which is not wanted.
	%		 so better remove special handling here?
	%        but then the until time maybe is set to an unwanted value? 
	(	option(intersect_scope, Ctx)
	->	Since0=Until
	;	Since0=Since
	),
	query_compiler:lookup_let_doc(StepVars, QueryVars, LetDoc),
	% build pipeline
	findall(Step,
		% $match s,p,o and overlapping scope
		(	Step=['$match',[
				['s',S0], ['p',P0], ['o',V0],
				['scope.time.since',['$lte',double(Until)]],
				['scope.time.until',['$gte',double(Since0)]]
			]]
		% only keep scope field
		;	Step=['$project',[['scope',int(1)]]]
		% toggle delete flag
		;	Step=['$set',['delete',bool(true)]]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% PROPAGATION
%%%%%%%%%%%%%%%%%%%%%%%

%%
unwind_parents(Context, Step) :-
	option(pstar, Context),
	(	Step=['$unwind', string('$next.p*')]
	;	Step=['$set', ['next.p', ['$next.p*']]]
	).

unwind_parents(Context, Step) :-
	option(ostar, Context),
	(	Step=['$unwind', string('$next.o*')]
	;	Step=['$set', ['next.o', string('$next.o*')]]
	).

%% set "parents" field by looking up subject+property then yielding o* field
lookup_parents_property(triple(_,rdf:type,O),           [O,rdfs:subClassOf]).
lookup_parents_property(triple(_,rdfs:subClassOf,O),    [O,rdfs:subClassOf]).
lookup_parents_property(triple(_,rdfs:subPropertyOf,O), [O,rdfs:subPropertyOf]).
lookup_parents_property(triple(_,P,_),                  [P,rdfs:subPropertyOf]).

%%
lookup_parents(Triple, Context, Step) :-
	memberchk(collection(Coll), Context),
	once(lookup_parents_property(Triple, [Child, Property])),
	% make sure value is wrapped in type term
	mng_typed_value(Child,   TypedValue),
	mng_typed_value(Property,TypedProperty),
	% first, lookup matching documents and yield o* in parents array
	(	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('parents')],
			['pipeline',array([
				['$match', [
					['s',TypedValue],
					['p',TypedProperty]
				]],
				['$project', [['o*', int(1)]]],
				['$unwind', string('$o*')]
			])]
		]]
	% convert parents from list of documents to list of strings.
	;	Step=['$set',['parents',['$map',[
			['input',string('$parents')],
			['in',string('$$this.o*')]
		]]]]
	% also add child to parents list
	;	array_concat('parents', array([TypedValue]), Step)
	).

%%
propagate_tell(S, Context, Step) :-
	memberchk(collection(Coll), Context),
	mng_typed_value(S,TypedS),
	% the inner lookup matches documents with S in o*
	findall(X,
		% match every document with S in o*
		(	X=['$match', [['o*',TypedS]]]
		% and add parent field from input documents to o*
		;	array_concat('o*', string('$$parents'), X)
		% only replace o*
		;	X=['$project',[['o*',int(1)]]]
		),
		Inner),
	% first, lookup matching documents and update o*
	(	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',[['parents',string('$parents')]]],
			['pipeline',array(Inner)]
		]]
	% second, add each document to triples array
	;	array_concat('triples', string('$next'), Step)
	).

%% the properties for which assertions must be propagated
must_propagate_tell(rdfs:subClassOf).
must_propagate_tell(rdfs:subPropertyOf).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% triple/3 query pattern
%%%%%%%%%%%%%%%%%%%%%%%

%% mng_triple_doc(+Triple, -Doc, +Context) is semidet.
%
mng_triple_doc(triple(S,P,O), Doc, Context) :-
	%% read options
	option(graph(Graph), Context, user),
	option(scope(Scope), Context),
	% special handling for some properties
	(	taxonomical_property(P)
	->	( Key_p='p',  Key_o='o*' )
	;	( Key_p='p*', Key_o='o' )
	),
	% strip term ->(Term,Var)
	mng_strip_variable(O, V),
	% get the query pattern
	findall(X,
		(	( query_value(S,Query_s), X=['s',Query_s] )
		;	( query_value(P,Query_p), X=[Key_p,Query_p] )
		;	( query_value(V,Query_v), X=[Key_o,Query_v] )
		;	graph_doc(Graph,X)
		;	scope_doc(Scope,X)
		),
		Doc
	).

%%
query_value(In, Out) :-
	% strip $eq operator not needed in triple query.
	% this is needed for current handling of regular expression patterns.
	% FIXME: still needed after $exp fix?
	mng_query_value(In, X),
	(	X=['$eq',Y]
	->	Out=Y
	;	Out=X
	).

%%
triple_arg_var(Arg, ArgVar) :-
	mng_strip_variable(Arg, X),
	term_variables(X, [ArgVar]).

%%
triple_arg_value(_Arg, ArgValue, FieldValue, _Ctx, ['$in',
		array([ string(ArgValue), string(FieldValue) ])]) :-
	% FIXME: operators are ignored!!
	% TODO: can be combined with other operators??
	atom_concat(_,'*',FieldValue),!.
	
triple_arg_value(Arg, ArgValue, FieldValue, _Ctx, [ArgOperator,
		array([ string(FieldValue), string(ArgValue) ])]) :-
	mng_strip_variable(Arg, X),
	mng_strip_operator(X, Operator1, _),
	mng_operator(Operator1, ArgOperator).

%%
graph_doc('*', _)    :- !, fail.
graph_doc('user', _) :- !, fail.
graph_doc(=(GraphName), ['graph',string(GraphName)]) :- !.
graph_doc(  GraphName,  ['graph',['$in',array(Graphs)]]) :-
	get_supgraphs(GraphName,Graphs).

%%
graph_match(Ctx, ['$expr', [Operator,
		array([ string(GraphValue), ArgVal ])
	]]) :-
	option(graph(Graph), Ctx, user),
	graph_doc(Graph, [GraphKey, Arg]),
	atom_concat('$',GraphKey,GraphValue),
	(	(   is_list(Arg), Arg=[Operator,ArgVal])
	;	(\+ is_list(Arg), Arg=[ArgVal], Operator='$eq')
	).

%%
scope_doc(QScope, [Key,Value]) :-
	scope_doc1(QScope, [Key,Value]),
	% do not proceed for variables in scope
	% these are handled later
	(Value=[_,string(_)] -> fail ; true).

scope_doc1(QScope, [Key,Value]) :-
	get_dict(ScopeName, QScope, ScopeData),
	scope_doc(ScopeData, SubPath, Value),
	atomic_list_concat([scope,ScopeName,SubPath], '.', Key).

scope_doc(Scope, Path, Value) :-
	is_dict(Scope),!,
	get_dict(Key,Scope,Data),
	scope_doc(Data,SubPath,Value),
	(	SubPath='' -> Path=Key
	;	atomic_list_concat([Key,SubPath],'.',Path)
	).

scope_doc(Value, '', Query) :-
	mng_query_value(Value, Query).

%%
scope_match(Ctx, ['$expr', ['$and', array(List)]]) :-
	option(scope(Scope), Ctx),
	findall([Operator, array([string(ScopeValue),string(Val)])],
		(	scope_doc1(Scope, [ScopeKey,Arg]),
			atom_concat('$',ScopeKey,ScopeValue),
			(	(   is_list(Arg), Arg=[Operator,ArgVal])
			;	(\+ is_list(Arg), Arg=ArgVal, Operator='$eq')
			),
			% only proceed for variables in scope
			% constants are handled earlier
			ArgVal=string(Val0),
			atom_concat('$',Val0,Val)
		),
		List
	),
	List \== [].
			

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
taxonomical_property(P) :- var(P),!,fail.
taxonomical_property(Term) :-
	compound(Term),!,
	Term =.. [_Functor,Arg],
	taxonomical_property(Arg).
taxonomical_property(rdf:type).
taxonomical_property(rdfs:subClassOf).
taxonomical_property(rdfs:subPropertyOf).

%%
extend_context(triple(_,P,_), P1, Context, Context0) :-
	% get the collection name
	(	option(collection(Coll), Context)
	;	mng_get_db(_DB, Coll, 'triples')
	),
	% read options from argument terms
	% e.g. properties can be wrapped in transitive/1 term to
	% indicate that the property is transitive which will then
	% generate an additional step in the aggregate pipeline.
	strip_property_modifier(P,P_opts,P1),
	% extend the context
	% NOTE: do not use findall here to keep var references in Context valid
	bagof(Opt,
		(	Opt=property(P1)
		;	Opt=collection(Coll)
		;	member(Opt, P_opts)
		;	member(Opt, Context)
		),
		Context0).

%%
get_triple_vars(S, P, O, Ctx, Vars) :-
	findall([Key,Field],
		(	member([Field,Arg], [[s,S],[p,P],[o,O]]),
			query_compiler:goal_var(Arg, Ctx, [Key, _Var])
		),
		Vars).

%%
set_triple_vars(S, P, O, Ctx, ['$set', ProjectDoc]) :-
	get_triple_vars(S,P,O,Ctx,TripleVars),
	findall([Key, string(NextValue)],
		(	member([Key, Field], TripleVars),
			atom_concat('$next.', Field, NextValue)
		),
		ProjectDoc),
	ProjectDoc \= [].

%%
strip_property_modifier(Var,[],Var) :- var(Var), !.
strip_property_modifier(Term,[X|Xs],Stripped) :-
	strip_property_modifier1(Term,X,Term0), !,
	strip_property_modifier(Term0,Xs,Stripped).
strip_property_modifier(Stripped,[],Stripped).

strip_property_modifier1(transitive(X),      ostar,      X) :- taxonomical_property(X),!.
strip_property_modifier1(transitive(X),      transitive, X).
strip_property_modifier1(reflexive(X),       reflexive,  X).
strip_property_modifier1(include_parents(X), pstar,      X).

%%
array_concat(Key,Arr,['$set',
		[Key,['$setUnion',
			array([string(Arr0),Arr])]
		]]) :-
	atom_concat('$',Key,Arr0).

%%
reduce_num_array(ArrayKey, Operator, Path, ValKey, Step) :-
	atom_concat('$$this.', Path, Path0),
	atom_concat('$', ValKey, ValKey0),
	(	Step=['$set',['num_array',['$map',[
			['input', ArrayKey],
			['in', string(Path0)]
		]]]]
	;	array_concat('num_array', array([string(ValKey0)]), Step)
	;	Step=['$set', [ValKey, [Operator, string('$num_array')]]]
	;	Step=['$unset',string('num_array')]
	).
