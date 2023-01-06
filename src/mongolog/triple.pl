:- module(mongolog_triple,
		[ 
			mng_triple_doc(t,-,t),
			triple(t,t,t),
			get_unique_name(r,-),
			is_unique_name(r),
			load_owl/1,
			load_owl/2,
			load_owl/3,
            drop_graph(+)
		]).
/** <module> Handling of triples in query expressions.

The following predicates are supported:

| Predicate            | Arguments |
| ---                  | ---       |
| triple/3         | ?Subject, ?Property, ?Value |

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
		[ rdf_meta/1, rdf_equal/2 ]).
:- use_module(library('semweb_ext'),
		[ get_supgraphs/2 ]).
:- use_module(library('scope'),
		[ universal_scope/1, time_scope/3, time_scope_data/2 ]).
:- use_module(library('mongodb/client')).
:- use_module(library('mongolog/mongolog')).
:- use_module(library('mongolog/mongolog_test')).
:- use_module(library('semweb_ext')).
:- use_module('subgraph').

:- rdf_meta(taxonomical_property(r)).
:- rdf_meta(must_propagate_assert(r)).
:- rdf_meta(lookup_parents_property(t,t)).

% define some settings
:- setting(drop_graphs, list, [user],
		'List of named graphs that should initially by erased.').

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
:- mongolog:add_command(triple).

%%
mongolog:step_expand(
	project(triple(S,P,O)),
	assert(triple(S,P,O))) :-
	!.

%%
mongolog:step_compile(assert(triple(S,P,term(O))), Ctx, Pipeline, StepVars) :-
	% HACK: convert term(A) argument to string.
	%       it would be better to store lists/terms directly without conversion.
	ground(O),!,
	( atom(O) -> Atom=O ; term_to_atom(O, Atom) ),
	mongolog:step_compile(assert(triple(S,P,string(Atom))), Ctx, Pipeline, StepVars).

mongolog:step_compile(triple(S,P,term(O)), Ctx, Pipeline, StepVars) :-
	% HACK: convert term(A) argument to string.
	%       it would be better to store lists/terms directly without conversion.
	ground(O),!,
	( atom(O) -> Atom=O ; term_to_atom(O, Atom) ),
	mongolog:step_compile(triple(S,P,string(Atom)), Ctx, Pipeline, StepVars).

%%
mongolog:step_compile(assert(triple(S,P,O)), Ctx, Pipeline, StepVars) :-
	% add step variables to compile context
	triple_step_vars(triple(S,P,O), Ctx, StepVars0),
	mongolog:add_assertion_var(StepVars0, StepVars),
	merge_options([step_vars(StepVars)], Ctx, Ctx0),
	% create pipeline
	compile_assert(triple(S,P,O), Ctx0, Pipeline).

%%
mongolog:step_compile(triple(S,P,O), Ctx, Pipeline, StepVars) :-
	% add step variables to compile context
	triple_step_vars(triple(S,P,O), Ctx, StepVars),
	merge_options([step_vars(StepVars)], Ctx, Ctx0),
	% create pipeline
	compile_ask(triple(S,P,O), Ctx0, Pipeline).

%%
triple_step_vars(triple(S,P,O), Ctx, StepVars) :-
	(	bagof(Var,
			(	mongolog:goal_var([S,P,O], Ctx, Var)
			;	mongolog:context_var(Ctx, Var)
			% HACK: remember that variable is wrapped in term/1
			;	(	nonvar(O),
					O=term(O1),
					var(O1),
					mongolog:var_key(O1, Ctx, Key),
					Var=[Key,term(O1)]
				)
			),
			StepVars)
	;	StepVars=[]
	).

%%
% ask(triple(S,P,O)) uses $lookup to join input documents with
% the ones matching the triple pattern provided.
%
compile_ask(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	findall(LookupStep,
		lookup_triple(triple(S,P1,O), Ctx0, LookupStep),
		LookupSteps),
	LookupSteps \== [],
	% compute steps of the aggregate pipeline
	findall(Step,
		% filter out documents that do not match the triple pattern.
		% this is done using $match or $lookup operators.
		(	member(Step, LookupSteps)
		% compute the intersection of scope so far with scope of next document
		;	scope_intersect('v_scope',
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
% assert(triple(S,P,O)) uses $lookup to find matching triples
% with overlapping scope which are toggled to be removed in next stage.
% then the union of their scopes is computed and used for output document.
%
compile_assert(triple(S,P,O), Ctx, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), P1, Ctx, Ctx0),
	option(collection(Collection), Ctx0),
	option(scope(Scope), Ctx0),
	triple_graph(Ctx0, Graph),
	time_scope_values(Scope, SinceTyped, UntilTyped),
	% throw instantiation_error if one of the arguments was not referred to before
	mongolog:all_ground([S,O], Ctx),
	% resolve arguments
	mongolog:var_key_or_val(S, Ctx, S_query),
	mongolog:var_key_or_val(O, Ctx, V_query),
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
					['since', SinceTyped],
					['until', UntilTyped]
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
		;	mongolog:add_assertions(string('$next'), Collection, Step)
		% add merged triple document to triples array
		;	mongolog:add_assertion(TripleDoc, Collection, Step)
		;	(	once(must_propagate_assert(P)),
				propagate_assert(S, Ctx0, Step)
			)
		),
		Pipeline
	).

%%
time_scope_values(Scope, SinceValue, UntilValue) :-
	time_scope_data(Scope, [Since,Until]),
	time_scope_value1(Since, SinceValue),
	time_scope_value1(Until, UntilValue).

time_scope_value1(V0, Value) :-
	mng_strip_operator(V0, _, V1),
	once(time_scope_value2(V1, Value)).

time_scope_value2('Infinity', double('Infinity')).
time_scope_value2(Num,  double(Num))  :- number(Num).
time_scope_value2(Atom, string(Atom)) :- atom(Atom).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LOOKUP triple documents
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_triple(triple(S,P,V), Ctx, Step) :-
	\+ memberchk(transitive, Ctx),
	memberchk(collection(Coll), Ctx),
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
				mongolog:var_key(ArgVar, Ctx, ArgKey),
				atom_concat('$$',ArgKey,ArgValue),
				atom_concat(ArgValue,'.type',ArgType),
				triple_arg_value(Arg, ArgValue, FieldValue, Ctx, ArgExpr),
				MatchQuery=['$expr', ['$or', array([
					% pass through if var is not grounded
					['$eq', array([string(ArgType), string('var')])],
					ArgExpr % else perform a match
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
	mongolog:lookup_let_doc(StepVars, LetDoc),
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
	mng_one_db(_DB, OneColl),
	% infer lookup parameters
	query_value(P,Query_p),
	% TODO: can query operators be supported?
	mng_strip_variable(S, S0),
	mng_strip_variable(V, V0),
	mongolog:var_key_or_val(S0, Ctx, S_val),
	mongolog:var_key_or_val(V0, Ctx, V_val),
	
	% FIXME: a runtime condition is needed to cover the case where S was
	%        referred to in ignore'd goal that failed.
	(	has_value(S0,Ctx)
	->	( Start=S_val, To='s', From='o', StartValue='$start.s' )
	;	( Start=V_val, To='o', From='s', StartValue='$start.o' )
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
	mongolog:is_referenced(Var, Ctx).

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
	memberchk(step_vars(StepVars), Ctx),
	% read triple data
	mongolog:var_key_or_val1(P, Ctx, P0),
	mongolog:var_key_or_val1(S, Ctx, S0),
	mongolog:var_key_or_val1(V, Ctx, V0),
	% read scope data
	option(scope(Scope), Ctx),
	time_scope_values(Scope, Since, Until),
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
	mongolog:lookup_let_doc(StepVars, LetDoc),
	% build pipeline
	findall(Step,
		% $match s,p,o and overlapping scope
		(	Step=['$match',[
				['s',S0], ['p',P0], ['o',V0],
				['scope.time.since',['$lte',Until]],
				['scope.time.until',['$gte',Since0]]
			]]
		% only keep scope field
		;	Step=['$project',[['scope',int(1)]]]
		% toggle delete flag
		;	Step=['$set',['delete',bool(true)]]
		),
		Pipeline
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% RDFS
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
lookup_parents_property(triple(_,rdfs:subPropertyOf,P), [P,rdfs:subPropertyOf]).
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
propagate_assert(S, Context, Step) :-
	memberchk(collection(Collection), Context),
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
			['from',string(Collection)],
			['as',string('next')],
			['let',[['parents',string('$parents')]]],
			['pipeline',array(Inner)]
		]]
	% second, add each document to triples array
	;	mongolog:add_assertions(string('$next'), Collection, Step)
	).

%% the properties for which assertions must be propagated
must_propagate_assert(rdfs:subClassOf).
must_propagate_assert(rdfs:subPropertyOf).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% triple/3 query pattern
%%%%%%%%%%%%%%%%%%%%%%%

%% triple(?Subject, ?Property, ?Value) is nondet.
%
% Query values of a property (and their sub-properties) on some subject in the triple DB.
% If the property is rdfs:subPropertyOf or rdf:type the query returns the values for the 
% subject and their super-class
% 
% The property can be wrapped in one of several options:
% 
%     - transitive(Property) 
%       indicates that the property is transitive
%     - reflexive(Property)
%       indicates that the property is reflexive
%     - pstar(Property)
%       binds the property to one of the values in the p* field in the mongodb
%
% The value can be wrapped in one of several options:
%
%     - ostar(Value)
%       binds the value to one of the values in the o* field in the mongodb
%
% @param Subject The subject of a triple.
% @param Property The predicate of a triple.
% @param Value The object of a triple.
%
triple(S,P,O) :-
	kb_call(triple(S,P,O)).

%% mng_triple_doc(+Triple, -Doc, +Context) is semidet.
%
% Translate a triple term into a mongo query document.
%
mng_triple_doc(triple(S,P,V), Doc, Context) :-
	%% read options
	triple_graph(Context, Graph),
	option(scope(Scope), Context, dict{}),
	% special handling for some properties
	(	taxonomical_property(P)
	->	( Key_p='p',  Key_o='o*' )
	;	( Key_p='p*', Key_o='o' )
	),
	% strip term ->(Term,Var)
	mng_strip_variable(S, S1),
	mng_strip_variable(P, P1),
	mng_strip_variable(V, V1),
	% get the query pattern
	% FIXME: query_value may silently fail on invalid input and this rule still succeeds
	findall(X,
		(	( query_value(S1,Query_s), X=['s',Query_s] )
		;	( query_value(P1,Query_p), X=[Key_p,Query_p] )
		;	( query_value(V1,Query_v), \+ is_term_query(Query_v), X=[Key_o,Query_v] )
		;	graph_doc(Graph,X)
		;	scope_doc(Scope,X)
		),
		Doc
	),
	% ensure doc has value if input is grounded
	once((\+ ground(S1) ; memberchk(['s',_],Doc))),
	once((\+ ground(P1) ; memberchk([Key_p,_],Doc))),
	once((\+ ground(V1) ; memberchk([Key_o,_],Doc))).

%%
is_term_query([[type,string(compound)],_]).
is_term_query([_, [[type,string(compound)],_]]).

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
	triple_graph(Ctx, Graph),
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


%% scope_intersect(+VarKey, +Since1, +Until1, +Options, -Step) is nondet.
%
% The step expects input documents with VarKey
% field, and another field Since1/Until1.
% The step uses these field to compute an intersection
% beteween both scopes.
% It will fail in case the intersection is empty.
%
scope_intersect(VarKey, Since1, Until1, Options, Step) :-
	atomic_list_concat(['$',VarKey,'.time.since'], '', Since0),
	atomic_list_concat(['$',VarKey,'.time.until'], '', Until0),
	atomic_list_concat(['$',VarKey], '', VarKey0),
	%
	Intersect = ['time', [
		['since', ['$max', array([string(Since0), Since1])]],
		['until', ['$min', array([string(Until0), Until1])]]
	]],
	% check if ignore flag if set, if so use a conditional step
	(	memberchk(ignore, Options)
	->	IntersectStep = ['$cond', array([
			['$not', array([Since1])],
			string(VarKey0),
			Intersect
		])]
	;	IntersectStep = Intersect
	),
	% first compute the intersection
	(	Step=['$set', ['v_scope', IntersectStep]]
	% then verify that the scope is non empty
	;	Step=['$match', ['$expr',
			['$lt', array([string(Since0), string(Until0)])]
		]]
	).


%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% ontology loading
%%%%%%%%%%%%%%%%%%%%%%%

load_owl(URL) :-
    load_owl(URL,[]).

load_owl(URL, Options) :-
    register_ontology_namespace(URL, Options),
    % FIXME: it is hard to follow here how the subgraph-of relation is build, i.e. using add_subgraph/2.
    %        it is also a question whether this relation should be maintained centrally
    %        for all reasoner that use RDF data.
	% get parent graph name, fall back to "common"
	option(parent_graph(ParentGraph), Options, common),
	% get fact scope
	universal_scope(UScope),
	option(scope(Scope), Options, UScope),
    %
	load_owl(URL, Scope, ParentGraph) .

load_owl(_URL, _Scope, _ParentGraph) :-
    setting(mng_client:read_only, true),
    !.

load_owl(URL, Scope, ParentGraph) :-
    % read graph and ontology version from URL
    ontology_url(URL, Resolved, OntologyGraph, OntologyVersion),
	% setup graph structure:
	% the parent graph is a super graph of the ontology graph, meaning the
	% ontology graph is included in the parent graph.
	% if the parent graph is dropped, the ontology graph will also be dropped.
	(	add_subgraph(OntologyGraph,ParentGraph),
	    % TODO: doesn't this create a cricle in the relation, e.g. when 'test' is used as
	    %        a parent in unittests to auto-drop the ontology graph after the test.
		add_subgraph(user,OntologyGraph)
	),
	% tests whether the ontology is already loaded. no triples mustbe loaded if versions unify,
	% but the ontology graph must be a sub-graph of all imported ontologies.
	(  mongo_ontology_version(OntologyGraph, OntologyVersion)
	-> load_ontology_graph(OntologyGraph)
	;  load_owl1(Resolved, OntologyGraph, OntologyVersion, Scope, ParentGraph)
	).

load_owl1(URL, OntologyGraph, OntologyVersion, Scope, ParentGraph) :-
    % read ontology data from URL
	ontology_url_read(URL, [
	    asserted_url(AssertedURL),
	    triples(TripleTerms),
	    annotations(AnnotationTerms)
	]),
	% erase old triples
	drop_graph(OntologyGraph),
	% load RDF data of imported ontologies
	rdf_equal(owl:'imports',OWL_Imports),
	forall(
		member(triple(AssertedURL, OWL_Imports, string(ImportedURL)), TripleTerms),
		(	ontology_url_graph(ImportedURL, ImportedGraph),
		    % TODO this seems odd, shouldn't it be the other direction?
		    %      Is this needed at all? It also seems risky that it cause an ontology
		    %      being accidentally dropped. Try to remove this....
			add_subgraph(OntologyGraph, ImportedGraph),
			load_owl(ImportedURL, Scope, ParentGraph)
		)
	),
	% FIXME: o* for subClassOf only includes direct super class when loading a list of triples at once.
	%mongolog_assert(TripleTerms, Scope, [graph(Graph)]),
	%mongolog_assert(AnnotationTerms, Scope, [graph(Graph)]),
	forall(
		(	member(Term, TripleTerms)
		;	member(Term, AnnotationTerms)
		),
		mongolog_call(assert(Term), [ scope(Scope), graph(OntologyGraph) ])
	),
	% assert ontology version
	set_mongo_ontology_version(AssertedURL, OntologyVersion, OntologyGraph),
	!,
	log_debug(mongolog(ontology_loaded(OntologyGraph,OntologyVersion))).


%%
load_ontology_graph(OntologyGraph) :-
	rdf_equal(owl:'imports',OWL_Imports),
	mng_get_db(DB, Coll, 'triples'),
	forall(
		mng_find(DB, Coll, [
			['p',     string(OWL_Imports)],
			['graph', string(OntologyGraph)]
		], Doc),
		(	mng_get_dict('o', Doc, string(Imported)),
			ontology_url_graph(Imported,ImportedGraph),
			% TODO: same note as above, this seems odd
			add_subgraph(OntologyGraph,ImportedGraph)
		)
	).

%% The version/last modification time of a loaded ontology
mongo_ontology_version(OntologyGraph, Version) :-
	mng_get_db(DB, Coll, 'triples'),
	once(mng_find(DB, Coll, [
		['p',     string(tripledbVersionString)],
		['graph', string(OntologyGraph)]
	], Doc)),
	mng_get_dict(o, Doc, string(Version)).

%% Write version string into DB
set_mongo_ontology_version(URL, Version, OntoGraph) :-
	mng_get_db(DB, Coll, 'triples'),
	mng_store(DB, Coll, [
		['s',     string(URL)],
		['p',     string(tripledbVersionString)],
		['o',     string(Version)],
		['graph', string(OntoGraph)]
	]).

%% drop_graph(+Name) is det.
%
% Deletes all triples asserted into given named graph.
%
% @param Name the graph name.
%
drop_graph(Name) :-
	mng_get_db(DB, Coll, 'triples'),
	mng_remove(DB, Coll, [
		[graph, string(Name)]
	]).

%%
% Drop graphs on startup if requested through settings.
% This is usually done to start with an empty "user" graph
% when KnowRob is started.
%
auto_drop_graphs :-
	\+ setting(mng_client:read_only, true),
	setting(mongolog_triple:drop_graphs, L),
	forall(member(X,L), drop_graph(X)).

:- ignore(auto_drop_graphs).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
triple_graph(Ctx, Graph) :-
	once((semweb_ext:default_graph(DefaultGraph) ; DefaultGraph=user)),
	option(graph(Graph), Ctx, DefaultGraph).

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
			mongolog:goal_var(Arg, Ctx, [Key, _Var])
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

%% is_unique_name(+Name) is semidet.
%
% True if Name is not the subject of any known fact.
%
is_unique_name(Name) :-
	mng_get_db(DB, Coll, 'triples'),
	\+ mng_find(DB, Coll, [['s',string(Name)]], _).

%% get_unique_name(+Prefix, -Name) is semidet.
%
% Generates a unique name with given prefix.
%
get_unique_name(Prefix, Name) :-
	% generate 8 random alphabetic characters
	randseq(8, 25, Seq_random),
	maplist(plus(65), Seq_random, Alpha_random),
	atom_codes(Sub, Alpha_random),
	% TODO: what IRI prefix? Currently we re-use the one of the type.
	%        but that seems not optimal. Probably best to
	%        have this in query context, and some meaningful default.
	atomic_list_concat([Prefix,'_',Sub], IRI),
	% check if there is no triple with this identifier as subject or object yet
	(	is_unique_name(IRI)
	->	Name=IRI
	;	unique_name(Prefix,Name)
	).