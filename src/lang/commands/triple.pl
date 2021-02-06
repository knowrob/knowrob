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
		  mng_strip_operator/3,
		  mng_typed_value/2 ]).
:- use_module(library('lang/compiler')).
:- use_module('intersect',
		[ mng_scope_intersect/5 ]).

:- rdf_meta(taxonomical_property(r)).
:- rdf_meta(propagate_tell(r)).
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
:- query_compiler:add_command(triple, [ask,tell]).

%%
% expose subject/predicate/object argument variables.
%
query_compiler:step_vars(triple(S,P,O), Ctx, Vars) :-
	% FIXME: it would be needed for terms at the moment that the requested type is stored
	%          to know that term_to_atom must be called during unification.
	bagof([Key, Var],
		(	triple_var(S, Ctx, [Key, Var])
		;	triple_var(P, Ctx, [Key, Var])
		;	triple_var(O, Ctx, [Key, Var])
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
	extend_context(triple(S,P,O), [P1,O1], Ctx, Ctx0),
	% compute steps of the aggregate pipeline
	findall(Step,
		% filter out documents that do not match the triple pattern.
		% this is done using $match or $lookup operators.
		(	lookup_triple(triple(S,P1,O1), Ctx0, Step)
		% conditionally needed to harmonize 'next' field
		;	harmonize_next(Ctx0, Step)
		% add additional results if P is a transitive property
		;	transitivity(Ctx0, Step)
		% add additional results if P is a reflexive property
		;	reflexivity(Ctx0, Step)
		% at this point 'next' field holds an array of matching documents
		% that is unwinded here.
		;	Step=['$unwind',string('$next')]
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
		;	set_triple_vars(S,P1,O1,Ctx0,Step)
		% remove next field again
		;	Step=['$unset',string('next')]
		;	Step=['$unset',string('start')]
		),
		Pipeline
	).

%%
% tell(triple(S,P,O)) uses $lookup to find matching triples
% with overlapping scope which are toggled to be removed in next stage.
% then the union of their scopes is computed and used for output document.
%
compile_tell(triple(S,P,O), Context, Pipeline) :-
	% add additional options to the compile context
	extend_context(triple(S,P,O), [P1,O1], Context, Context0),
	option(graph(Graph), Context0, user),
	option(scope(Scope), Context0),
	time_scope_data(Scope, [Since,Until]),
	% strip the value, assert that operator must be $eq
	mng_strip_variable(O1, V),
	mng_query_value(V, ['$eq', MngValue]),
	% build triple docuemnt
	% TODO: do all docs have o*?
	once(pstar_value(P1, Pstar)),
	once(ostar_value(P1, MngValue, Ostar)),
	TripleDoc=[
		['s', string(S)], ['p', string(P1)], ['o', MngValue],
		['p*', Pstar], ['o*', Ostar],
		['graph', string(Graph)],
		['scope', string('$v_scope')]
	],
	% configure the operation performed on scopes.
	% the default is to compute the union of scopes.
	(	option(intersect_scope, Context)
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
		;	delete_overlapping(TripleDoc, Context0, Step)
		% lookup parent documents into the 'parents' field
		;	lookup_parents(triple(S,P1,O1), Context0, Step)
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
				propagate_tell(S, Context0, Step)
			)
		),
		Pipeline
	).

%%
pstar_value(rdf:type,           array([string(rdf:type)])).
pstar_value(rdfs:subClassOf,    array([string(rdfs:subClassOf)])).
pstar_value(rdfs:subPropertyOf, array([string(rdfs:subPropertyOf)])).
pstar_value(_,                  string('$parents')).

%%
ostar_value(rdf:type,           _, string('$parents')).
ostar_value(rdfs:subClassOf,    _, string('$parents')).
ostar_value(rdfs:subPropertyOf, _, string('$parents')).
ostar_value(_,                  O, array([O])).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% LOOKUP triple documents
%%%%%%%%%%%%%%%%%%%%%%%

%%
lookup_triple(triple(S,P,O), Ctx, Step) :-
	memberchk(outer_vars(QueryVars), Ctx),
	memberchk(collection(Coll), Ctx),
	% get query pattern, and variables in term
	mng_triple_doc(triple(S,P,O), QueryDoc, Ctx),
	get_triple_vars(S,P,O,Ctx,TripleVars),
	% match triple pattern _and_ grounded variables (if any)
	(	lookup_join_vars(TripleVars, QueryVars, MatchDoc)
	->	Match=['$match', [['$expr', ['$and', array(MatchDoc)]] | QueryDoc ]]
	;	Match=['$match', QueryDoc]
	),
	% append $limit after $match if requested in Context
	(	member(limit(Limit),Ctx)
	->	Pipeline=[Match,['$limit',int(Limit)]]
	;	Pipeline=[Match]
	),
	% pass input document value to lookup
	query_compiler:lookup_let_doc(TripleVars, QueryVars, LetDoc),
	% lookup matching documents and store in 'next' field
    (	Step=['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['let',LetDoc],
			['pipeline', array(Pipeline)]
		]]
    % unwind the 'next' field
    ;	Step=['$unwind',string('$next')]
    ).

%%
lookup_join_vars(TripleVars, QueryVars, Doc) :-
	% find all joins with input documents
	findall([Field_j,Value_j],
		(	member([Field_j,Value_j], TripleVars),
			member([Field_j,_], QueryVars)
		),
		Joins),
	% perform the join operation (equals the input document value)
	findall(['$eq', array([string(Match_key),string(Match_val)])],
		% { $eq: [ "$s",  "$$R" ] },
		(	member([Join_var,Join_field],Joins),
			atom_concat('$',Join_field,Match_key),
			atom_concat('$$',Join_var,Match_val)
		),
		Doc),
	% make sure list is not empty
	Doc \== [].

%%
delete_overlapping(TripleDoc, Context,
		['$lookup', [
			['from',string(Coll)],
			['as',string('next')],
			['pipeline',array(Pipeline)]
		]]) :-
	option(collection(Coll), Context),
	% read triple data
	memberchk(['s',S], TripleDoc),
	memberchk(['p',P], TripleDoc),
	memberchk(['o',O], TripleDoc),
	% read scope data
	option(scope(Scope), Context),
	time_scope_data(Scope,[Since,Until]),
	% set Since=Until in case of scope intersection.
	% this is to limit to results that do hold at Until timestamp.
	% FIXME: when overlap yields no results, zero is used as since
	%        by until. but then the new document could overlap
	%        with existing docs, which is not wanted.
	%		 so better remove special handling here?
	%        but then the until time maybe is set to an unwanted value? 
	(	option(intersect_scope, Context)
	->	Since0=Until
	;	Since0=Since
	),
	% build pipeline
	findall(Step,
		% $match s,p,o and overlapping scope
		(	Step=['$match',[
				['s',S], ['p',P], ['o',O],
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
%%%%%%%%% PROPERTY SEMANTICS
%%%%%%%%%%%%%%%%%%%%%%%

%%
transitivity(Context, Step) :-
	% TODO: does it work if triple is ground? i.e. to find out if there is a path.
	% read options
	memberchk(transitive, Context),
	memberchk(collection(Coll), Context),
	memberchk(property(Property), Context),
	% yield steps
	(	Step=['$graphLookup', [
			['from',string(Coll)],
			['startWith',string('$next.o')],
			['connectFromField',string('o')],
			['connectToField',string('s')],
			['as',string('paths')],
			['restrictSearchWithMatch',['p*',string(Property)]]
		]]
	;	Step=['$addFields', ['paths', ['$concatArrays', array([
			string('$paths'),
			array([string('$next')])
		])]]]
	;	Step=['$set', ['start', string('$next')]]
	;	Step=['$set', ['next', string('$paths')]]
	% remove "paths" field again
	;	Step=['$unset', string('paths')]
	).

%%
% FIXME: this creates redundant results for the case of graph queries
%        that receive multiple documents with the same subject as input.
reflexivity(Context, Step) :-
	memberchk(reflexive,Context),
	% TODO: o* and p* missing, why not copy whole document instead?
	Step=['$set', ['next', ['$concatArrays',
		array([string('$next'), array([[
			['s',string('$start.s')],
			['p',string('$start.p')],
			['o',string('$start.s')],
			['scope',string('$start.scope')]
		]])])
	]]].

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
	;	Step=['$set', ['next.o', ['$next.o*']]]
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
		(	( mng_query_value(S,Query_s), X=['s',Query_s] )
		;	( mng_query_value(P,Query_p), X=[Key_p,Query_p] )
		;	( mng_query_value(V,Query_v), X=[Key_o,Query_v] )
		;	graph_doc(Graph,X)
		;	scope_doc(Scope,X)
		),
		Doc
	).

%%
graph_doc('*', _)    :- !, fail.
graph_doc('user', _) :- !, fail.
graph_doc(=(GraphName), ['graph',string(GraphName)]) :- !.
graph_doc(  GraphName,  ['graph',['$in',array(Graphs)]]) :-
	get_supgraphs(GraphName,Graphs).

%%
scope_doc(QScope, [Key,Value]) :-
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

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%

%%
taxonomical_property(P) :- var(P),!,fail.
taxonomical_property(rdf:type).
taxonomical_property(rdfs:subClassOf).
taxonomical_property(rdfs:subPropertyOf).

%%
extend_context(triple(_,P,O), [P1,O1], Context, Context0) :-
	% get the collection name
	(	option(collection(Coll), Context)
	;	mng_get_db(_DB, Coll, 'triples')
	),
	% read options from argument terms
	% e.g. properties can be wrapped in transitive/1 term to
	% indicate that the property is transitive which will then
	% generate an additional step in the aggregate pipeline.
	strip_property_modifier(P,P_opts,P1),
	strip_object_modifier(O,O_opts,O1),
	% extend the context
	findall(Opt,
		(	Opt=property(P1)
		;	Opt=collection(Coll)
		;	member(Opt, O_opts)
		;	member(Opt, P_opts)
		;	member(Opt, Context)
		),
		Context0).

%% this step is used to harmonize documents
harmonize_next(Context, Step) :-
	\+ memberchk(transitive,Context),
	(	% assign *start* field in case of reflexive property
		(	memberchk(reflexive,Context),
			Step=['$set', ['start', string('$next')]]
		)
	;	% transform next into single-element array
		Step=['$set', ['next', array([string('$next')])]]
	).

%%
triple_var(Arg, Ctx, [Key, Var]) :-
	% strip variable
	% TODO: left side of -> could also have var? e.g. `in(List)->Elem`
	once((
		( nonvar(Arg), Arg=(_->Var0) )
	;	Var0=Arg
	)),
	strip_modifier(Var0, Var1),
	mng_strip(Var1, _, _, Var),
	query_compiler:var_key(Var, Ctx, Key).

%%
get_triple_vars(S, P, O, Ctx, Vars) :-
	findall([Key,Field],
		(	member([Field,Arg], [[s,S],[p,P],[o,O]]),
			triple_var(Arg, Ctx, [Key, _Var])
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
strip_modifier(Term, Stripped) :-
	strip_object_modifier(Term, _, Stripped0),
	strip_property_modifier(Stripped0, _, Stripped).

%%
strip_object_modifier(Var,[],Var) :- var(Var), !.
strip_object_modifier(Term,[X|Xs],Stripped) :-
	strip_object_modifier1(Term,X,Term0), !,
	strip_object_modifier(Term0,Xs,Stripped).
strip_object_modifier(Stripped,[],Stripped).

strip_object_modifier1(include_parents(X), ostar, X).

%%
strip_property_modifier(Var,[],Var) :- var(Var), !.
strip_property_modifier(Term,[X|Xs],Stripped) :-
	strip_property_modifier1(Term,X,Term0), !,
	strip_property_modifier(Term0,Xs,Stripped).
strip_property_modifier(Stripped,[],Stripped).

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
