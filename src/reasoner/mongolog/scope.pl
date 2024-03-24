:- module(mongolog_scope,
	[ mongolog_universal_scope/1,
	  mongolog_scope_doc/2,
	  mongolog_scope_intersect/5,
	  mongolog_scope_is_valid/1,
	  mongolog_scope_match/2,
	  mongolog_time_scope/3,
	  mongolog_resolve_scope/3
	]).

:- use_module('client',
        [ mng_strip_operator/3,
          mng_typed_value/2
        ]).

%% mongolog_universal_scope(-Scope) is det.
%
% The scope of facts that are universally true.
%
% @param Scope A scope dictionary.
%
mongolog_universal_scope(dict{
	epistemicMode: knowledge,
	temporalMode: always
}).

%% mongolog_scope_is_valid(-Pipeline) is det.
%
% Only proceed in the current pipeline if the acccumulated scope
% so far is valid, i.e. if there can be records that satisfy it.
% It is assumed that the accumulated scope so far is stored in
% input documents under the key "v_scope".
%
mongolog_scope_is_valid(['$match',
    ['$expr', ['$lt', array([
        string('$v_scope.time.since'),
        string('$v_scope.time.until')
    ])]]]).

%%
% Generates a scope JSON document intended to be used in $match operations.
% Note that the scope may contain runtime variables that cannot be included
% in $match operations, thes are not included in the document!
% It is further expected that variables in the scope are replaced by terms
% using functor string/1 that hold the name of the variable in input documents.
%
mongolog_scope_doc(QScope, [Key,Value]) :-
	scope_doc1(QScope, [Key,Value]),
	% do not proceed for variables in scope these are handled later
	(( Value=array([_,[_,[_,string(VarKey)]]]),
	   atom_concat('$',_,VarKey) ) -> fail ; true).

scope_doc1(QScope, ['$or', array([
		[Key, ['$exists', bool(false)]],
		[Key, Value]
	])]) :-
	get_dict(ScopeName, QScope, ScopeData),
	scope_doc(ScopeName, ScopeData, Key, Value).

scope_doc(epistemicMode, ScopeVal, uncertain, DocVal) :-
	epistemic_doc(ScopeVal, DocVal),!.
scope_doc(agent, ScopeVal, agent, ['$eq', Typed]) :-
	mng_typed_value(ScopeVal, Typed).
scope_doc(confidence, ScopeVal, confidence, ['$gte', Typed]) :-
	mng_typed_value(ScopeVal, Typed).

scope_doc(temporalMode, ScopeVal, occasional, DocVal) :-
	temporal_doc(ScopeVal, DocVal), !.
scope_doc(since, ScopeVal, 'scope.time.since', ['$lte', Typed]) :-
	mng_typed_value(ScopeVal, Typed).
scope_doc(until, ScopeVal, 'scope.time.until', ['$gte', Typed]) :-
	mng_typed_value(ScopeVal, Typed).

%%
epistemic_doc(belief, _) :- !, fail.
epistemic_doc(knowledge, ['$eq', bool(false)]).
epistemic_doc('B', Val) :- epistemic_doc(belief, Val).
epistemic_doc('K', Val) :- epistemic_doc(knowledge, Val).
epistemic_doc(string(ScopeVal), Val) :- epistemic_doc(ScopeVal, Val).
epistemic_doc(string(ScopeVal), ['$eq', string(ScopeVal)]).

%%
temporal_doc(sometimes, _) :- !, fail.
temporal_doc(always, ['$eq', bool(false)]).
temporal_doc('H', Val) :- temporal_doc(always, Val).
temporal_doc('P', Val) :- temporal_doc(sometimes, Val).
temporal_doc(string(ScopeVal), Val) :- temporal_doc(ScopeVal, Val).
temporal_doc(string(ScopeVal), ['$eq', string(ScopeVal)]).

%% mongolog_scope_match(+Ctx, -Pipeline) is semidet.
%
% Generates an $expr command to match all runtime variables
% in the scope.
%
mongolog_scope_match(Ctx, ['$expr', ['$and', array(List)]]) :-
	option(query_scope(Scope), Ctx),
	findall(
		% [Operator, array([string(ScopeValue),string(Val)])],
		['$or', array([
			['$not', array([string(ScopeValue)])],
			[Operator, array([string(ScopeValue),string(Val)])]
		])],
		% only include variables, constants are handled earlier
		(	scope_doc1(Scope, [ScopeKey,[Operator,string(Val0)]]),
			atom_concat('$',_,Val0),
			atom_concat('$',ScopeKey,ScopeValue),
			atom_concat('$',Val0,Val)
		),
		List
	),
	List \== [].

%% mongolog_scope_intersect(+VarKey, +Since1, +Until1, +Options, -Step) is nondet.
%
% The step expects input documents with VarKey field, and another field Since1/Until1.
% The step uses these field to compute an intersection beteween both scopes.
% It will fail in case the intersection is empty.
%
mongolog_scope_intersect(VarKey, Since1, Until1, Options, Step) :-
	% TODO: remember if the solution is uncertain
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
	;	Step=['$match', ['$expr', ['$lt', array([string(Since0), string(Until0)])] ]]
	).

%% mongolog_time_scope(+Scope, ?Since, ?Until) is semidet.
%
% Read since and until values from query scope.
% Currently silently proceeds if scope only restricts since/until
% to a range. If since/until are not defined the predicate still succeeds
% but leaves the arguments as free variables.
%
mongolog_time_scope(Scope, SinceValue, UntilValue) :-
	time_scope_data(Scope, [Since,Until]),
	time_scope_value1(Since, SinceValue),
	time_scope_value1(Until, UntilValue).

time_scope_value1(V0, Value) :-
	mng_strip_operator(V0, _, V1),
	once(time_scope_value2(V1, Value)).

time_scope_value2(Inf,  double('Infinity')) :- Inf == 'Infinity'.
time_scope_value2(Num,  double(Num))        :- number(Num).
time_scope_value2(Atom, string(Atom))       :- atom(Atom).
time_scope_value2(Val, Typed) :- mng_typed_value(Val, Typed).

%%
% Read since/until pair of temporal scope.
% Note that vars are used in case since or until
% not known.
%
time_scope_data(Scope,[Since,Until]) :-
	get_dict(since,Scope,Since),
	get_dict(until,Scope,Until).

%%
% variables maybe used in the scope.
% if this is the case, they must be replaced by variable keys to be referred to in queries.
%
mongolog_resolve_scope(In, Ctx, [query_scope(Scope1)|Rest]) :-
	select_option(query_scope(Scope0),In,Rest),!,
	resolve_scope1(Scope0, Ctx, Scope1).

resolve_scope1(DictIn, Ctx, DictOut) :-
	is_dict(DictIn),!,
	findall(Key-Rewritten,
		(	get_dict(Key,DictIn,Val),
			resolve_scope1(Val, Ctx, Rewritten)
		), Pairs),
	dict_pairs(DictOut, dict, Pairs).

resolve_scope1(In, Ctx, Out) :-
	mng_strip_operator(In, Operator, Val1),
	mongolog:var_key_or_val(Val1, Ctx, Val2),
	mng_strip_operator(Out, Operator, Val2).
