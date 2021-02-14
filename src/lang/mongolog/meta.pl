:- module(meta_commands, []).

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module(library('db/mongo/client'),
		[ mng_strip/4 ]).
:- use_module(library('lang/compiler')).
:- use_module(library('lang/query')).

%%%% query commands
:- query_compiler:add_command(call).
:- query_compiler:add_command(once).
:- query_compiler:add_command(limit).
:- query_compiler:add_command(ignore).
:- query_compiler:add_command(call_with_args).
:- query_compiler:add_command(call_with_context).
% TODO: move these to somewhere else
:- query_compiler:add_command(set).
:- query_compiler:add_command(pragma).
:- query_compiler:add_command(context).
:- query_compiler:add_command(ask).

%%%% query expansion
	

%% once(:Goal)
% Make a possibly nondet goal semidet, i.e., succeed at most once.
%
query_compiler:step_expand(
		once(Goal), Expanded, Context) :-
	append_cut(Goal, WithCut),
	query_expand(WithCut, Expanded, Context).

%% ignore(:Goal)
% Calls Goal as once/1, but succeeds, regardless of whether Goal succeeded or not.
%
query_compiler:step_expand(
		ignore(Goal), Expanded, Context) :-
	append_cut(Goal, WithCut),
	query_expand((WithCut ; true), Expanded, Context).

%%
query_compiler:step_expand(
		limit(Count, Goal),
		limit(Count, Expanded),
		Context) :-
	query_expand(Goal, Expanded, Context).

%%
query_compiler:step_expand(call(Goal), call(Expanded), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(call(Goal,Arg1),
		call_with_args(Expanded,[Arg1]), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(call(Goal,Arg1,Arg2),
		call_with_args(Expanded,[Arg1,Arg2]), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(call(Goal,Arg1,Arg2,Arg3),
		call_with_args(Expanded,[Arg1,Arg2,Arg3]), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(call(Goal,Arg1,Arg2,Arg3,Arg4),
		call_with_args(Expanded,[Arg1,Arg2,Arg3,Arg4]), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(
		call_with_context(Goal,Args),
		call_with_context(Expanded,Args), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(ask(Goal), ask(Expanded), _Context) :-
	query_expand(Goal, Expanded, ask).

%%
ensure_list([X|Xs],[X|Xs]) :- !.
ensure_list(X,[X]).

%%
% ignore vars referred to in pragma as these are handled compile-time.
% only the ones also referred to in parts of the query are added to the document.
%
query_compiler:step_vars(pragma(_),_,[]).

%% limit(+Count, :Goal)
% Limit the number of solutions.
% True if Goal is true, returning at most Count solutions.
%
query_compiler:step_compile(
		limit(_, Terminals), _Ctx, []) :-
	is_list(Terminals),
	Terminals=[],
	!.

query_compiler:step_compile(
		limit(Count, Terminals),
		Ctx, Pipeline) :-
	query_compiler:var_key_or_val(Count,Ctx,Count0),
	% appended to inner pipeline of lookup
	Prefix=[],
	Suffix=[['$limit',Count0]],
	% create a lookup and append $limit to inner pipeline,
	% then unwind next and assign variables to the toplevel document.
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			Prefix, Suffix, Ctx, Step),
		Pipeline),
	% the inner goal of limit is not satisfiable if Pipeline==[]
	Pipeline \== [].

%% call(:Goal)
% Call Goal. This predicate is normally used for goals that are not known at compile time.
%
query_compiler:step_compile(
		call(Terminals), Ctx, Pipeline) :-
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			[], [], Ctx, Step),
		Pipeline).

%% call_with_args(:Goal,:Args)
% Call Goal. This predicate is normally used for goals that are not known at compile time.
%
query_compiler:step_compile(
		call_with_args(Term0,Args), Ctx, Pipeline) :-
	Term0 =.. Buf0,
	append(Buf0, Args, Buf1),
	Term1 =.. Buf1,
	query_compiler:step_compile(call(Term1), Ctx, Pipeline).

%%
%
query_compiler:step_compile(
		call_with_context(Terminals, NewCtx),
		OldCtx, Pipeline) :-
	option(outer_vars(V0), OldCtx),
	% resolve since/until values.
	% this is needed if the values are grounded within the query
	% by mongo DB and not provided in the call context.
	resolve_scope(NewCtx, OldCtx, NewCtx0),
	% add provided options to context
	merge_options(NewCtx0, OldCtx, Ctx),
	% finally compile call goal with new context
	(	option(mode(tell),Ctx) -> Goal=Terminals
	;	Goal=call(Terminals)
	),
	query_compiler:compile_terms(
		Goal, Pipeline,
		V0->_, Ctx).

%%
query_compiler:step_compile(
		set(Var,Value), Ctx,
		[['$set', [[Key,Value0]]]]) :-
	query_compiler:var_key_or_val(Value, [], Value0),
	query_compiler:var_key(Var,Ctx,Key).

%%
% pragma(Goal) is evaluated compile-time by calling
% the Goal. This is usually done to unify variables
% used in the aggregation pipeline from the compile context.
%
query_compiler:step_compile(pragma(Goal), _, []) :-
	call(Goal).

%%
% context(-Option) and context(-Option, +Default) are used to read
% options from compile context to make them accessible in rules.
% The main usecase is that some temporal predicates need to access
% the query scope.
%
query_compiler:step_compile(context(Option), Ctx, []) :-
	option(Option, Ctx).

query_compiler:step_compile(context(Option, Default), Ctx, []) :-
	option(Option, Ctx, Default).

%% ask(:Goal)
% Call Goal in ask mode.
%
query_compiler:step_compile(ask(Goal), Ctx, Pipeline) :-
	merge_options([mode(ask)], Ctx, Ctx0),
	query_compiler:step_compile(call(Goal), Ctx0, Pipeline).

%%
% variables maybe used in the scope.
% if this is the case, they must be replaces by variable
% keys to be referred to in queries.
%
resolve_scope(In, Ctx, [scope(Scope1)|Rest]) :-
	select_option(scope(Scope0),In,Rest),!,
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	resolve_scope1(Since0,Ctx,Since1),
	resolve_scope1(Until0,Ctx,Until1).
resolve_scope(In, _, In).

%%
resolve_scope1(In, Ctx, Out) :-
	mng_strip_operator(In, Operator, Time1),
	query_compiler:var_key_or_val(Time1, Ctx, Time2),
	mng_strip_operator(Out, Operator, Time2).
	

%%
% append cut operator at the end of a goal.
%
append_cut(Goal, WithCut) :-
	(	is_list(Goal) -> Terms=Goal
	;	comma_list(Goal,Terms)
	),
	append(Terms, ['!'], X),
	comma_list(WithCut, X).

		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

:- begin_tests('meta_commands').

test('limit(1, +Goal)'):-
	lang_query:test_command(
		limit(1, (
			(X is (Num + 5))
		;	(X is (Num * 2))
		)),
		Num, 4.5),
	assert_equals(X,9.5).

test('limit(2, +Goal)'):-
	findall(X,
		lang_query:test_command(
			limit(2, (
				(X is (Num + 5))
			;	(X is (Num * 2))
			;	(X is (Num * 2))
			)),
			Num, 4.5),
		Results
	),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5, Results)),
	assert_true(memberchk(9.0, Results)).

test('once(+Goal)'):-
	lang_query:test_command(
		once((
			(X is (Num + 5))
		;	(X is (Num * 2))
		)),
		Num, 4.5),
	assert_equals(X,9.5).

test('ignore(+Failing)'):-
	lang_query:test_command(
		ignore(((Num < 3), (X is (Num * 2)))),
		Num, 4.5),
	assert_unifies(X,_).

test('ignore(+Failing), +Goal'):-
	lang_query:test_command(
		(ignore(Num < 3), (X is (Num * 2))),
		Num, 4.5),
	assert_equals(X,9.0).

test('ignore(+FailingWithVar), +Goal'):-
	% test with variable Z being assigned only in failing ignored
	% goal. Then no grounding will be part of result set and Z still a variable
	% after the call.
	lang_query:test_command(
		(	ignore((Num < 3, Z is Num + 2)),
			X is (Num * 2)
		),
		Num, 4.5),
	assert_equals(X,9.0),
	assert_unifies(Z,_).

test('call(+Goal)'):-
	lang_query:test_command(
		call(Y is X), X, -3.25),
	assert_equals(Y, -3.25).

test('call(+Functor, -Arg1, +Arg2)'):-
	lang_query:test_command(
		call(is, Arg1, Arg2), Arg2, -3.25),
	assert_equals(Arg1, -3.25).

:- end_tests('meta_commands').
