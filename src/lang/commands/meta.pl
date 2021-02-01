:- module(meta_commands, []).

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module(library('db/mongo/client'),
		[ mng_strip/4 ]).
:- use_module(library('lang/compiler')).
:- use_module(library('lang/query')).

%%%% query commands
:- query_compiler:add_command(call,   [ask,tell]).
:- query_compiler:add_command(once,   [ask,tell]).
:- query_compiler:add_command(limit,  [ask,tell]).
:- query_compiler:add_command(ignore, [ask,tell]).

:- query_compiler:add_command(call_with_context, [ask,tell]).

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
query_compiler:step_expand(
		call(Goal),
		call(Expanded), Context) :-
	query_expand(Goal, Expanded, Context).

query_compiler:step_expand(
		call_with_context(Goal,Args),
		call_with_context(Expanded,Args), Context) :-
	query_expand(Goal, Expanded, Context).

%%
query_compiler:step_var(limit(_Count, Terminals), Ctx, Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Ctx, Var).

query_compiler:step_var(limit(Count, _Terminals), Ctx, Var) :-
	query_compiler:get_var([Count], Ctx, Var).

query_compiler:step_var(call(Terminals), Ctx, Var) :-
	ensure_list(Terminals,List),
	member(X,List),
	compound(X),
	query_compiler:step_var(X, Ctx, Var).

query_compiler:step_var(call_with_context(Terminals, _Context), Ctx, Var) :-
	ensure_list(Terminals,List),
	member(X,List),
	compound(X),
	query_compiler:step_var(X, Ctx, Var).

query_compiler:step_var(call_with_context(_Terminals, Context), Ctx, Var) :-
	% only scope values may be variables
	option(scope(Scope), Context),
	time_scope(Since, Until, Scope),
	member(Value, [Since,Until]),
	query_compiler:get_var([Value], Ctx, Var).

%%
ensure_list([X|Xs],[X|Xs]) :- !.
ensure_list(X,[X]).

%% limit(+Count, :Goal)
% Limit the number of solutions.
% True if Goal is true, returning at most Count solutions.
%
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
		Pipeline).

%% call(:Goal)
% Call Goal. This predicate is normally used for goals that are not known at compile time.
%
query_compiler:step_compile(
		call(Terminals), Ctx, Pipeline) :-
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			[], [], Ctx, Step),
		Pipeline).

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
% variables maybe used in the scope.
% if this is the case, they must be replaces by variable
% keys to be referred to in queries.
%
resolve_scope(In, Ctx, [scope(Scope1)|Rest]) :-
	select_option(scope(Scope0),In,Rest),!,
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	query_compiler:var_key_or_val(Since0,Ctx,Since1),
	query_compiler:var_key_or_val(Until0,Ctx,Until1).
resolve_scope(In, _, In).

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

test('once(+Goal)'):-
	lang_query:test_command(
		once((
			(X is (Num + 5))
		;	(X is (Num * 2))
		)),
		Num, double(4.5)),
	assert_equals(X,9.5).

test('limit(1, +Goal)'):-
	lang_query:test_command(
		limit(1, (
			(X is (Num + 5))
		;	(X is (Num * 2))
		)),
		Num, double(4.5)),
	assert_equals(X,9.5).

test('limit(2, +Goal)'):-
	findall(X,
		lang_query:test_command(
			limit(2, (
				(X is (Num + 5))
			;	(X is (Num * 2))
			;	(X is (Num * 2))
			)),
			Num, double(4.5)),
		Results
	),
	assert_unifies(Results,[_,_]),
	assert_true(memberchk(9.5, Results)),
	assert_true(memberchk(9.0, Results)).

test('ignore(+Failing)'):-
	lang_query:test_command(
		ignore(((Num < 3), (X is (Num * 2)))),
		Num, double(4.5)),
	assert_unifies(X,_).

test('ignore(+Failing), +Goal'):-
	lang_query:test_command(
		(ignore(Num < 3), (X is (Num * 2))),
		Num, double(4.5)),
	assert_equals(X,9.0).

test('call(+Goal)'):-
	lang_query:test_command(
		call(Y is X), X, double(-3.25)),
	assert_equals(Y, -3.25).

:- end_tests('meta_commands').
