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
query_compiler:step_var(limit(_Count, Terminals), Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Var).

query_compiler:step_var(limit(Count, _Terminals), Var) :-
	query_compiler:get_var([Count], Var).

query_compiler:step_var(call(Terminals), Var) :-
	ensure_list(Terminals,List),
	member(X,List),
	compound(X),
	query_compiler:step_var(X, Var).

query_compiler:step_var(call_with_context(Terminals, _Context), Var) :-
	ensure_list(Terminals,List),
	member(X,List),
	compound(X),
	query_compiler:step_var(X, Var).

query_compiler:step_var(call_with_context(_Terminals, Context), Var) :-
	% currently only scope values may be grounded in a query
	option(scope(Scope), Context),
	time_scope(Since, Until, Scope),
	member(Value, [Since,Until]),
	query_compiler:get_var([Value], Var).

%%
ensure_list(List,List) :- is_list(List), !.
ensure_list(X,[X]).

%% limit(+Count, :Goal)
% Limit the number of solutions.
% True if Goal is true, returning at most Count solutions.
%
query_compiler:step_compile(
		limit(Count, Terminals),
		Context, Pipeline) :-
	query_compiler:var_key_or_val(Count,Count0),
	% appended to inner pipeline of lookup
	Prefix=[],
	Suffix=[['$limit',Count0]],
	% create a lookup and append $limit to inner pipeline,
	% then unwind next and assign variables to the toplevel document.
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			Prefix, Suffix, Context, Step),
		Pipeline).

%% call(:Goal)
% Call Goal. This predicate is normally used for goals that are not known at compile time.
%
query_compiler:step_compile(
		call(Terminals), Context, Pipeline) :-
	findall(Step,
		query_compiler:lookup_next_unwind(Terminals,
			[], [], Context, Step),
		Pipeline).

%%
%
query_compiler:step_compile(
		call_with_context(Terminals, NewContext),
		OldContext, Pipeline) :-
	option(outer_vars(V0), OldContext),
	% resolve since/until values.
	% this is needed if the values are grounded within the query
	% by mongo DB and not provided in the call context.
	resolve_scope(NewContext, NewContext0),
	% add provided options to context
	merge_options(NewContext0, OldContext, Context),
	% finally compile call goal with new context
	(	option(mode(tell),Context) -> Goal=Terminals
	;	Goal=call(Terminals)
	),
	query_compiler:compile_terms(
		Goal, Pipeline,
		V0->_, Context).

%%
resolve_scope(In, [scope(Scope1)|Rest]) :-
	select_option(scope(Scope0),In,Rest),!,
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	query_compiler:var_key_or_val(Since0,Since1),
	query_compiler:var_key_or_val(Until0,Until1).
resolve_scope(In, In).

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% helper
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
append_cut(Goal, WithCut) :-
	(	is_list(Goal) -> Terms=Goal
	;	comma_list(Goal,Terms)
	),
	append(Terms, ['!'], X),
	comma_list(WithCut, X).
