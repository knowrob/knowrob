:- module(lang_call,
    [ match_operator/2
    ]).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

%% register query commands
:- add_query_command(call).

%%
query_compiler:step_expand(
		call(Goal, Scope),
		call(Expanded, Scope),
		Context) :-
	mng_expand(Goal, Expanded, Context).

%%
query_compiler:step_var(call(Terminals, _Scope), Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Var).

query_compiler:step_var(call(_Terminals, Scope), Var) :-
	time_scope(Since, Until, Scope),
	member(X, [Since,Until]),
	mng_strip(X, _Operator, _Type, Y),
	query_compiler:step_var(Y, Var).

%%
query_compiler:step_compile(
		call(Terminals, Scope),
		Context,
		Pipeline) :-
	% get since/until values
	time_scope(Since, Until, Scope),
	query_compiler:var_key_or_val(Since,Since0),
	query_compiler:var_key_or_val(Until,Until0),
	time_scope(Since0, Until0, Scope0),
	% remove previous scope from context
	select_option(scope(_), Context, Context0),
	% finally compile called goal
	% and replace the scope in compile context
	query_compile(Terminals,
		Pipeline,
		[scope(Scope0)|Context0]).
