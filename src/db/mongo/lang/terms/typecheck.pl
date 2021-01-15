:- module(mng_term_typecheck, []).

:- use_module(library('db/mongo/lang/compiler')).
:- use_module(library('db/mongo/lang/query')).

%% register query commands
:- mng_query_command(ground).
:- mng_query_command(var).

%%
mng_compiler:step_var(ground(X), [V,X]) :- mng_compiler:var_key(X, V).
mng_compiler:step_var(var(X),    [V,X]) :- mng_compiler:var_key(X, V).

%%
% ground(X) holds iff a previous step has assigned
% a value to the variable key.
%
mng_compiler:step_compile(
		ground(A0),
		Context,
		[['$match', [
			[A, ['$exists', bool(true)]]
		]]]) :-
	option(ask, Context),!,
	mng_compiler:var_key(A0, A).

%%
% var(X) holds iff no previous step has assigned
% a value to the variable key.
%
mng_compiler:step_compile(
		var(A0),
		Context,
		[['$match', [
			[A, ['$exists', bool(false)]]
		]]]) :-
	option(ask, Context),!,
	mng_compiler:var_key(A0, A).
