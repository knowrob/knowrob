:- module(lang_equality, []).

:- use_module(library('lang/compiler')).

%% query commands
:- query_compiler:add_command( ==, [ask]).
:- query_compiler:add_command(\==, [ask]).

%% query variables
query_compiler:step_var( ==(X,Y), Var) :- query_compiler:get_var([X,Y],Var).
query_compiler:step_var(\==(X,Y), Var) :- query_compiler:get_var([X,Y],Var).

%% query compilation
query_compiler:step_compile(==(X,Y), _,
		[['$match', ['$eq', array([X0,Y0])]]]) :-
	query_compiler:var_key_or_val(X,X0),
	query_compiler:var_key_or_val(Y,Y0).

query_compiler:step_compile(\==(X,Y), _,
		[['$match', ['$ne', array([X0,Y0])]]]) :-
	query_compiler:var_key_or_val(X,X0),
	query_compiler:var_key_or_val(Y,Y0).
