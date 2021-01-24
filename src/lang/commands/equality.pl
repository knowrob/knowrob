:- module(lang_equality, []).

:- use_module(library('lang/compiler')).

% TODO: term_variable, term_value

%% query commands
:- query_compiler:add_command( ==, [ask]).
:- query_compiler:add_command(\==, [ask]).

%% query variables
query_compiler:step_var( ==(X,Y), Var) :- lang_terms:term_variable([X,Y],Var).
query_compiler:step_var(\==(X,Y), Var) :- lang_terms:term_variable([X,Y],Var).

%% query compilation
query_compiler:step_compile(==(X,Y), _,
		[['$match', ['$eq', array([X0,Y0])]]]) :-
	lang_terms:term_value(X,X0),
	lang_terms:term_value(Y,Y0).

query_compiler:step_compile(==(X,Y), _,
		[['$match', ['$ne', array([X0,Y0])]]]) :-
	lang_terms:term_value(X,X0),
	lang_terms:term_value(Y,Y0).
