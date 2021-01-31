:- module(lang_equality, []).

:- use_module(library('lang/compiler')).

%% query_compiler:add_command
:- query_compiler:add_command( ==, [ask]).
:- query_compiler:add_command(\==, [ask]).
% TODO: support unification operator =\2
%:- query_compiler:add_command(  =, [ask]).
%:- query_compiler:add_command( \=, [ask]).

%% @Term1 \= @Term2
%
% Equivalent to \+Term1 = Term2.
%
% This predicate is logically sound if its arguments are sufficiently instantiated.
% In other cases, such as ?- X \= Y., the predicate fails although there are solutions.
% This is due to the incomplete nature of \+/1. 
%
query_compiler:step_expand(\=(A,B), Expanded, Mode) :-
	query_compiler:step_expand(\+(=(A,B)), Expanded, Mode).

%%
query_compiler:step_expand(
		=(Term1,Term2),
		=(Expanded1,Expanded2), Mode) :-
	query_compiler:step_expand(Term1,Expanded1,Mode),
	query_compiler:step_expand(Term2,Expanded2,Mode).


%% query_compiler:step_var
query_compiler:step_var( ==(X,Y), Var) :- query_compiler:get_var([X,Y],Var).
query_compiler:step_var(\==(X,Y), Var) :- query_compiler:get_var([X,Y],Var).

%%
query_compiler:step_var(=(Term1, Term2), Var) :-
	query_compiler:get_var([Term1, Term2],Var).


%% query_compiler:step_compile
query_compiler:step_compile(==(X,Y), _, []) :-
	ground([X,Y]),!,
	X == Y.

query_compiler:step_compile(==(X,Y), _,
		[['$match', ['$eq', array([X0,Y0])]]]) :-
	query_compiler:var_key_or_val(X,X0),
	query_compiler:var_key_or_val(Y,Y0).

query_compiler:step_compile(\==(X,Y), _, []) :-
	ground([X,Y]),!,
	X \== Y.

query_compiler:step_compile(\==(X,Y), _,
		[['$match', ['$ne', array([X0,Y0])]]]) :-
	query_compiler:var_key_or_val(X,X0),
	query_compiler:var_key_or_val(Y,Y0).

%% ?Term1 = ?Term2
% Unify Term1 with Term2. True if the unification succeeds.
%
%query_compiler:step_compile(=(Term1, Term2), Context, Pipeline) :-
%	fail.
	