:- module(lang_match, []).

:- use_module(library('db/mongo/client'),
		[ mng_operator/2 ]).
:- use_module(library('lang/compiler')).

%% register query commands
:- query_compiler:add_command(match, [ask]).
:- query_compiler:add_command(set,   [ask,tell]).

%%
% match uses 2-ary operator whose arguments maybe variables.
%
query_compiler:step_var(match(Expr), Ctx, [VarKey, Var]) :-
	Expr =.. [_Functor, A, B],
	(	Var=A
	;	Var=B
	),
	query_compiler:var_key(Var, Ctx, VarKey).

query_compiler:step_var(set(A,_Val), Ctx, Var) :-
	query_compiler:get_var(A, Ctx, Var).

%%
% match(Expr) uses $match operator on existing variables
% and constants.
% Expr is a 2-ary term, and the functor can be one of:
%	=, \=, >=, =<, >, <, in, nin
%
query_compiler:step_compile(
		match(Expr), Ctx,
		[['$match', [
			[X, [A, B]]
		]]]) :-
	% option(mode(ask), Context),
	% unpack expression
	Expr =.. [X0,A0,B0],
	mng_operator(X0, X),
	% 
	query_compiler:var_key_or_val(A0, Ctx, A),
	query_compiler:var_key_or_val(B0, Ctx, B).

%%
query_compiler:step_compile(
		set(Var,Value), Ctx,
		[['$set', [[Key,Value]]]]) :-
	query_compiler:var_key(Var,Ctx,Key).

