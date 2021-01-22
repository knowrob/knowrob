:- module(lang_match,
    []).

:- use_module(library('db/mongo/compiler')).
:- use_module(library('db/mongo/query')).

%% register query commands
:- query_command_add(match).

%%
% match uses 2-ary operator whose arguments maybe variables.
%
query_compiler:step_var(match(Expr), [VarKey, Var]) :-
	Expr =.. [_Functor, A, B],
	( Var=A ; Var=B ),
	query_compiler:var_key(Var, VarKey).

%%
% match(Expr) uses $match operator on existing variables
% and constants.
% Expr is a 2-ary term, and the functor can be one of:
%	=, \=, >=, =<, >, <, in, nin
%
query_compiler:step_compile(
		match(Expr),
		Context,
		[['$match', [
			[X, [A, B]]
		]]]) :-
	option(ask, Context),!,
	% unpack expression
	Expr =.. [X0,A0,B0],
	mng_operator(X0, X),
	% 
	query_compiler:var_key_or_val(A0, A),
	query_compiler:var_key_or_val(B0, B).

