:- module(mng_term_match,
    [ match_operator/2
    ]).

:- use_module(library('db/mongo/lang/compiler')).
:- use_module(library('db/mongo/lang/query')).

%% register query commands
:- mng_query_command(match(_)).

%%
% Operators that can be used in match/1 queries.
%
match_operator('=',		'$eq').
match_operator('\=',	'$ne').
match_operator('>=',	'$gte').
match_operator('=<',	'$lte').
match_operator('>',		'$gt').
match_operator('<',		'$lt').
match_operator('in',	'$in').
match_operator('nin',	'$nin').

%%
% match uses 2-ary operator whose arguments maybe variables.
%
mng_compiler:step_var(match(Expr), [VarKey, Var]) :-
	Expr =.. [_Functor, A, B],
	( Var=A ; Var=B ),
	mng_compiler:var_key(Var, VarKey).

%%
% match(Expr) uses $match operator on existing variables
% and constants.
% Expr is a 2-ary term, and the functor can be one of:
%	=, \=, >=, =<, >, <, in, nin
%
mng_compiler:step_compile(
		match(Expr),
		Context,
		[['$match', [
			[X, [A, B]]
		]]]) :-
	option(ask, Context),!,
	% unpack expression
	Expr =.. [X0,A0,B0],
	match_operator(X0, X),
	% 
	mng_compiler:var_key_or_val(A0, A),
	mng_compiler:var_key_or_val(B0, B).

