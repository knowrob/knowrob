:- module(mongolog_context, []).
/** <module> Accessing compile-context in mongolog programs.

The following predicates are supported:

| Predicate             | Arguments |
| ---                   | ---       |
| context/1             | -Option |
| context/2             | -Option, +Default |
| call_with_context/2   | +Goal, +Options |

@author Daniel Beßler
@license BSD
*/

:- use_module(library('mongodb/client')).
:- use_module(library('scope'),
		[ time_scope/3 ]).
:- use_module('mongolog').

:- mongolog:add_command(call_with_context).
:- mongolog:add_command(context).

%%
mongolog:step_expand(
		call_with_context(Goal,Args),
		call_with_context(Expanded,Args)) :-
	mongolog_expand(Goal, Expanded).

%%
% context(-Option) and context(-Option, +Default) are used to read
% options from compile context to make them accessible in rules.
% The main usecase is that some temporal predicates need to access
% the query scope.
%
mongolog:step_compile(context(Option), Ctx, []) :-
	option(Option, Ctx).

mongolog:step_compile(context(Option, Default), Ctx, []) :-
	option(Option, Ctx, Default).

%%
%
mongolog:step_compile(
		call_with_context(Terminals, NewCtx),
		OldCtx, Pipeline, StepVars) :-
	option(outer_vars(V0), OldCtx),
	% resolve since/until values.
	% this is needed if the values are grounded within the query
	% by mongo DB and not provided in the call context.
	resolve_scope(NewCtx, OldCtx, NewCtx0),
	% add provided options to context
	merge_options(NewCtx0, OldCtx, Ctx),
	% finally compile call goal with new context
	mongolog:compile_terms(
		Terminals, Pipeline,
		V0->_, StepVars, Ctx).

%%
% variables maybe used in the scope.
% if this is the case, they must be replaced by variable keys to be referred to in queries.
%
% TODO: is it sufficient to do this only for call_with_context/2? maybe it should also be
%       done by mongolog_call predicate?
% TODO: move to scope.pl
%
resolve_scope(In, Ctx, [query_scope(Scope1)|Rest]) :-
	select_option(query_scope(Scope0),In,Rest),!,
	resolve_scope1(Scope0, Ctx, Scope1).

resolve_scope1(DictIn, Ctx, DictOut) :-
	is_dict(DictIn),!,
	findall(Key-Rewritten,
		(	get_dict(Key,DictIn,Val),
			resolve_scope1(Val, Ctx, Rewritten)
		), Pairs),
	dict_pairs(DictOut, dict, Pairs).

resolve_scope1(In, Ctx, Out) :-
	mng_strip_operator(In, Operator, Val1),
	mongolog:var_key_or_val(Val1, Ctx, Val2),
	mng_strip_operator(Out, Operator, Val2).
