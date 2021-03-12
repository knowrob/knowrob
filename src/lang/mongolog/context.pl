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

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module('mongolog').

:- mongolog:add_command(call_with_context).
:- mongolog:add_command(context).

%%
mongolog:step_expand(
		call_with_context(Goal,Args),
		call_with_context(Expanded,Args), Context) :-
	mongolog_expand(Goal, Expanded, Context).

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
% if this is the case, they must be replaces by variable
% keys to be referred to in queries.
%
resolve_scope(In, Ctx, [scope(Scope1)|Rest]) :-
	select_option(scope(Scope0),In,Rest),!,
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	resolve_scope1(Since0,Ctx,Since1),
	resolve_scope1(Until0,Ctx,Until1).
resolve_scope(In, _, In).

%%
resolve_scope1(In, Ctx, Out) :-
	mng_strip_operator(In, Operator, Time1),
	mongolog:var_key_or_val(Time1, Ctx, Time2),
	mng_strip_operator(Out, Operator, Time2).

