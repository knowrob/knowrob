:- module(meta_commands, []).

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).
:- use_module(library('db/mongo/client'),
		[ mng_strip/4 ]).
:- use_module(library('lang/compiler')).
:- use_module(library('lang/query')).

%%
% Make a possibly nondet goal semidet, i.e., succeed at most once.
%
% TODO: triple has special handling for this (modifier), remove it?
%
once(Goal) ?+> call(Goal), !.

%%
% Make a possibly nondet goal semidet, i.e., succeed at most once.
%
% TODO: triple has special handling for this (modifier), remove it?
%
ignore(Goal)  ?+> call(Goal), !.
ignore(_Goal) ?+> true.

%% query commands
:- query_compiler:add_command(call, [ask,tell]).

%% query expansion
query_compiler:step_expand(
		call(Goal, Scope),
		call(Expanded, Scope),
		Context) :-
	query_expand(Goal, Expanded, Context).

%% query compilation
query_compiler:step_var(call(Terminals, _Scope), Var) :-
	member(X,Terminals),
	query_compiler:step_var(X, Var).

query_compiler:step_var(call(_Terminals, Scope), Var) :-
	time_scope(Scope, Since, Until),
	member(X, [Since,Until]),
	mng_strip(X, _Operator, _Type, Y),
	query_compiler:step_var(Y, Var).

%%
query_compiler:step_compile(
		call(Terminals, Scope0),
		Context0,
		Pipeline) :-
	% get since/until values
	time_scope(Since0, Until0, Scope0),
	time_scope(Since1, Until1, Scope1),
	query_compiler:var_key_or_val(Since0,Since1),
	query_compiler:var_key_or_val(Until0,Until1),
	% remove previous scope from context
	merge_options([scope(Scope1)], Context0, Context1),
	% finally compile called goal
	% and replace the scope in compile context
	query_compile(Terminals, Pipeline, Context1).

