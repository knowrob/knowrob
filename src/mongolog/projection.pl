:- module(mongolog_projection, []).
/** <module> Projecting the consequences of rules.

The following predicates are supported:

| Predicate    | Arguments |
| ---          | ---       |
| project/1    | :Goal |

@author Daniel Beßler
@license BSD
*/

:- use_module('mongolog').

%%
:- mongolog:add_command(project).

%%
mongolog:step_expand(
		project(->(If,Then)),
		->(If_expanded,Then_expande)) :-
	mongolog_expand(If, If_expanded),
	mongolog_expand(project(Then), Then_expande).

mongolog:step_expand(project(','(A,B)), ','(C,D)) :-
	mongolog:step_expand(project(A),C),
	mongolog:step_expand(project(B),D).

mongolog:step_expand(project(;(A,B)), ;(C,D)) :-
	mongolog:step_expand(project(A),C),
	mongolog:step_expand(project(B),D).

mongolog:step_expand(
		project(call_with_context(Goal,Ctx)),
		call_with_context(project(Goal),Ctx)).

mongolog:step_expand(project(call(Goal)), call(Expanded)) :-
	mongolog:step_expand(project(Goal),Expanded).

mongolog:step_expand(project(once(Goal)), once(Expanded)) :-
	mongolog:step_expand(project(Goal),Expanded).

mongolog:step_expand(project(limit(N,Goal)), limit(N,Expanded)) :-
	mongolog:step_expand(project(Goal),Expanded).

mongolog:step_expand(project(pragma(Goal)), pragma(Goal)) :- !.

mongolog:step_expand(project(ask(Goal)), Expanded) :-
	mongolog_expand(Goal, Expanded).

mongolog:step_expand(project(List), Expanded) :-
	is_list(List),!,
	maplist([X,project(X)]>>true, List, Mapped),
	mongolog_expand(Mapped, Expanded).

mongolog:step_expand(project(Goal), Expanded) :-
	nonvar(Goal),
	comma_list(Goal, Terms),
	length(Terms, NumTerms),
	NumTerms > 1,
	mongolog_expand(project(Terms), Expanded).

mongolog:step_expand(project(Goal), Expanded) :-
	compound(Goal),
	Goal =.. [Functor|Args],
	atom_concat('project_', Functor, Functor0),
	Goal0 =.. [Functor0|Args],
	catch(mongolog:expand_term_0([Goal0], Expanded), _, fail).

%%
%
mongolog:step_compile(project(Goal), Ctx, Pipeline, StepVars) :-
	% just call a query in case project/1 has not been expanded into assert/1
	mongolog:step_compile(call(Goal), Ctx, Pipeline, StepVars).

