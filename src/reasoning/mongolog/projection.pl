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
lang_query:step_expand(
		project(->(If,Then)),
		->(If_expanded,Then_expande)) :-
	lang_query:kb_expand(If, If_expanded),
	lang_query:kb_expand(project(Then), Then_expande).

lang_query:step_expand(project(','(A,B)), ','(C,D)) :-
	lang_query:step_expand(project(A),C),
	lang_query:step_expand(project(B),D).

lang_query:step_expand(project(;(A,B)), ;(C,D)) :-
	lang_query:step_expand(project(A),C),
	lang_query:step_expand(project(B),D).

lang_query:step_expand(
		project(call_with_context(Goal,Ctx)),
		call_with_context(project(Goal),Ctx)).

lang_query:step_expand(project(call(Goal)), call(Expanded)) :-
	lang_query:step_expand(project(Goal),Expanded).

lang_query:step_expand(project(once(Goal)), once(Expanded)) :-
	lang_query:step_expand(project(Goal),Expanded).

lang_query:step_expand(project(limit(N,Goal)), limit(N,Expanded)) :-
	lang_query:step_expand(project(Goal),Expanded).

lang_query:step_expand(project(pragma(Goal)), pragma(Goal)) :- !.

lang_query:step_expand(project(ask(Goal)), Expanded) :-
	lang_query:kb_expand(Goal, Expanded).

lang_query:step_expand(project(List), Expanded) :-
	is_list(List),!,
	maplist([X,project(X)]>>true, List, Mapped),
	lang_query:kb_expand(Mapped, Expanded).

lang_query:step_expand(project(Goal), Expanded) :-
	nonvar(Goal),
	comma_list(Goal, Terms),
	length(Terms, NumTerms),
	NumTerms > 1,
	lang_query:kb_expand(project(Terms), Expanded).

lang_query:step_expand(project(Goal), Expanded) :-
	compound(Goal),
	Goal =.. [Functor|Args],
	atom_concat('project_', Functor, Functor0),
	Goal0 =.. [Functor0|Args],
	catch(lang_query:expand_term_0([Goal0], Expanded), _, fail).

%%
%
mongolog:step_compile(project(Goal), Ctx, Pipeline, StepVars) :-
	% just call a query in case project/1 has not been expanded into assert/1
	mongolog:step_compile(call(Goal), Ctx, Pipeline, StepVars).

