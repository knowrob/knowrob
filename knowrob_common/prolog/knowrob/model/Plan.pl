
:- module(knowrob_model_Plan,
    [
      plan_has_goal/2,
      plan_defines_task/2
    ]).
/** <module> Interface to RDF model of plans.

*Plan* is defined as a *Description* having an explicit *Goal*, to be achieved by executing the plan.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      plan_has_goal(r,r),
      plan_defines_task(r,r).

%% plan_has_goal(?Plan,?Goal) is semidet.
%
% Relates a plan to its goal.
%
% @param Plan An individual of type dul:'Plan'.
% @param Goal An individual of type dul:'Description'.
%
plan_has_goal(Plan,Goal) :-
  kb_triple(Plan,dul:hasComponent,Goal),
  kb_type_of(Goal,dul:'Goal').

%% plan_defines_task(?Plan,?Tsk) is semidet.
%
% Relates a plan to the task it defines.
%
% @param Plan An individual of type dul:'Plan'.
% @param Tsk An individual of type dul:'Task'.
%
plan_defines_task(Plan,Tsk) :-
  kb_triple(Plan,ease:isPlanFor,Tsk) *-> true ;
  property_range(Plan,ease:isPlanFor,Tsk).
