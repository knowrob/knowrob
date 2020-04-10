
:- module(model_DUL_EventType,
    [
      is_event_type(r),
      is_task(r),
      task_parameter(r,r,r),
      task_parameter_range(r,r,r),
      task_role(r,r,r),
      task_role_range(r,r,r)
    ]).
:- rdf_module.
/** <module> DUL notion of Object.

In DUL, Object is defined as:
  "A Concept that classifies an Event . An event type describes how an Event should be interpreted, executed, expected, seen, etc., according to the Description that the EventType isDefinedIn (or used in)."

@author Daniel Beßler
@license BSD
*/

%% is_event_type(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'EventType'.
%
% @param Entity An entity IRI.
%
is_event_type(Entity) :-
  ask( Entity rdf:type dul:'EventType' ).

%% is_task(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Task'.
%
% @param Entity An entity IRI.
%
is_task(Entity) :-
  ask( Entity rdf:type dul:'Task' ).

%% task_role(?Tsk,?Role,?RoleType) is semidet.
%
% Relates a task to roles of objects related to the tasks.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param RoleType A sub-class of dul:'Role'.
%
task_role(EventType,Role,RoleType) :-
  ask( EventType dul:isTaskOf Role ),
  ask( Role      rdf:type     RoleType ).

%%
%
task_role_range(EventType,Role,Obj) :-
  ask( EventType dul:isTaskOf   some Role ),
  ask( Role      dul:classifies only Obj ).

%%
%
task_parameter(EventType,Param,ParamType) :-
  ask( EventType dul:hasParameter Param ),
  ask( Param     rdf:type         ParamType ).

%%
%
task_parameter_range(EventType,Parameter,Region) :-
  ask( EventType dul:hasParameter some Parameter ),
  ask( Parameter dul:classifies   only Region ).
