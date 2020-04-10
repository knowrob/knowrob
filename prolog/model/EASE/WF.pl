
:- module(model_EASE_WF,
    [
      is_binding(r),
      is_succeedence(r),
      plan_defines_task(r,r),
      workflow_step(r,r),
      workflow_first_step(r,r),
      workflow_role_range(r,r,r),
      workflow_constituent(r,r)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-WF model.

@author Daniel Beßler
@license BSD
*/

%% is_binding(+Entity) is semidet.
%
% True iff Entity is an instance of ease_wf:'Binding'.
%
% @param Entity An entity IRI.
%
is_binding(Entity) :-
  ask( Entity rdf:type ease_wf:'Binding' ).

%% is_succeedence(+Entity) is semidet.
%
% True iff Entity is an instance of ease_wf:'Succeedence'.
%
% @param Entity An entity IRI.
%
is_succeedence(Entity) :-
  ask( Entity rdf:type ease_wf:'Succeedence' ).

%% plan_defines_task(?Plan,?Tsk) is semidet.
%
% Relates a plan to the task it defines.
%
% @param Plan An individual of type dul:'Plan'.
% @param Tsk An individual of type dul:'Task'.
%
plan_defines_task(Plan,Tsk) :-
  ask( Plan ease:isPlanFor Tsk ).

%% plan_has_goal(?WF,?Step) is semidet.
%
% Relates a workflow to one of its steps.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_step(WF,Step) :-
  ask( WF ease_wf:hasStep Step ).

%% workflow_first_step(?WF,?Step) is semidet.
%
% Relates a workflow to the dedicated first step of it.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_first_step(WF,Step) :-
  ask( WF ease_wf:hasFirstStep Step ).

%% workflow_role_range(?WF,?Role,?ObjectType) is semidet.
%
% Relates a workflow to roles of objects defined by the tasks
% of the workflow, and also infers the required type for fillers
% of the role.
%
% @param WF An individual of type dul:'Workflow'.
% @param Role An individual of type dul:'Role'.
% @param ObjectType A sub-class of dul:'Object'.
%
workflow_role_range(WF,Role,ObjectType) :-
  ( ask( WF ease:isPlanFor Tsk ) ;
    workflow_step(WF,Tsk)
  ),
  task_role_range(Tsk,Role,ObjectType).

%% workflow_constituent(+WF, ?Constituent) is semidet.
%
% @param WF A workflow.
%
workflow_constituent(WF, Constituent) :-
  ask( WF ease:isPlanFor X0 ) ;
  ask( WF dul:definesTask X0 );
  ask( WF ease_proc:definesProcess X0 ).
