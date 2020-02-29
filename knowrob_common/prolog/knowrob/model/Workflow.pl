
:- module(knowrob_model_Workflow,
    [
      workflow_step/2,
      workflow_first_step/2,
      workflow_role_range/3
    ]).
/** <module> Interface to RDF model of workflows.

*Workflow* is defined as a *Plan* that *defines* *Role(s)*, *Task(s)*, and a specific structure for tasks to be executed, usually supporting the work of an *Organization*.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- use_module(library('knowrob/model/Task'), [ task_role_range/3 ]).

:- rdf_meta
      workflow_step(r,r),
      workflow_first_step(r,r),
      workflow_role_range(r,r,r).

%% plan_has_goal(?WF,?Step) is semidet.
%
% Relates a workflow to one of its steps.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_step(WF,Step) :-
  kb_triple(WF,ease_wf:hasStep,Step).

%% workflow_first_step(?WF,?Step) is semidet.
%
% Relates a workflow to the dedicated first step of it.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_first_step(WF,Step) :-
  kb_triple(WF,ease_wf:hasFirstStep,Step).

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
  ( kb_triple(WF,ease:isPlanFor,Tsk) ; workflow_step(WF,Tsk) ),
  task_role_range(Tsk,Role,ObjectType).
