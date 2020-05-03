:- module(model_EASE_WF,
    [ is_binding(r),
      is_succeedence(r),
      plan_defines_task(r,r),
      workflow_step(r,r),
      workflow_first_step(r,r),
      workflow_role_range(r,r,r),
      workflow_constituent(r,r)
    ]).
/** <module> Interface predicates for EASE-WF model.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS')
    [ has_type/2
    ]).
:- use_module(library('model/DUL/Event')
    [ task_role_range/3
    ]).
:- use_module(library('db/tripledb')
    [ tripledb_load/2
    ]).

:- tripledb_load('http://www.ease-crc.org/ont/EASE-WF.owl',
    [ graph(tbox),
      namespace(ease_wf)
    ]).

%% is_binding(+Entity) is semidet.
%
% True iff Entity is an instance of ease_wf:'Binding'.
%
% @param Entity An entity IRI.
%
is_binding(Entity) ?+>
  has_type(Entity, ease_wf:'Binding').

%% is_succeedence(+Entity) is semidet.
%
% True iff Entity is an instance of ease_wf:'Succeedence'.
%
% @param Entity An entity IRI.
%
is_succeedence(Entity) ?+>
  has_type(Entity, ease_wf:'Succeedence').

%% plan_defines_task(?Plan,?Tsk) is semidet.
%
% Relates a plan to the task it defines.
%
% @param Plan An individual of type dul:'Plan'.
% @param Tsk An individual of type dul:'Task'.
%
plan_defines_task(Plan,Tsk) ?+>
  holds(Plan,ease:isPlanFor,Tsk).

%% plan_has_goal(?WF,?Step) is semidet.
%
% Relates a workflow to one of its steps.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_step(WF,Step) ?+>
  holds(WF,ease_wf:hasStep,Step).

%% workflow_first_step(?WF,?Step) is semidet.
%
% Relates a workflow to the dedicated first step of it.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_first_step(WF,Step) ?+>
  holds(WF,ease_wf:hasFirstStep,Step).

%% workflow_constituent(+WF, ?Constituent) is semidet.
%
% @param WF A workflow.
%
workflow_constituent(WF,X) ?> holds(WF,dul:describes,X).
workflow_constituent(WF,X) ?> holds(WF,dul:definesTask,X).
workflow_constituent(WF,X) ?> holds(WF,ease_proc:definesProcess,X).

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
workflow_role_range(WF,Role,ObjectType) ?>
  workflow_constituent(WF,Tsk),
  task_role_range(Tsk,Role,ObjectType).
