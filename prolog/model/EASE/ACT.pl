
:- module(model_EASE_ACT,
    [
      is_manipulation_action(r),
      is_mental_action(r),
      is_mental_task(r),
      is_physical_task(r),
      action_performed_by(r,r),
      action_set_performed_by(r,r),
      action_status(r,r),
      action_set_succeeded(r),
      action_set_failed(r),
      action_set_active(r),
      action_set_paused(r),
      action_set_pending(r),
      action_set_cancelled(r),
      task_effect(r,t)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-ACT model.

@author Daniel Beßler
@license BSD
*/

:- rdf_meta action_set_status_(r,r).

%% is_manipulation_action(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'ManipulationAction'.
%
% @param Entity An entity IRI.
%
is_manipulation_action(Entity) :-
  ask( Entity rdf:type ease_act:'ManipulationAction' ).

%% is_mental_action(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'MentalAction'.
%
% @param Entity An entity IRI.
%
is_mental_action(Entity) :-
  ask( Entity rdf:type ease_act:'MentalAction' ).

%% is_physical_task(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'PhysicalTask'.
%
% @param Entity An entity IRI.
%
is_physical_task(Entity) :-
  ask( Entity rdf:type ease_act:'PhysicalTask' ).

%% is_mental_task(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'MentalTask'.
%
% @param Entity An entity IRI.
%
is_mental_task(Entity) :-
  ask( Entity rdf:type ease_act:'MentalTask' ).

%% action_performed_by(?Act,?Agent) is semidet.
%
% Relates an action to the agent that performs it.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_performed_by(Act,Agent) :-
  ask( Act ease_act:isPerformedBy Agent ).

%% action_set_performed_by(+Act,+Agent) is semidet.
%
% Asserts the agent that performs an action.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_set_performed_by(Act,Agent) :-
  tell( Act ease_act:isPerformedBy Agent ).

%% action_status(?Act,?Status) is semidet.
%
% Relates an action to its execution status.
%
% @param Act An individual of type dul:'Action'.
% @param Status The execution status of Act.
%
action_status(Act,Status) :-
  ask( Act ease_act:hasExecutionState Status ).

%% action_set_succeeded(?Act) is det.
%
% Set the execution status of an action to 'succeeded'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_succeeded(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Succeeded').

%% action_set_failed(?Act) is det.
%
% Set the execution status of an action to 'failed'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_failed(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Failed').

%% action_set_active(?Act) is det.
%
% Set the execution status of an action to 'active'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_active(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Active').

%% action_set_paused(?Act) is det.
%
% Set the execution status of an action to 'paused'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_paused(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Paused').

%% action_set_pending(?Act) is det.
%
% Set the execution status of an action to 'planning'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_pending(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Pending').

%% action_set_cancelled(?Act) is det.
%
% Set the execution status of an action to 'cancelled'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_cancelled(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Cancelled').

%%
%
action_set_status_(Act,Status) :-
  tell( Act ease_act:hasExecutionState Status ).

%% task_effect(?EventType, ?Effect) is nondet
%
% Relates an action or task to roles that imply change,
% and that need to be taken by some object when the task is executed.
%
%	| created(Type)			| An object has been created.	|
%	| destroyed(Type)		| An object has been destroyed.	|
%	| altered(Type,QualityType)	| An object has been changed.	|
%	| linked(Type)			| An object has been linked. 	|
%	| deposited(Type)		| An object has been deposited. |
%	| commited(Type)		| An object has been commited. 	|
%	| included(Type)		| An object has been included.	|
%	| extracted(Type)		| An object has been extracted.	|
%
% @param ActOrTsk An individual of type dul:'Action' or dul:'Task', or a subclass of dul:'Task'.
%
task_effect(Tsk,Effect) :-
  ground(Effect) ->
  once(task_effect_(Tsk,Effect));
  task_effect_(Tsk,Effect).

task_effect_(Tsk, created(Type)) :-
  task_role_range(Tsk,ease_obj:'CreatedObject',Type).

task_effect_(Tsk, destroyed(Type)) :-
  task_role_range(Tsk,ease_obj:'DestroyedObject',Type).

task_effect_(Tsk, altered(Type,QualityType)) :-
  task_role_range(Tsk,ease_obj:'AlteredObject',Type),
  task_parameter_range(Tsk,ease_obj:'Setpoint',Region),
  get_altered_quality_type_(Type,Region,QualityType).

task_effect_(Tsk, linked(Type)) :-
  task_role_range(Tsk,ease_obj:'LinkedObject',Type).

task_effect_(Tsk, commited(Type)) :-
  task_role_range(Tsk,ease_obj:'CommitedObject',Type).

task_effect_(Tsk, deposited(Type)) :-
  task_role_range(Tsk,ease_obj:'DepositedObject',Type).

task_effect_(Tsk, extracted(Type)) :-
  task_role_range(Tsk,ease_obj:'ExtractedObject',Type).

task_effect_(Tsk, included(Type)) :-
  task_role_range(Tsk,ease_obj:'IncludedObject',Type).

%%
get_altered_quality_type_(Concept,Region,Quality_type) :-
  get_altered_quality_type__(Concept,Region,Quality_type),
  \+ rdf_equal(Quality_type,dul:'Quality'),!.

get_altered_quality_type__(_Concept,Region,Quality_type) :-
  ask( Region dul:isRegionFor only(Quality_type) ).

get_altered_quality_type__(Concept,_Region,Quality_type) :-
  ask( Concept    ease_obj:isTriggerDefinedIn only(Affordance) ),
  ask( Affordance ease_obj:describesQuality   only(Quality_type)).
