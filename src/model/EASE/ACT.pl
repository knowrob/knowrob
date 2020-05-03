:- module(model_EASE_ACT,
    [ is_manipulation_action(r),
      is_mental_action(r),
      is_mental_task(r),
      is_physical_task(r),
      is_performed_by(r,r),
      has_subevent(r,r),
      action_status(r,r),
      action_succeeded(r),
      action_failed(r),
      action_active(r),
      action_paused(r),
      action_pending(r),
      action_cancelled(r),
      task_effect(r,t)
    ]).
/** <module> Interface predicates for EASE-ACT model.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db'),
    [ rdf_equal/2
    ]).
:- use_module(library('model/RDFS')
    [ has_type/2
    ]).
:- use_module(library('model/DUL/Event')
    [ task_role_range/3
    ]).
:- use_module(library('model/DUL/Region')
    [ has_parameter_range/3
    ]).
:- use_module(library('db/tripledb')
    [ tripledb_load/2
    ]).

:- tripledb_load('http://www.ease-crc.org/ont/EASE-ACT.owl',
    [ graph(tbox),
      namespace(ease_act)
    ]).

%% is_manipulation_action(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'ManipulationAction'.
%
% @param Entity An entity IRI.
%
is_manipulation_action(Entity) ?+>
  has_type(Entity, ease_act:'ManipulationAction').

%% is_mental_action(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'MentalAction'.
%
% @param Entity An entity IRI.
%
is_mental_action(Entity) ?+>
  has_type(Entity, ease_act:'MentalAction').

%% is_physical_task(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'PhysicalTask'.
%
% @param Entity An entity IRI.
%
is_physical_task(Entity) ?+>
  has_type(Entity, ease_act:'PhysicalTask').

%% is_mental_task(+Entity) is semidet.
%
% True iff Entity is an instance of ease_act:'MentalTask'.
%
% @param Entity An entity IRI.
%
is_mental_task(Entity) ?+>
  has_type(Entity, ease_act:'MentalTask').

%% is_performed_by(?Act,?Agent) is semidet.
%
% Relates an action to the agent that performs it.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
is_performed_by(Act,Agent) ?+>
  holds(Act, ease_act:isPerformedBy, Agent).

%% has_subevent(+Event,?Sub) is nondet.
%
%
has_subevent(Event,Sub) ?>
  holds(Event,dul:hasConstituent,Sub).

has_subevent(Event,Sub) ?>
  holds(Event,ease:hasPhase,Sub).

has_subevent(Event,Sub) +>
  { is_action(Sub) },
  holds(Event,dul:hasConstituent,Sub).

has_subevent(Event,Sub) +>
  { is_process(Sub) },
  holds(Event,ease:hasPhase,Sub).

has_subevent(Event,Sub) +>
  { is_state(Sub) },
  holds(Event,ease:hasPhase,Sub).

%% action_status(?Act,?Status) is semidet.
%
% Relates an action to its execution status.
%
% @param Act An individual of type dul:'Action'.
% @param Status The execution status of Act.
%
action_status(Act,Status) ?+>
  holds(Act, ease_act:hasExecutionState, Status).

%% action_succeeded(?Act) is det.
%
% Set the execution status of an action to 'succeeded'.
%
% @param Act An individual of type dul:'Action'.
%
action_succeeded(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Succeeded').

%% action_failed(?Act) is det.
%
% Set the execution status of an action to 'failed'.
%
% @param Act An individual of type dul:'Action'.
%
action_failed(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Failed').

%% action_active(?Act) is det.
%
% Set the execution status of an action to 'active'.
%
% @param Act An individual of type dul:'Action'.
%
action_active(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Active').

%% action_paused(?Act) is det.
%
% Set the execution status of an action to 'paused'.
%
% @param Act An individual of type dul:'Action'.
%
action_paused(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Paused').

%% action_pending(?Act) is det.
%
% Set the execution status of an action to 'planning'.
%
% @param Act An individual of type dul:'Action'.
%
action_pending(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Pending').

%% action_cancelled(?Act) is det.
%
% Set the execution status of an action to 'cancelled'.
%
% @param Act An individual of type dul:'Action'.
%
action_cancelled(Act) ?+>
  action_status(Act, ease_act:'ExecutionState_Cancelled').

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

task_effect_(Tsk, altered(Type,QualityType)) :-
  task_role_range(Tsk,ease_obj:'AlteredObject',Type),
  has_parameter_range(Tsk,ease_obj:'Setpoint',Region),
  get_altered_quality_type_(Type,Region,QualityType).

%%
get_altered_quality_type_(Concept,Region,Quality_type) :-
  get_altered_quality_type__(Concept,Region,Quality_type),
  \+ rdf_equal(Quality_type,dul:'Quality'),!.

get_altered_quality_type__(_Concept,Region,Quality_type) :-
  holds(Region, dul:isRegionFor, only(Quality_type)).

get_altered_quality_type__(Concept,_Region,Quality_type) :-
  holds(Concept, ease_obj:isTriggerDefinedIn, only(Affordance)),
  subclass_of(Affordance,only(ease_obj:describesQuality,Quality_type)).
