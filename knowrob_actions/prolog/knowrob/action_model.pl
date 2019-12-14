
:- module(knowrob_action_model,
    [
      event_create/3,
      event_time/3,
      event_begin_time/2,
      event_end_time/2,
      event_set_begin_time/1,
      event_set_begin_time/2,
      event_set_end_time/1,
      event_set_end_time/2,
      event_participant/3,
      %%
      action_create/3,
      action_status/2,
      action_set_succeeded/1,
      action_set_failed/1,
      action_set_active/1,
      action_set_paused/1,
      action_set_pending/1,
      action_set_cancelled/1,
      action_has_task/2,
      action_set_task/2,
      action_add_filler/2,
      action_performed_by/2,
      action_set_performed_by/2,
      %%
      task_role/3,
      task_parameter/3,
      task_role_range/3,
      task_parameter_range/3,
      %%
      plan_has_goal/2,
      plan_defines_task/2,
      %%
      workflow_step/2,
      workflow_first_step/2,
      workflow_role_range/3,
      %%
      situation_satisfies/2,
      situation_includes_classification/3,
      situation_includes_assignment/3,
      situation_create/3,
      situation_add/2,
      situation_add_satisfies/2,
      situation_add_classification/3,
      situation_add_assignment/3
    ]).
/** <module> Interface to RDF model of actions, tasks, plans, and plan executions.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      event_create(r,r,+),
      event_time(r,?,?),
      event_begin_time(r,?),
      event_end_time(r,?),
      event_set_begin_time(r,?),
      event_set_end_time(r,?),
      event_set_begin_time(r),
      event_set_end_time(r),
      event_participant(r,r,r),
      action_create(r,r,+),
      action_set_succeeded(r),
      action_set_cancelled(r),
      action_set_pending(r),
      action_set_paused(r),
      action_set_active(r),
      action_set_failed(r),
      action_status(r,r),
      action_has_task(r,r),
      action_set_task(r,r),
      action_performed_by(r,r),
      action_set_performed_by(r,r),
      task_role(r,r,r),
      task_parameter(r,r,r),
      task_role_range(r,r,r),
      task_parameter_range(r,r,r),
      workflow_step(r,r),
      workflow_first_step(r,r),
      workflow_role_range(r,r,r),
      situation_satisfies(r,r),
      situation_includes_classification(r,r,r),
      situation_includes_assignment(r,r,r),
      situation_create(r,-,+),
      situation_add(r,r),
      situation_add_satisfies(r,r),
      situation_add_classification(r,r,r),
      situation_add_assignment(r,r,r),
      action_set_status_(r,r).

		 /*******************************
		 *	dul:'Event'		*
		 *******************************/

%% event_create(+EvtType,-Evt,+Graph) is semidet.
%
% Creates an event individual. Note that actions, processes,
% and states are all sub-classes of event in this model.
%
% @param EvtType A sub-class of dul:'Event'.
% @param Evt An individual of type EvtType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
event_create(EvtType,Evt,Graph) :-
  kb_create(EvtType,Evt,_{graph:Graph}),
  kb_create(dul:'TimeInterval',I,_{graph:Graph}),
  kb_assert(Evt,dul:hasTimeInterval,I,_{graph:Graph}).

%% event_time(+Evt,?BeginTime,?EndTime) is det.
%
% Maps an event to its begin and end time.
% BeginTime/EndTime remains unchanged in case no
% begin/end time is defined for the given event.
%
% @param Evt An individual of type dul:'Event'.
% @param BeginTime The timestamp indicating the begin of the event.
% @param EndTime The timestamp indicating the end of the event.
%
event_time(Evt,BeginTime,EndTime) :-
  ignore(event_begin_time(Evt,BeginTime)),
  ignore(event_end_time(Evt,EndTime)).

%% event_begin_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its begin time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the begin of the event.
%
event_begin_time(Evt,Stamp) :-
  interval_start(Evt,Stamp).

%% event_end_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its end time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the end of the event.
%
event_end_time(Evt,Stamp) :-
  interval_end(Evt,Stamp).

%% event_set_begin_time(+Evt,?Stamp) is det.
%% event_set_begin_time(+Evt) is det.
%
% Associates an event to its begin time.
% event_set_begin_time/1 sets the begin time to the current time.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the begin of the event.
%
event_set_begin_time(Evt) :-
  current_time(Stamp),
  event_set_begin_time(Evt,Stamp).

event_set_begin_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalBegin,_),
  kb_assert(I,ease:hasIntervalBegin,Stamp).


%% event_set_end_time(+Evt,?Stamp) is det.
%% event_set_end_time(+Evt) is det.
%
% Associates an event to its end time.
% event_set_end_time/1 sets the end time to the current time.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the end of the event.
%
event_set_end_time(Evt) :-
  current_time(Stamp),
  event_set_end_time(Evt,Stamp).

event_set_end_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalEnd,_),
  kb_assert(I,ease:hasIntervalEnd,Stamp).

%% event_participant(+Evt,?Participant,?Class) is semidet.
%
% Relates an event to a tuple of an object partipating in
% the event and its type.
%
% @param Evt An individual of type dul:'Event'.
% @param Participant An individual of type dul:'Object'.
% @param Class The most specific type of Participant.
%
event_participant(Evt,Participant,Class) :-
  kb_triple(Evt,dul:hasParticipant,Participant),
  kb_type_of(Participant,Class).

		 /*******************************
		 *	dul:'Action'		*
		 *******************************/

%% action_create(+ActType,-Act,+Graph) is semidet.
%
% Creates an action individual.
%
% @param ActType A sub-class of dul:'Action'.
% @param Act An individual of type ActType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
action_create(ActType,Act,Graph) :-
  event_create(ActType,Act,Graph).

%% action_status(?Act,?Status) is semidet.
%
% Relates an action to its execution status.
%
% @param Act An individual of type dul:'Action'.
% @param Status The execution status of Act.
%
action_status(Act,Status) :-
  kb_triple(Act, ease_act:hasExecutionState, Status).

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
action_set_status_(Act,Status) :-
  kb_retract(Act, ease_act:hasExecutionState, _),
  kb_assert(Act, ease_act:hasExecutionState, Status).

%% action_has_task(?Act,?Tsk) is semidet.
%
% Relates an action to the task that it executes.
% Note that this relations may not be known, e.g. in case
% of observing another agent performing an action.
% In such a case this predicate fails.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_has_task(Act,Tsk) :-
  kb_triple(Act,dul:executesTask,Tsk).

%% action_set_task(+Act,+Tsk) is semidet.
%
% Asserts the task that is executed by an action.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_set_task(Act,Tsk) :-
  kb_assert(Act,dul:executesTask,Tsk).

%% action_add_filler(+Act,+Filler) is semidet.
%
% Asserts that some object or region was involved
% in an action.
% This does not assert what role the object played,
% or what the parameter associated to this region is.
%
% @param Act An individual of type dul:'Action'.
% @param Filler An individual of type dul:'Object' or dul:'Region'.
%
action_add_filler(Act,X) :-
  kb_type_of(X,dul:'Region'),!, (
    kb_triple(Act,dul:hasRegion,X);
    kb_assert(Act,dul:hasRegion,X)
  ), !.

action_add_filler(Act,X) :-
  kb_type_of(X,dul:'Object'),!, (
    kb_triple(Act,dul:hasParticipant,X);
    kb_assert(Act,dul:hasParticipant,X)
  ), !.

action_add_filler(_Action,X) :-
  print_message(warning,
    model_error(not_a_region_or_object(X))),
  fail.

%% action_performed_by(?Act,?Agent) is semidet.
%
% Relates an action to the agent that performs it.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_performed_by(Act,Agent) :-
  kb_triple(Act,ease_act:isPerformedBy,Agent).

%% action_set_performed_by(+Act,+Agent) is semidet.
%
% Asserts the agent that performs an action.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_set_performed_by(Act,Agent) :-
  kb_assert(Act,ease_act:isPerformedBy,Agent).


		 /*******************************
		 *	dul:'Task		*
		 *******************************/

%% task_role(?Tsk,?Role,?RoleType) is semidet.
%
% Relates a task to roles of objects related to the tasks.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param RoleType A sub-class of dul:'Role'.
%
task_role(Tsk,Role,RoleType) :-
  kb_triple(Tsk,dul:isTaskOf,Role),
  kb_type_of(Role,RoleType).

task_parameter(Tsk,Param,ParamType) :-
  kb_triple(Tsk,dul:hasParameter,Param),
  kb_type_of(Param,ParamType).

%%
task_role_range(Tsk,Role,Obj) :-
  property_cardinality(Tsk,dul:isTaskOf,RoleDescr,CR,_), CR>0,
  property_range(RoleDescr,dul:classifies,ObjDescr),
  once((
    %%
    owl_subclass_of(RoleDescr,Role),
    rdfs_subclass_of(Role,dul:'Role'),
    %%
    owl_subclass_of(ObjDescr,Obj),
    rdfs_subclass_of(Obj,dul:'Object')
  )).

%%
task_parameter_range(Tsk,Parameter,Region) :-
  property_cardinality(Tsk,dul:hasParameter,ParamDescr,CR,_), CR>0,
  property_range(ParamDescr,dul:classifies,RegionDescr),
  once((
    %%
    owl_subclass_of(ParamDescr,Parameter),
    rdfs_subclass_of(Parameter,dul:'Parameter'),
    %%
    owl_subclass_of(RegionDescr,Region),
    rdfs_subclass_of(Region,dul:'Region')
  )).

		 /*******************************
		 *	dul:'Plan'		*
		 *******************************/


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

		 /*******************************
		 *	dul:'Workfow'		*
		 *******************************/

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

		 /*******************************
		 *	dul:'Situation'		*
		 *******************************/

%% situation_create(+SitType,-Sit,+Graph) is semidet.
%
% Creates a situation individual.
%
% @param SitType A sub-class of dul:'Situation'.
% @param Sit An individual of type SitType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
situation_create(SitType,Sit,Graph) :-
  kb_create(SitType,Sit,_{graph:Graph}).

%% situation_add(+Sit,+Entity) is det.
%
% Asserts that some entity is included in the situation.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity A named individual.
%
situation_add(Sit,Act) :-
  kb_type_of(Act,dul:'Action'),!,
  kb_assert(Sit,dul:includesAction,Act).

situation_add(Sit,Evt) :-
  kb_type_of(Evt,dul:'Event'),!,
  kb_assert(Sit,dul:includesEvent,Evt).

situation_add(Sit,Agent) :-
  kb_type_of(Agent,dul:'Agent'),!,
  kb_assert(Sit,dul:includesAgent,Agent).

situation_add(Sit,SubSituation) :-
  kb_type_of(SubSituation,dul:'Situation'),!,
  kb_assert(Sit,ease:includesSituation,SubSituation).

situation_add(Sit,Concept) :-
  kb_type_of(Concept,dul:'Concept'),!,
  kb_assert(Sit,ease:includesConcept,Concept).

situation_add(Sit,Region) :-
  kb_type_of(Region,dul:'Region'),!,
  % TODO: use more specific relation
  kb_assert(Sit,dul:isSettingFor,Region).

situation_add(Sit,Object) :-
  kb_type_of(Object,dul:'Object'),!,
  kb_assert(Sit,dul:includesObject,Object).


%% situation_includes_assignment(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_includes_assignment(Sit,Argument,Value) :-
  kb_triple(Sit,ease:includesSituation,Assignment),
  rdfs_individual_of(Assignment,knowrob:'Assignment'),
  ( ground(Argument) ->
  ( kb_triple(Assignment,dul:includesObject,Argument),
    kb_triple(Assignment,dul:isSettingFor,Value) ) ;
  ( kb_triple(Assignment,dul:isSettingFor,Value),
    kb_triple(Assignment,dul:includesObject,Argument) )
  ),
  Argument \= Value.

%% situation_add_assignment(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_add_assignment(Sit,Argument,Value) :-
  situation_create(knowrob:'Assignment',Assignment,belief_state),
  situation_add(Sit,Assignment),
  situation_add(Assignment,Argument),
  situation_add(Assignment,Value).

%% situation_includes_classification(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_includes_classification(Sit,Entity,Concept) :-
  kb_triple(Sit,ease:includesSituation,Classification),
  rdfs_individual_of(Classification,dul:'Classification'),
  ( ground(Concept) ->
  ( kb_triple(Classification,ease:includesConcept,Concept),
    kb_triple(Classification,dul:isSettingFor,Entity) ) ;
  ( kb_triple(Classification,dul:isSettingFor,Entity),
    kb_triple(Classification,ease:includesConcept,Concept) )
  ),
  Entity \= Concept.

%% situation_add_classification(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_add_classification(Sit,Entity,Concept) :-
  is_class(Concept),!,
  ( situation_event_concept_(Sit,Concept,Concept0) ;
    mem_new_individual(Concept,Concept0)
  ),!,
  situation_add_classification(Sit,Entity,Concept0).

situation_add_classification(Sit,Entity,Concept0) :-
  is_individual(Concept0),
  situation_create(dul:'Classification',Classification,belief_state),
  situation_add(Sit,Classification),
  situation_add(Classification,Concept0),
  situation_add(Classification,Entity).

situation_event_concept_(Sit,Concept,Concept0) :-
  kb_triple(Sit,dul:includesEvent,Evt),
  ( action_has_task(Evt,EvtType) ;
    situation_classifies(Sit,Evt,EvtType)
  ),
  ( kb_triple(EvtType,dul:isTaskOf,Concept0) ;
    kb_triple(EvtType,dul:hasParameter,Concept0)
  ),
  kb_type_of(Concept0,Concept), !.

%% situation_satisfies(?Sit,?Descr) is nondet.
%
% Associates a situation to a description that is
% satisfied by the situation.
% An example is that the execution of a plan (a situation)
% satisfies the plan (a description).
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
situation_satisfies(Sit,Descr) :-
  kb_triple(Sit,dul:satisfies,Descr).

%% situation_add_satisfies(+Sit,+Descr) is det.
%
% Asserts that a situation satisfies some description
% (such as a plan).
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
situation_add_satisfies(Sit,Descr) :-
  kb_assert(Sit,dul:satisfies,Descr).
