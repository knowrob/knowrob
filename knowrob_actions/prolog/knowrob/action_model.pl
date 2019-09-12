
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
      %%
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
      situation_create/3,
      situation_add/2,
      situation_add_satisfies/2
    ]).
/** <module> Interface to RDF model of actions and tasks.

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
      task_role_range(r,r,r),
      task_parameter_range(r,r,r),
      workflow_step(r,r),
      workflow_first_step(r,r),
      workflow_role_range(r,r,r),
      situation_create(r,-,+),
      situation_add(r,r),
      situation_add_satisfies(r,r),
      action_set_status_(r,r).

		 /*******************************
		 *	dul:'Event'		*
		 *******************************/

%%
event_create(EvtType,Evt,Graph) :-
  kb_create(EvtType,Evt,_{graph:Graph}),
  kb_create(dul:'TimeInterval',I,_{graph:Graph}),
  kb_assert(Evt,dul:hasTimeInterval,I,_{graph:Graph}).

%%
event_time(Evt,BeginTime,EndTime) :-
  event_begin_time(Evt,BeginTime),
  ignore(event_end_time(Evt,EndTime)).

%%
event_begin_time(Evt,Stamp) :-
  interval(Evt,[Stamp|_]).

%%
event_end_time(Evt,Stamp) :-
  interval(Evt,[_,Stamp]).

%%
event_set_begin_time(Evt) :-
  current_time(Stamp),
  event_set_begin_time(Evt,Stamp).

event_set_begin_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalBegin,_),
  kb_assert(I,ease:hasIntervalBegin,Stamp).

%%
event_set_end_time(Evt) :-
  current_time(Stamp),
  event_set_end_time(Evt,Stamp).

event_set_end_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalEnd,_),
  kb_assert(I,ease:hasIntervalEnd,Stamp).

%%
event_participant(Evt,Participant,Class) :-
  kb_triple(Evt,dul:hasParticipant,Participant),
  kb_type_of(Participant,Class).

		 /*******************************
		 *	dul:'Action'		*
		 *******************************/
%%
action_create(ActType,Act,Graph) :-
  event_create(ActType,Act,Graph).

%%
action_status(Act,Status) :-
  kb_triple(Act, ease_act:hasExecutionState, Status).

%%
action_set_succeeded(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Succeeded').
%%
action_set_failed(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Failed').
%%
action_set_active(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Active').
%%
action_set_paused(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Paused').
%%
action_set_pending(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Pending').
%%
action_set_cancelled(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Cancelled').

action_set_status_(Act,Status) :-
  kb_retract(Act, ease_act:hasExecutionState, _),
  kb_assert(Act, ease_act:hasExecutionState, Status).

%%
action_has_task(Act,Tsk) :-
  kb_triple(Act,dul:executesTask,Tsk).

%%
action_set_task(Act,Tsk) :-
  kb_assert(Act,dul:executesTask,Tsk).


%% action_add_filler(Action,RegionOrObject)
%
action_add_filler(Action,X) :-
  kb_type_of(X,dul:'Region'),!, (
    kb_triple(Action,dul:hasRegion,X);
    kb_assert(Action,dul:hasRegion,X)
  ), !.

action_add_filler(Action,X) :-
  kb_type_of(X,dul:'Object'),!, (
    kb_triple(Action,dul:hasParticipant,X);
    kb_assert(Action,dul:hasParticipant,X)
  ), !.

action_add_filler(_Action,X) :-
  writef('[WARN] %w is not a Region or Object!\n', [X]),
  fail.

%%
action_performed_by(Act,Agent) :-
  kb_triple(Act,ease:performedBy,Agent). % TODO


		 /*******************************
		 *	dul:'Task		*
		 *******************************/

%%
task_role_range(Tsk,Role,Obj) :-
  property_cardinality(Tsk,dul:isTaskOf,RoleDescr,CR,_), CR>0,
  property_range(RoleDescr,dul:classifies,ObjectDescr),
  once((
    %% TODO owl_type_of
    %%
    owl_subclass_of(RoleDescr,Role),
    rdfs_subclass_of(Role,dul:'Role'),
    %%
    owl_subclass_of(ObjectDescr,Obj),
    rdfs_subclass_of(Obj,dul:'Object')
  )).

%%
task_parameter_range(Tsk,Parameter,Region) :-
  property_cardinality(Tsk,dul:hasParameter,ParamDescr,CR,_), CR>0,
  property_range(ParamDescr,dul:classifies,RegionDescr),
  once((
    %% TODO owl_type_of
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

%%
plan_has_goal(Plan,Goal) :-
  kb_triple(Plan,dul:hasComponent,Goal),
  kb_type_of(Goal,dul:'Goal').

%%
plan_defines_task(Plan,Tsk) :-
  kb_triple(Plan,ease:isPlanFor,Tsk) *-> true ;
  property_range(Plan,ease:isPlanFor,Tsk).

		 /*******************************
		 *	dul:'Workfow'		*
		 *******************************/

%%
workflow_step(WF,Step) :-
  kb_triple(WF,ease_wf:hasStep,Step).

%%
workflow_first_step(WF,Step) :-
  kb_triple(WF,ease_wf:hasFirstStep,Step).

%%
workflow_role_range(WF,Role,ObjectType) :-
  ( kb_triple(WF,ease:isPlanFor,Tsk) ; workflow_step(WF,Tsk) ),
  task_role_range(Tsk,Role,ObjectType).

		 /*******************************
		 *	dul:'Situation'		*
		 *******************************/

situation_create(Type,Situation,Graph) :-
  kb_create(Type,Situation,_{graph:Graph}).

situation_add(Situation,Act) :-
  kb_type_of(Act,dul:'Action'),!,
  kb_assert(Situation,dul:includesAction,Act).

situation_add(Situation,Evt) :-
  kb_type_of(Evt,dul:'Event'),!,
  kb_assert(Situation,dul:includesEvent,Evt).

situation_add(Situation,Agent) :-
  kb_type_of(Agent,dul:'Agent'),!,
  kb_assert(Situation,dul:includesAgent,Agent).

situation_add(Situation,SubSituation) :-
  kb_type_of(SubSituation,dul:'Situation'),!,
  kb_assert(Situation,ease:includesSituation,SubSituation).

situation_add(Situation,Concept) :-
  kb_type_of(Concept,dul:'Concept'),!,
  kb_assert(Situation,ease:includesConcept,Concept).

situation_add(Situation,Object) :-
  kb_type_of(Object,dul:'Object'),!,
  kb_assert(Situation,dul:includesObject,Object).

situation_add_satisfies(Situation,Description) :-
  kb_assert(Situation,dul:satisfies,Description).
