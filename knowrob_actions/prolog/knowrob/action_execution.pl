/*
  Copyright (C) 2019 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(action_execution,
    [
      plan_start_action/2,        % create new action instance and assign startTime
      plan_finish_action/1,       % assign endTime and project action effects
      action_registry/2,
      task_isExecutionPossible/1,
      task_isExecutedIn/3,
      execute_task/4,
      action_call_or_failure/4,
      action_filler_for/2,
      action_add_filler/2,
      action_status/2,
      ros_querying/5
    ]).
/** <module> The execution of actions.

*action_registry* is a multifile predicate, different action
execution methods need to define a clause of it.
Fillers of task roles and parameters are assigned as
participants of the action.
Also, a status quality is asserted for each action, and its
quale is updated during the course of its execution.
After action execution, the final status is added as participant
of the action and classified by the *Status* parameter of
the task (if any).

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).
:- use_module(library(list_util)).

:- rdf_meta action_registry(r,?),
            action_filler_for(t,t),
            action_call_or_failure(r,t,r,+),
            action_status(r,r),
            action_bindings(r,t),
            task_isExecutionPossible(r),
            task_isExecutedIn(r,r,r),
            handle_action_failure(r,r,+),
            ros_querying(r,r,t,t,-),
            execute_task(r,t,r,t),
            create_action_symbol(r,r,r,t,r).

%% action_registry(ActionConcept, Goal)
%
% A multifile predicate that defines Prolog goals
% that would when called perform some action.
%
:- multifile action_registry/2.

action_registry('http://www.ease-crc.org/ont/ROS.owl#ServiceQuerying', action_execution:ros_querying).

%% ros_querying(+Action) is semidet.
%
%
ros_querying(Action,ExecutionPlan,InputDict,ActionDict,OutputPairs) :-
  %%%%%%%%%
  %%%%% Find ServiceInterface participant.
  action_call_or_failure(Action, (
      action_participant_type(Action,ServiceInterface,ros:'ServiceInterface')
    ),
    knowrob:'ACTION_INPUT_MISSING',
    'no service interface participant'
  ),!,
  %%%%%%%%%
  %%%%% Find the ROS service (i.e. some object that concretely realizes
  %%%%% the interface.
  action_call_or_failure(Action, (
      rdf_has(Service,ros:concretelyImplements,ServiceInterface),
      ( rdf_has_prolog(ExecutionPlan,ros:hasServiceName,SName) ->
        rdf_has_prolog(Service,ros:hasServiceName,SName) ;
        true )
    ),
    ros:'SERVICE_NODE_UNREACHABLE',
    'OWL ROS Service missing'
  ),
  action_call_or_failure(Action, (
      owl_has(ServiceInterface,ros:hasResponseType,ResType),
      owl_has(ServiceInterface,ros:hasRequestType,ReqType),
      rdf_assert(Action,ease:isAnsweredBy,Service)
    ),
    knowrob:'ACTION_MODEL_ERROR',
    'request or response type missing'
  ),
  action_call_or_failure(Action, (
    forall(rdf_has(ReqType,dul:hasPart,DataSlot), 
      once(action_filler_for(_:InputDict,DataSlot:ActionDict)))
    ),
    knowrob:'ACTION_INPUT_MISSING',
    'request slot(s) missing'
  ),
  %%%%%%%%%
  %%%%% Create request and response message
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Response,dul:realizes,ResType),
  rdf_assert(Action,ros:hasResponse,Response),
  create_ros_request(Action,InputDict,ActionDict,ReqType,Request),
  %%%%%%%%%
  %%%%% Call the service
  catch(
    rosowl_service_call(Service, Request, Response),
    ros_error(Error),
    throw(action_failure(Action, Error, 'ROS service call failed'))
  ),
  %%%%%%%%%
  %%%%% Assign roles of response slots
  findall(R-X, (
    rdf_has(ResType,dul:hasPart,DataSlot),
    rdf_has(Response,dul:hasPart,Slot),
    rdf_has(Slot,dul:realizes,DataSlot),
    get_dict(R, ActionDict, Slot),
    get_slot_filler(Slot,X),
    ( rdfs_individual_of(DataSlot,ros:'StatusSlot') ->
      % handle dedicated status field
      ros_querying_set_status(Action,DataSlot,X) ;
      % handle roles and parameters
      action_add_filler(Action,X)
    )
  ),OutputPairs).

get_slot_filler(Slot,Obj) :-
  ( rdfs_individual_of(Slot,ros:'PrimitiveValue') ;
    rdfs_individual_of(Slot,ros:'PrimitiveArray') ), !,
  rdf_has(Slot, dul:hasRegion, Obj).
get_slot_filler(Slot,Obj) :-
  % a message with a region value
  rdfs_individual_of(Slot,ros:'Message'),
  rdf_has(Slot, dul:hasRegion, Obj),!.
get_slot_filler(Slot,Slot).

% update action status according to
% status field of response message
ros_querying_set_status(Action,DataSlot,SlotRegion) :-
  % get ROS status code
  rdf_has_prolog(SlotRegion,dul:hasDataValue,StatusCode),
  % find mapping into region
  rdf_has(DataSlot,ros:hasRegionMapping,Mapping),
  rdf_has_prolog(Mapping,dul:hasDataValue,StatusCode),
  rdf_has(       Mapping,dul:hasRegion,Status),
  % update the status
  set_action_status(Action,Status), !.
ros_querying_set_status(Action,_,SlotRegion) :-
  rdf_has_prolog(SlotRegion,dul:hasDataValue,StatusCode),
  rdfs_split_url(_,AName,Action),
  writef('[WARN] Action %w has unknown status %w.', [AName,StatusCode]),
  set_action_status(Action,SlotRegion).

%% task_is_executed_in(Task, ActionConcept, ExecutionPlan)
%
% Finds actions that could execute a task.
% - ActionExecutionPlan may define tasks and restricts how they are to be exectued
%
task_isExecutedIn(WF_Task,ActionConcept,ActionExecutionPlan) :-
  % individual tasks in workflows maybe defined by action execution plans
  rdf_has(ActionExecutionPlan, dul:definesTask, WF_Task),
  rdfs_individual_of(ActionExecutionPlan,ease:'ActionExecutionPlan'),
  % TODO: avoid redundant results here
  owl_property_range_on_subject(ActionExecutionPlan,dul:definesTask,TaskClass),
  owl_property_range_on_class(TaskClass,dul:isExecutedIn,ActionConcept),
  rdfs_subclass_of(ActionConcept, dul:'Action'),
  \+ rdf_equal(ActionConcept, dul:'Action').

%% task_isExecutionPossible(Task)
%
% True iff a goal is known that implements an action
% which would execute the task.
%
task_isExecutionPossible(Task) :-
  task_isExecutedIn(Task,ActionConcept,_),
  action_registry(ActionConcept,_),!.

%% execute_task(+Task,+InputDict,-Action,-OutputDicts)
%
execute_task(Task,InputDict,Action,OutputDicts) :-
  %%%%%%%%%
  %%%%% Find action concept
  %%%%%%%%%
  task_isExecutedIn(Task,ActionConcept,ExecutionPlan),
  action_registry(ActionConcept,ActionGoal),
  action_bindings(ExecutionPlan,ActionDict),
  %%%%%%%%%
  %%%%% Instantiate action concept
  %%%%%%%%%
  create_action_symbol(ActionConcept,Task,ExecutionPlan,InputDict,Action),
  !, % TODO <-- this kills choicepoints for different ways to execute the task
  lazy_findall(OutputDict,
    execute_task_(Action,ActionGoal,ExecutionPlan,InputDict,ActionDict,OutputDict),
    OutputDicts).

execute_task_(Action,ActionGoal,ExecutionPlan,InputDict,ActionDict,OutputDict) :-
  %%%%%%%%%
  %%%%% Run the action!
  %%%%%%%%%
  ( catch(
    owl_run_event(Action, ActionGoal, [ExecutionPlan,InputDict,ActionDict,OutputPairs]),
    action_failure(Action, Failure, Message),
    handle_action_failure(Action, Failure, Message)) *->
    (
      ( var(OutputPairs) -> OutputPairs=[] ; true ),
      dict_pairs(OutputDict,_,OutputPairs)
    ) ; (
      % some unknown error occured
      handle_action_failure(Action,
        knowrob:'ACTION_PREDICATE_ERROR',
        'calling the action predicate failed'),
      OutputDict=_{}
    )
  ),
  %%%%%%%%%
  %%%%% Handle action status
  %%%%%%%%%
  assign_status_region(Action,OutputDict).

		 /*******************************
		 *	ACTION SYMBOL		*
		 *******************************/

%% create_action_symbol(ActionConcept,Task,ExecutionPlan,InputDict,Action)
%
% Createsa new action symbols and assigns participants
% and regions from roles and parameters of a task.
%
create_action_symbol(ActionConcept,Task,ExecutionPlan,InputDict,Action) :-
  % create action symbol
  rdf_instance_from_class(ActionConcept,Action),
  rdf_assert(Action,dul:executesTask,Task),
  % create status region
  rdf_instance_from_class(ease:'ActionStatus',ActionStatus),
  rdf_assert(Action, ease:hasStatus, ActionStatus),
  rdf_assert(ActionStatus, dul:hasRegion, knowrob:'ACTION_OK'),
  ( rdf_has(ExecutionPlan,dul:describes,IFace) ->
    rdf_assert(Action,dul:hasParticipant,IFace) ; true ),
  %%%%%%%%
  dict_pairs(InputDict,_,Pairs),
  forall(
    member(_Key-Value,Pairs),
    action_add_filler(Action,Value)
  ).


%% action_add_filler(Action,RegionOrObject)
%
action_add_filler(Action,X) :-
  rdfs_individual_of(X,dul:'Region'),!, (
    rdf_has(Action,dul:hasRegion,X);
    rdf_assert(Action,dul:hasRegion,X)
  ), !.
action_add_filler(Action,X) :-
  rdfs_individual_of(X,dul:'Object'),!,
  ( rdf_has(Action,dul:hasParticipant,X);
    rdf_assert(Action,dul:hasParticipant,X) ).
action_add_filler(_Action,X) :-
  writef('[WARN] %w is not a Region or Object!\n', [X]),
  fail.

%% action_filler_for(Filler:InputDict,Entity:ActionDict)
%
% Find region or participant of action that is classified
% with the same concept as some other entity.
%
action_filler_for(Filler:InputDict,Entity:ActionDict) :-
  get_dict(Concept,InputDict,Filler),
  get_dict(Concept,ActionDict,Entity).

action_bindings(ExecutionPlan,ActionDict) :-
  findall(R-X, (
    rdf_has(ExecutionPlan, ease_wf:hasBinding, B),
    rdf_has(B, ease_wf:hasBindingFiller, X),
    rdf_has(B, ease_wf:hasBindingRole, R)),
    Pairs),
  dict_pairs(ActionDict,_,Pairs).

		 /*******************************
		 *	ACTION STATUS		*
		 *******************************/

%% action_status(Action,Region)
%
% True iff Region is the *final* status of Action.
%
action_status(Action,Region) :-
  rdf_has(Action,ease:hasStatus,ActionStatus),
  rdf_has(ActionStatus,dul:hasRegion,Region).

% tasks may have a Status parameter that wants to
% classify the region of the final status of the action
get_task_status_parameter(Task,Parameter) :-
  rdf_has(Task,dul:definesParameter, Parameter),
  rdfs_individual_of(Parameter, ease:'Status'), !.

handle_action_failure(Action,Failure,Message) :-
  log_action_failure(Action,Failure,Message),
  action_set_status(Action,Failure).

log_action_failure(Action,Failure,Message) :-
  rdf_split_url(_, A_name, Action),
  rdf_split_url(_, F_name, Failure),
  writef('    [action_execution] Execution of %w failed: %w (%w).\n', [A_name,Message,F_name]).

% throw an action_failure if the call fails
action_call_or_failure(Action,Goal,Failure,Message) :-
  call(Goal) *-> true ; throw(action_failure(Action,Failure,Message)).

assign_status_region(Action,_OutputDict) :-
  action_status(Action,StatusRegion),
  rdf_has(Action,dul:hasRegion,StatusRegion),!.
assign_status_region(Action,OutputDict) :-
  % assign current status as participant and classify it by Status parameter
  action_status(Action,StatusRegion),
  rdf_assert(Action,dul:hasRegion,StatusRegion),
  rdf_has(Action,dul:executesTask,Task),
  ( get_task_status_parameter(Task,StatusParameter) ->
    b_set_dict(StatusParameter, OutputDict, StatusRegion) ; true ).

%% plan_start_action(+ActionClass:iri, -ActionInstance:iri) is semidet.
%
% ActionInstance is a newly created instance of type ActionClass.
%
% @param ActionClass Some sublcass of knowrob:'Action'
% @param ActionInstance Instance of ActionClass
% 
plan_start_action(ActionClass, ActionInstance) :-
  current_time(Now),
  % create instance of the action
  rdf_instance_from_class(ActionClass, ActionInstance),
  rdf_assert_prolog(ActionInstance, knowrob:'startTime', Now).

%% plan_finish_action(+ActionInstance:iri) is semidet.
%
% Specify the endTime of ActionInstance and project its effects.
%
% @param ActionInstance Instance of ActionClass
% 
plan_finish_action(ActionInstance) :-
  current_time(Now),
  % specify endTime and project the action effects
  rdf_assert_prolog(ActionInstance, knowrob:'endTime', Now),
  action_effects_apply(ActionInstance).
