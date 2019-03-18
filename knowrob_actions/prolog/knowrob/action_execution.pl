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
      action_registry/2,
      action_status/2,
      set_action_status/2,
      task_isExecutionPossible/1,
      task_isExecutedIn/3,
      execute_task/2,
      action_call_or_failure/4,
      action_add_filler/2,
      action_filler_for/3,
      create_action_symbol/3
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

:- rdf_meta action_registry(r,?),
            action_status(r,r),
            action_filler_for(r,r,r),
            set_action_status(r,r),
            action_call_or_failure(r,t,r,+),
            task_isExecutedIn(r,r,r),
            task_isExecutionPossible(r),
            handle_action_failure(r,r,+),
            execute_task(r,r),
            create_action_symbol(r,r,-).

%% action_registry(ActionConcept, Goal)
%
% A multifile predicate that defines Prolog goals
% that would when called perform some action.
%
:- multifile action_registry/2.

%% task_isExecutedIn(Task, ActionConcept, ExecutionPlan)
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

%% execute_task(+Task,-Action)
%
% The procedure as as follows:
% (1) Find action concept that executes task
% (2) Instantiate action concept
% (3) Run action
% (4) Handle action status
%
execute_task(WF_Task,Action) :-
  %%%%%%%%%
  %%%%% Find action concept
  %%%%%%%%%
  task_isExecutedIn(WF_Task,ActionConcept,_ExecutionPlan),
  action_registry(ActionConcept,ActionGoal),
  %%%%%%%%%
  %%%%% Instantiate action concept
  %%%%%%%%%
  create_action_symbol(ActionConcept,WF_Task,Action),
  !, % TODO <-- this kills choicepoints for different ways to execute the task
  %%%%%%%%%
  %%%%% Run the action!
  %%%%%%%%%
  ( catch(
    owl_run_event(Action, ActionGoal),
    action_failure(Action, Failure, Message),
    handle_action_failure(Action, Failure, Message)) *-> true ; (
    % some unknown error occured
    handle_action_failure(Action,
        knowrob:'ACTION_PREDICATE_ERROR',
        'calling the action predicate failed')
  )),
  %% TODO: think about choicepoints
  %%          - task could be executable in different ways
  %%          - action goal may return many results, one at a time
  %%          - assignment to roles/parameters is problematic
  %%          - maybe restrict task execution to use interface that does some bookkeeping,
  %%              and can switch to next solution.
  %% !!! talk to Mihai since he has same problems with workflow
  !, % kill action goal choicepoints for now :(
  %%%%%%%%%
  %%%%% Handle action status
  %%%%%%%%%
  assign_status_region(Action).

		 /*******************************
		 *	ACTION SYMBOL		*
		 *******************************/

%% create_action_symbol(ActionConcept,Task,Action)
%
% Createsa new action symbols and assigns participants
% and regions from roles and parameters of a task.
%
create_action_symbol(ActionConcept,Task,Action) :-
  % create action symbol
  rdf_instance_from_class(ActionConcept,Action),
  rdf_assert(Action,dul:executesTask,Task),
  % create status region
  rdf_instance_from_class(ease:'ActionStatus',ActionStatus),
  rdf_assert(Action, ease:hasStatus, ActionStatus),
  rdf_assert(ActionStatus, dul:hasRegion, knowrob:'ACTION_OK'),
  %%%%%%%%
  assign_action_participants(Action,Task),
  assign_action_regions(Action,Task).

assign_action_participants(Action,Task) :-
  % TODO Classification of hasParticipant property?
  %         - some actions may want more specific roles e.g. askedBy or answeredBy
  %         - but I think this is not safe to do, since different sup-properties
  %            not necessary have disjunctive (askedBy and answeredBy both Agent)
  %         - action implementation can do all sorts of hacks to get it done, here should be clean
  %
  % rdfs_assert_specific <--
  forall((
    rdf_has(Task,dul:isTaskOf,Role),
    owl_has(X,dul:isClassifiedBy,Role)),
    once((
      rdf_has(Action,dul:hasParticipant,X);
      rdf_assert(Action,dul:hasParticipant,X)
    ))
  ).
assign_action_regions(Action,Task) :-
  forall((
    rdf_has(Task,dul:hasParameter,Parameter),
    owl_has(X,dul:isClassifiedBy,Parameter)),
    once((
      rdf_has(Action,dul:hasRegion,X);
      rdf_assert(Action,dul:hasRegion,X)
    ))
  ).

%% action_filler_for(Action,RegionOrObject)
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

%% action_filler_for(Action,Entity,Filler)
%
% Find region or participant of action that is classified
% with the same concept as some other entity.
%
action_filler_for(Action,Entity,Filler) :-
  ( rdf_has(Action,dul:hasRegion,Filler) ;
    rdf_has(Action,dul:hasParticipant,Filler) ),
  once((
    owl_has(Filler,dul:isClassifiedBy,Concept),
    owl_has(Entity,dul:isClassifiedBy,Concept),
    Filler \= Entity
  )).

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

%% set_action_status(Action,Region)
%
% update the quale of the ActionStatus quality
set_action_status(Action,Region) :-
  rdfs_individual_of(Region,dul:'Region'),
  rdf_has(Action,ease:hasStatus,ActionStatus),
  rdf_retractall(ActionStatus,dul:hasRegion,_),
  rdf_assert(ActionStatus,dul:hasRegion,Region).

% tasks may have a Status parameter that wants to
% classify the region of the final status of the action
get_task_status_parameter(Task,Parameter) :-
  rdf_has(Task,dul:definesParameter, Parameter),
  rdfs_individual_of(Parameter, ease:'Status'), !.

handle_action_failure(Action,Failure,Message) :-
  log_action_failure(Action,Failure,Message),
  set_action_status(Action,Failure).

log_action_failure(Action,Failure,Message) :-
  rdf_split_url(_, A_name, Action),
  rdf_split_url(_, F_name, Failure),
  writef('    [action_execution] Execution of %w failed: %w (%w).\n', [A_name,Message,F_name]).

% throw an action_failure if the call fails
action_call_or_failure(Action,Goal,Failure,Message) :-
  call(Goal) *-> true ; throw(action_failure(Action,Failure,Message)).

assign_status_region(Action) :-
  % assign current status as participant and classify it by Status parameter
  action_status(Action,StatusRegion),
  rdf_assert(Action,dul:hasRegion,StatusRegion),
  rdf_has(Action,dul:executesTask,Task),
  ( get_task_status_parameter(Task,StatusParameter) ->
    rdf_assert(StatusRegion,dul:isClassifiedBy,StatusParameter) ; true ).
