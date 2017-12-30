/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
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

:- module(action_planning,
    [
      plan_step/1,                % update active processes
      plan_start_action/2,        % create new action instance and assign startTime
      plan_finish_action/1,       % assign endTime and project action effects
      plan_subevents/2,           % ordered list of sub-actions
      plan_subevents_recursive/2,
      plan_objects/2,             % all object types involved in performing a complexaction
      plan_constrained_objects/3  % link outputs of previous actions as inputs of another action
    ]).
/** <module> Methods for reasoning about object changes caused by actions

@author Moritz Tenorth
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).

:- use_module(library('knowrob/action_effects')).
:- use_module(library('knowrob/actions')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake, 'http://knowrob.org/kb/pancake-making.owl#', [keep(true)]).

:-  rdf_meta
      plan_subevents(r,-),
      plan_subevents_recursive(r,r),
      plan_objects(r,r),
      plan_constrained_objects(r,r,t).

%% plan_subevents(+Plan:iri, ?SubEvents:list) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @tbd  Unify with plan_subevents_recursive (return list or single instances)
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents(Plan, SubEvents) :-
  findall(SubAction, (owl_class_properties(Plan, knowrob:subAction, SubAction)), Sub),
  predsort(knowrob_actions:compare_actions_partial_order, Sub, SubEvents).

%% plan_subevents_recursive(+Plan:iri, ?SubEvents:list) is semidet.
%
% Recursively read all sub-action classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan      Plan identifier
% @param SubEvents Sub-events of the plan
%
plan_subevents_recursive(Plan, SubAction) :-
  owl_class_properties(Plan, knowrob:subAction, SubAction).

plan_subevents_recursive(Plan, SubAction) :-
  owl_class_properties(Plan, knowrob:subAction, Sub),
  Sub \= Plan,
  plan_subevents_recursive(Sub, SubAction).

%% plan_constrained_objects(+Plan:iri, +Action:iri, +PrevActions:list)
%
% Link outputs of previous actions to `Action` via `objectActedOn` property
% based on the description of IO constraints.
%
% @param Plan        Plan identifier
% @param Action      Action identifier
% @param PrevActions List of previous actions performed within the plan
%
plan_constrained_objects(Plan, Action, PrevActions) :-
  findall(Obj, plan_constrained_objects(Plan, Action, PrevActions, Obj), Objs),
  forall(member(Obj,Objs), rdf_assert(Action, knowrob:objectActedOn, Obj)).
  
plan_constrained_objects(_Plan, Action, PrevActions, Obj) :-
  % TODO: check if this constraint is part of plan!
  %owl_class_properties(Plan, knowrob:inputOutputConstraint, Constraint),
  rdfs_individual_of(Action, Cls),
  rdf_has(Constraint, knowrob:requiresInput, Cls),
  % read the outputs created by previous actions
  plan_constrained_object(Constraint, PrevActions, Obj).

plan_constrained_object(Constraint, PrevActions, Object) :-
  rdf_has(Constraint, knowrob:createsOutput, OtherCls),
  member(OtherAction, PrevActions),
  rdfs_individual_of(OtherAction, OtherCls), !,
  % query the (typed) output created
  rdf_has(OtherAction, knowrob:outputsCreated, Object),
  (  rdf_has(Constraint, knowrob:resourceType, OutputType)
  -> owl_individual_of(Object, OutputType)
  ;  true ).

%% plan_objects(+Plan:iri, -Objects:list) is semidet.
%
% Read all objects mentioned in sub-actions of the imported plan
%
% @param Plan Plan identifier
% @param SubEvents List of objects of the plan
% 
plan_objects(Plan, Objects) :-
  plan_subevents(Plan, SubEvents),
  findall(Obj,
    (member(SubEvent, SubEvents),
     action_objectActedOn(SubEvent, Obj)), Objects).

%% plan_start_action(+ActionClass:iri, -ActionInstance:iri) is semidet.
%
% ActionInstance is a newly created instance of type ActionClass.
%
% @param ActionClass Some sublcass of knowrob:'Action'
% @param ActionInstance Instance of ActionClass
% 
plan_start_action(ActionClass, ActionInstance) :-
  plan_step(Now),
  owl_instance_from_class(knowrob:'TimePoint', [instant=Now], Timepoint),
  % create instance of the action
  rdf_instance_from_class(ActionClass, ActionInstance),
  rdf_assert(ActionInstance, knowrob:'startTime', Timepoint).

%% plan_finish_action(+ActionInstance:iri) is semidet.
%
% Specify the endTime of ActionInstance and project its effects.
%
% @param ActionInstance Instance of ActionClass
% 
plan_finish_action(ActionInstance) :-
  plan_step(Now),
  owl_instance_from_class(knowrob:'TimePoint', [instant=Now], Timepoint),
  % specify endTime and project the action effects
  rdf_assert(ActionInstance, knowrob:'endTime', Timepoint),
  action_effects_apply(ActionInstance).

%% plan_step(-Now:float) is semidet.
%
% Updates ongoing processes.
%
plan_step(Now) :-
  current_time(Now),
  forall(plan_active_process(Process),
         action_effects_apply(Process)).

plan_active_process(Process) :-
  findall(Process, (
      % processes are started by some action
      rdf_has(_, knowrob:processStarted, Process),
      % processes are active if no endTime is specified
      \+ rdf_has(Process, knowrob:'endTime', _)
  ), X),
  list_to_set(X, Processes),
  member(Process, Processes).
