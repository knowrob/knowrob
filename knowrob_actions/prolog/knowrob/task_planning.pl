/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2018 Daniel Beßler
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

:- module(task_planning,
    [
      plan_subevents/2,           % ordered list of sub-actions
      plan_subevents_recursive/2,
      plan_objects/2,             % all object types involved in performing a complexaction
      plan_constrained_objects/3, % link outputs of previous actions as inputs of another action
      workflow_steps/3,
      workflow_processes/3,
      process_flow_steps/3
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
:- use_module(library('knowrob/event_graph')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).

:-  rdf_meta
      plan_subevents(r,-),
      plan_subevents(r,r,-),
      plan_subevents_recursive(r,r),
      plan_objects(r,r),
      plan_constrained_objects(r,r,t),
      workflow_steps(r,t,t),
      workflow_processes(r,t,t).

%% plan_subevents(+Tsk:iri, ?SubTasks:list) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @param Tsk Task identifier
% @param WF A workflow
% @param SubTasks List of sub-tasks of the plan
%
plan_subevents(Tsk, SubTasks) :-
  rdfs_individual_of(Tsk0,Tsk),
  rdf_has(WF,dul:definesTask,Tsk0),% FIXME definesTask too general
  rdfs_individual_of(WF,dul:'Workflow'),
  plan_subevents(Tsk0, WF, SubTasks).

plan_subevents(Tsk, WF, SubTasks) :-
  % find steps and their relation to each other
  workflow_steps(WF,Steps,Step_Constraints),
  % gather allen constraints about the occurance of Act
  findall(X, allen_constraint(Tsk,X), Task_Constraints),
  append(Step_Constraints,Task_Constraints,Constraints),
  esg_truncated(Tsk,Steps,Constraints,[Sequence,_,_]),
  esg_events(Sequence,[_|SubTasks]).

%% plan_subevents_recursive(+Plan:iri, ?SubEvents:list) is semidet.
%
% Yield recursive sub-actions in the order they should be performed.
%
% @param Act       Action class
% @param SubAction The next sub-action
%
plan_subevents_recursive(Act, SubAction) :-
  plan_subevents(Act, Xs),
  member(X,Xs),
  ( SubAction=X ;
    plan_subevents_recursive(X,SubAction) ).

%% plan_objects(+Plan:iri, -Objects:list) is semidet.
%
% Read all objects mentioned in sub-actions of the imported plan
%
% @param Plan Plan identifier
% @param SubEvents List of objects of the plan
% 
plan_objects(Tsk, ObjectTypes) :-
  plan_subevents(Tsk, SubEvents),
  findall(ObjectType, (
    member(SubEvent, SubEvents),
    plan_object(SubEvent, ObjectType)
  ), X),
  list_to_set(X,ObjectTypes).

plan_object(Tsk,ObjectType) :-
  rdf_has(Tsk,rdf:type,TskType),
  action_effects:task_class_has_role(TskType,_Role,ObjectType),
  \+ rdf_equal(ObjectType,dul:'Object').

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
  forall(
    plan_constrained_objects(Plan, Action, PrevActions, Obj),
    rdfs_assert_specific(Action, knowrob:objectActedOn, Obj)
  ).
  
plan_constrained_objects(_Plan, Action, _PrevActions, Obj) :-
  % TODO: only use outputs from prev actions!
  action_requires(Action,knowrob:objectActedOn,OutputType),
  plan_constrained_object(OutputType, Obj).

plan_constrained_object(OutputType, Object) :-
  rdf_has(_, knowrob:outputsCreated, Object),
  \+ rdf_has(_, knowrob:inputsDestroyed, Object),
  owl_individual_of(Object, OutputType), !.

		 /*******************************
		 *	Process flows		*
		 *******************************/

%%
process_flow_steps(WF,Steps,Constraints) :-
  findall(X, rdf_has(WF, ease_proc:hasStage, X), Steps), 
  findall(Constraint, (
    member(Step,Steps),
    allen_constraint(Step,Constraint)
  ), Constraints).

		 /*******************************
		 *	Workflows		*
		 *******************************/

workflow_is_for_task(WF,Tsk) :- fail. % TODO

%%
workflow_event_graph(WF,ESG) :-
  workflow_is_for_task(WF,Tsk),
  findall(X, (
    rdf_has(WF, dul:defines, X),
    rdfs_individual_of(X, dul:'EventType')
  ), EventTypes), 
  findall(Constraint, (
    member(EventType,EventTypes),
    allen_constraint(EventType,Constraint)
  ), Constraints).
  %
  esg_truncated(Tsk,EventTypes,Constraints,ESG).
  

%%
workflow_steps(WF,Steps,Constraints) :-
  findall(X, rdf_has(WF, ease_wf:hasStep, X), Steps), 
  findall(Constraint, (
    member(Step,Steps),
    allen_constraint(Step,Constraint)
  ), Constraints).

workflow_processes(WF,Procs,Constraints) :-
  findall(X, rdf_has(WF, ease_proc:definesProcess, X), Procs), 
  findall(Constraint, (
    member(Proc,Procs),
    allen_constraint(Proc,Constraint)
  ), Constraints).

%%
allen_constraint(X,Constraint) :-
  rdf_has(X,Relation,Other),
  atom(Relation),
  %rdfs_individual_of(Other,dul:'Task'),
  rdf_has_prolog(Relation,ease:symbol,Symbol), % FIXME: bad test
  Constraint =.. [Symbol,X,Other].
