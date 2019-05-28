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

:- module(action_planning,
    [
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
:- use_module(library('knowrob/ESG')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).

:-  rdf_meta
      plan_subevents(r,-),
      plan_subevents_recursive(r,r),
      plan_objects(r,r),
      plan_constrained_objects(r,r,t).

%% plan_subevents(+Plan:iri, ?SubEvents:list) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents(Act, SubEvents) :-
  % find constituents and their relation to each other
  action_constituents(Act,Constituents,Constituent_Constraints),
  % gather allen constraints about the occurance of Act
  action_boundary_constraints(Act,Act_Constraints),
  append(Constituent_Constraints,Act_Constraints,Constraints),
  esg_truncated(Act,Constituents,Constraints,[Sequence,_,_]),
  esg_events(Sequence,[_|SubEvents]).

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

%% plan_objects(+Plan:iri, -Objects:list) is semidet.
%
% Read all objects mentioned in sub-actions of the imported plan
%
% @param Plan Plan identifier
% @param SubEvents List of objects of the plan
% 
plan_objects(Plan, Objects_set) :-
  plan_subevents(Plan, SubEvents),
  findall(Obj,
    (member(SubEvent, SubEvents),
     action_objectActedOn(SubEvent, Obj)), Objects),
  list_to_set(Objects,Objects_set).

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
