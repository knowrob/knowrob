/** <module> Methods for reasoning about action descriptions
  
  Copyright (C) 2011 Moritz Tenorth
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

@author Moritz Tenorth
@license BSD

*/

:- module(knowrob_actions,
    [
      plan_subevents/2,
      plan_subevents_recursive/2,
      plan_objects/2,
      plan_constrained_objects/3,
      action_objectActedOn/2,
      action_toLocation/2,
      action_fromLocation/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).


:- rdf_meta
      plan_subevents(r,-),
      plan_subevents_recursive(r,r),
      plan_objects(r,r),
      plan_constrained_objects(r,r,t),
      action_objectActedOn(r,r),
      action_toLocation(r,r),
      action_fromLocation(r,r).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).


%% plan_subevents(+Plan, ?SubEvents) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @tbd  Unify with plan_subevents_recursive (return list or single instances)
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents(Plan, SubEvents) :-
  findall(SubAction, (class_properties(Plan, knowrob:subAction, SubAction)), Sub),
  predsort(compare_actions_partial_order, Sub, SubEvents).



%% plan_subevents_recursive(+Plan, ?SubEvents) is semidet.
%
% Recursively read all sub-action classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan      Plan identifier
% @param SubEvents Sub-events of the plan
%
plan_subevents_recursive(Plan, SubAction) :-
    class_properties(Plan, knowrob:subAction, SubAction).

plan_subevents_recursive(Plan, SubAction) :-
    class_properties(Plan, knowrob:subAction, Sub),
    Sub \= Plan,
    plan_subevents_recursive(Sub, SubAction).


%% plan_constrained_objects(+Plan, +Action, +PrevActions)
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
  
plan_constrained_objects(Plan, Action, PrevActions, Obj) :-
  % FIXME: check if this constraint is part of plan!
  %class_properties(Plan, knowrob:inputOutputConstraint, Constraint),
  rdf_has(Constraint, knowrob:requiresInput, Cls),
  rdfs_individual_of(Action, Cls),
  % read the outputs created by previous actions
  plan_constrained_object(Constraint, PrevActions, Obj).

plan_constrained_object(Constraint, PrevActions, Object) :-
  rdf_has(Constraint, knowrob:createsOutput, OtherCls),
  member(OtherAction, PrevActions),
  % TODO: take latest action output if multiple actions of same type were performed
  rdfs_individual_of(OtherAction, OtherCls), !,
  % query the (typed) output created
  rdf_has(OtherAction, knowrob:outputsCreated, Object),
  (  rdf_has(Constraint, knowrob:resourceType, OutputType)
  -> owl_individual_of(Object, OutputType)
  ;  true ).


%% plan_objects(+Plan, -Objects) is semidet.
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


%% action_objectActedOn(?Action, ?Object) is nondet.
%
% Reads the objectActedOn for a TBOX action description
%
% @param Action  Action class with a restriction on the objectActedOn
% @param Object  Value set as objectActedOn for this action
% 
action_objectActedOn(Action, Object) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction('http://knowrob.org/kb/knowrob.owl#objectActedOn', some_values_from(Object))).
action_objectActedOn(Action, Object) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction('http://knowrob.org/kb/knowrob.owl#objectActedOn', some_values_from(Object))).

%% action_toLocation(?Action, ?Loc) is nondet.
%
% Reads the toLocation for a TBOX action description
%
% @param Action Action class with a restriction on the toLocation
% @param Loc    Value set as toLocation for this action
% 
action_toLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction('http://knowrob.org/kb/knowrob.owl#toLocation', some_values_from(Loc))).
action_toLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction('http://knowrob.org/kb/knowrob.owl#toLocation', some_values_from(Loc))).

%% action_fromLocation(?Action, ?Loc) is nondet.
%
% Reads the fromLocation for a TBOX action description
%
% @param Action Action class with a restriction on the fromLocation
% @param Loc    Value set as fromLocation for this action
% 
action_fromLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction('http://knowrob.org/kb/knowrob.owl#fromLocation', some_values_from(Loc))).
action_fromLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction('http://knowrob.org/kb/knowrob.owl#fromLocation', some_values_from(Loc))).


%% compare_actions_partial_order(-Rel, +Act1, +Act2) is semidet.
%
% Compare predicate to be used in predsort for sorting a list of actions
% based on partial-order constraints. Checks if there is an ordering
% constraint that has these two actions as before/after
% 
% @tbd can we check if these constraints belong to the current task?
% @param Rel   Relation operator, i.e. either '<' or '>'
% @param Act1  Action class
% @param Act2  Action class
% 
compare_actions_partial_order('<', Act1, Act2) :-
  owl_has(Constraint, knowrob:occursBeforeInOrdering, Act1),
  owl_has(Constraint, knowrob:occursAfterInOrdering, Act2),!.

compare_actions_partial_order('>', Act1, Act2) :-
  owl_has(Constraint, knowrob:occursBeforeInOrdering, Act2),
  owl_has(Constraint, knowrob:occursAfterInOrdering, Act1),!.

% default: keep ordering if there are no matching ordering constraints
compare_actions_partial_order('<', _, _).
