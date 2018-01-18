/*
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
*/

:- module(knowrob_actions,
    [
      action_objectActedOn/2,
      action_toLocation/2,
      action_fromLocation/2,
      action_inputs/2,
      action_outputs/2,
      action_missing_inputs/2
    ]).
/** <module> Methods for reasoning about action descriptions

@author Moritz Tenorth
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).


:- rdf_meta
      action_objectActedOn(r,r),
      action_toLocation(r,r),
      action_fromLocation(r,r),
      action_inputs(r,r),
      action_missing_inputs(r,r),
      action_outputs(r,r).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

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

%% action_inputs(+Action, -Input)
%
% Required inputs for an action
%
% @param Action   Action class
% @param Input    Input linked via a preActors restriction
%
action_inputs(Action, Input) :-
  owl_class_properties(Action, knowrob:'preActors', Input).

%% action_missing_inputs(+Action, -Missing)
%
% Missing inputs of an action (required, but not available)
%
% @param Action   Action class
% @param Missing  Input linked via a preActors restriction, but not available
%
action_missing_inputs(Action, Missing) :-
  findall(Pre, (action_inputs(Action, Pre), \+ resource_available(Pre)), Missing).

%% action_outputs(+Action, -Output)
%
% Outputs of an action
%
% @param Action   Action class
% @param Output   Output linked via a postActors restriction
%
action_outputs(Action, Output) :-
  owl_class_properties(Action, knowrob:'postActors', Output).
%TODO: check class subsumption (allow more complex requirements)

%% resource_available(+Resource)
%
% Resource is available
%
% @param Resource Resource whose availability is to be checked (e.g. object class, check if instance of that class exists)
%
resource_available(Resource) :-
  owl_individual_of(ObjInst, Resource),
  % HACK
  \+ rdfs_individual_of(ObjInst, knowrob:'TemporalPart').


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
