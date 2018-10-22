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
      action_constituents/3,
      action_boundary_constraints/2,
      action_objectActedOn/2,
      action_toLocation/2,
      action_fromLocation/2,
      action_inputs/2,
      action_outputs/2,
      action_missing_inputs/2,
      action_requires/3,
      action_class_requires/3,
      comp_action_participant/3,
      physical_actions/2
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
      action_constituents(r,t,t),
      action_boundary_constraints(r,t),
      action_objectActedOn(r,r),
      action_toLocation(r,r),
      action_fromLocation(r,r),
      action_inputs(r,r),
      action_missing_inputs(r,r),
      action_outputs(r,r),
      action_requires(r,r,r),
      action_class_requires(r,r,r),
      transformed_into(r, r),
      transformed_into_transitive(r,r),
      comp_action_participant(r,r,r).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease, 'http://www.ease.org/ont/EASE.owl#', [keep(true)]).

%%
action_constituents(Evt,Constituents,Constraints) :-
  findall(Type-Constraints, (
    rdfs_subclass_of(Evt, Restr),
    % FIXME this assumes particular structure of axioms
    owl_description_recursive(Restr,
        restriction(P, some_values_from(X))),
    rdfs_subproperty_of(P,dul:hasConstituent),
    ( X=intersection_of([Type|Constraints]) ;
    ( Type=class(_), Type=X, Constraints=[] )),
    % HACK better check which ones are subclass of each other and "merge"
    Type=class(Iri),
    \+ rdf_equal(Iri,ease:'PhysicsProcess'),
    \+ rdf_equal(Iri,dul:'Action')),
    TypeConstraints),
  findall(Constituent,
    member(class(Constituent)-_, TypeConstraints),
    Constituents),
  findall(Constraint, (
    member(class(A)-Axioms, TypeConstraints),
    member(restriction(P, some_values_from(B)),Axioms),
    allen_constraint(A,P,B,Constraint)),
    Constraints).

%%
action_boundary_constraints(Evt,Constraints) :-
  findall(Constraint, (
    rdfs_subclass_of(Evt, Restr),
    owl_description(Restr, restriction(P, some_values_from(B))),
    allen_constraint(Evt,P,B,Constraint)),
    Constraints).

%% action_objectActedOn(?Action, ?Object) is nondet.
%
% Reads the objectActedOn for a TBOX action description
%
% @param Action  Action class with a restriction on the objectActedOn
% @param Object  Value set as objectActedOn for this action
% 
action_objectActedOn(Action, Object) :-
  action_class_requires(Action,knowrob:objectActedOn,Object).

%% action_toLocation(?Action, ?Loc) is nondet.
%
% Reads the toLocation for a TBOX action description
%
% @param Action Action class with a restriction on the toLocation
% @param Loc    Value set as toLocation for this action
% 
action_toLocation(Action, Loc) :-
  action_class_requires(Action,knowrob:toLocation,Loc).

%% action_fromLocation(?Action, ?Loc) is nondet.
%
% Reads the fromLocation for a TBOX action description
%
% @param Action Action class with a restriction on the fromLocation
% @param Loc    Value set as fromLocation for this action
% 
action_fromLocation(Action, Loc) :-
  action_class_requires(Action,knowrob:fromLocation,Loc).

%% action_inputs(+Action, -Input)
%
% Required inputs for an action
%
% @param Action   Action class
% @param Input    Input linked via a preActors restriction
%
action_inputs(Action, Input) :-
  action_class_requires(Action,knowrob:preActor,Input).

%% action_outputs(+Action, -Output)
%
% Outputs of an action
%
% @param Action   Action class
% @param Output   Output linked via a postActors restriction
%
action_outputs(Action, Output) :-
  action_class_requires(Action,knowrob:postActor,Output).

%% action_missing_inputs(+Action, -Missing)
%
% Missing inputs of an action (required, but not available)
%
% @param Action   Action class
% @param Missing  Input linked via a preActors restriction, but not available
%
action_missing_inputs(Action, Missing) :-
  findall(Pre, (action_inputs(Action, Pre), \+ resource_available(Pre)), Missing).

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 'Physical action'

% set of all 'Physical action' subclasses without subclass
physical_actions(ActionSet) :-
  physical_actions(ActionSet,user).

physical_actions(ActionSet,RDFGraph) :-
  findall(ActionClass, (
    rdfs_subclass_of(ActionClass, ease:'PhysicalAction'),
    once((
      rdf(ActionClass, rdfs:subClassOf, _, RDFGraph),
      \+ rdf(_, rdfs:subClassOf, ActionClass)
    ))
  ), Actions),
  list_to_set(Actions, ActionSet).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % %  TODO Below could be moved into common module.
% % % %    - assert_triple_most_specific is redundant with code in knowrob_planning

% FIXME: owl_cardinality_on_* could include
%   general and specific axioms! prefer specific axioms

%%
action_requires(S,P,O) :-
  owl_cardinality_on_subject(S,P,O,cardinality(Min,_)),
  Min > 0.

%%
action_class_requires(S,P,O) :-
  owl_cardinality_on_class(S,P,O,cardinality(Min,_)),
  Min > 0.

%%
comp_action_participant(Entity,P,Symbol) :-
  ground(Entity),
  owl_cardinality_on_subject(Entity, P, Type, cardinality(Min,_)),
  % TODO: only Min - Actual choicepoints!
  between(1,Min,_), % gen `Min` choicepoints
  rdf_instance_from_class(Type,Symbol).
