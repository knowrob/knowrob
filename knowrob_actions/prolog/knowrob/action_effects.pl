/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2016-2017 Daniel Beßler
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
:- module(action_effects,
    [
      action_effects_apply/1,
      action_effect_apply/2,
      action_effect_on_object/2,
      action_precondition_check/1,
      action_precondition_check/2,
      comp_actionEffectRule/2
    ]).
/** <module> Prediction of action effects

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/swrl')).
:- use_module(library('knowrob/owl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_ns(action_effects,  'http://knowrob.org/kb/action-effects.owl#', [keep(true)]).

:-  rdf_meta
    action_effects_apply(r),
    action_effect_apply(r,r),
    action_effect_on_object(r,t),
    action_precondition_check(r),
    action_precondition_check(r,r),
    comp_actionEffectRule(r,r).

% TODO: event system for processes started by actions and their effects
%         - allow robot to react on predictable effects of processes

%% comp_actionEffectRule(+Action:iri, ?Effect:iri)
%
% Effect is a RDF description of a SWRL rule that 
% describes how the effect of Action can be projected
% into the knowledge base.
%
% @param Action Instance of knowrob:Action
% @param Effect RDF name of SWRL rule
%
comp_actionEffectRule(Action, Effect) :-
  rdfs_individual_of(Action, ActionClass),
  rdf_has(Effect, knowrob:swrlActionConcept, literal(type(_,ActionClass))).

%% action_effect_on_object(?ActionClass:iri, ?EffectTerm:term) is nondet
%
% Reasoning about which ActionClass (when the action is successfully performed)
% will have a desired effect on the object manipulated during the action.
% EffectTerm is a Prolog term describing the effect:
%
%	| updated(P,O)		   | If value O is specified for property P	  |
%	| created(Type)		   | If Type is a new type of the manipulated object	|
%	| destroyed(Type)	   | If Type is not a type of the manipulated object anymore after the action was performed	  |
%	| destroyed			   | If the object is not spatially existing anymore	  |
%
% `created` and `destroyed` effect terms follow the convention that type assertions
% are represented in rule heads as `Type(?obj)`, and type retractions as `(not Type)(?obj)`.
% 
% For example:
% ==
%     action_effect_on_object(knowrob:'TurningOnPoweredDevice',
%            updated(knowrob:stateOfObject,knowrob:'DeviceStateOn'))
%     action_effect_on_object(knowrob:'BakingFood',
%            created(knowrob:'Baked'))
%     action_effect_on_object(knowrob:'Cracking',
%            destroyed)
% ==
%
action_effect_on_object(ActionClass, updated(P,O)) :-
  % find SWRL action rule and the variable denoting the manipulated object within the rule
  action_effect_objectActedOn(ActionClass, Var_objectActedOn, Head :- Body),
  % the implication of the rule must assert a new value for property P
  member(property(Var_objectActedOn, P_rule, O_rule), Head),
  rdfs_subproperty_of(P, P_rule),
  % and the value type must match the specified value/type O
  swrl_type_of(Head :- Body, O_rule, O).

action_effect_on_object(ActionClass, created(Type)) :-
  action_effect_objectActedOn(ActionClass, Var_objectActedOn, Head :- _Body),
  member(class(Type_rule, Var_objectActedOn), Head),
  % TODO: Type could be Prolog term representing owl_class! (same for destroy case below)
  once(owl_subclass_of(Type, Type_rule)).

action_effect_on_object(ActionClass, destroyed) :-
  action_effect_on_object(ActionClass, destroyed('http://knowrob.org/kb/knowrob.owl#SpatialThing')).
action_effect_on_object(ActionClass, destroyed(Type)) :-
  action_effect_objectActedOn(ActionClass, Var_objectActedOn, Head :- _Body),
  member(class(X, Var_objectActedOn), Head),
  rdf_has(X, owl:complementOf, Type_rule),
  once(owl_subclass_of(Type, Type_rule)).

%% action_effects_apply(+Act:iri)
%
% Apply all the effects that are known to apply if Act is performed successfully.
%
% @param Act Instance of knowrob:'Action'
%
action_effects_apply(Act) :-
  % NOTE: `ignore` is not nice here, but some effects may only be applied under certain conditions in the rule body.
  forall( comp_actionEffectRule(Act, Descr),
          ignore( action_effect_apply(Act, Descr) )).

%% action_effect_apply(+Act:iri,+Effect:iri)
%
% Effect is a RDF SWRL rule that expresses an effect of Act.
% The implications of the rule (i.e., the rule head) is
% projected to the RDF triple store with this call.
%
% SWRL action effect rules have additional annotations used to
% reason about the context in which the rule applies (i.e., the action class).
% These rules usually start with a statement `Action(?act)` which assigns an action instance
% to the rule. The action instance Act provided will be bound to this action variable in the rule.
%
% @param Act Instance of knowrob:'Action'
% @param Effect RDF description of SWRL action effect rule
%
action_effect_apply(Act,Effect) :-
  rdf(Act, action_effects:actionEffectProjected, Effect, action_projection), !.
action_effect_apply(Act,Effect) :-
  rdf_has_prolog(Effect, knowrob:swrlActionVariable, Var),
  rdf_swrl_project(Effect, [var(Var,Act)]),
  % start/stop processes
  forall(rdf_has(Act, knowrob:processStarted, Started),
         action_effect_start_process(Started)),
  forall(rdf_has(Act, knowrob:processStopped, Stopped),
         action_effect_stop_process(Stopped)),
  % remember that this effect was projected before to avoid
  % that it is projected again.
  rdf_assert(Act, action_effects:actionEffectProjected, Effect, action_projection), !.

action_effect_start_process(Proc) :-
  rdf_has(Proc, knowrob:startTime, _), !.
action_effect_start_process(Proc) :-
  current_time(Now),
  owl_instance_from_class(knowrob:'TimePoint', [instant=Now], Timepoint),
  rdf_assert(Proc, knowrob:startTime, Timepoint).

action_effect_stop_process(Proc) :-
  rdf_has(Proc, knowrob:endTime, _), !.
action_effect_stop_process(Proc) :-
  current_time(Now),
  owl_instance_from_class(knowrob:'TimePoint', [instant=Now], Timepoint),
  rdf_assert(Proc, knowrob:endTime, Timepoint).

%% action_precondition_check(+Act:iri).
%% action_precondition_check(+Act:iri,+Effect:iri).
%
% True if Act is an action without unsatisfied preconditions, or
% if Effect is a satisfied precondition of Act.
%
% @param Act Instance of knowrob:'Action'
% @param Effect RDF description of SWRL action effect rule
%
action_precondition_check(Act) :-
  action_precondition_check(Act, _), !.

action_precondition_check(Act, Effect) :-
  rdf_has_prolog(Effect, knowrob:swrlActionVariable, Var),
  rdf_swrl_satisfied(Effect, [var(Var,Act)]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % %  Utility predicates

action_effect_objectActedOn(ActionClass, ObjVar, Head :- Body) :-
  rdf_has(Effect, knowrob:swrlActionConcept, literal(type(_,ActionClass))),
  rdf_swrl_rule(Effect, Head :- Body),
  rdf_has(Effect, knowrob:swrlActionVariable, literal(type(_,ActVar))),
  member(property(ActVar, 'http://knowrob.org/kb/knowrob.owl#objectActedOn', ObjVar), Body).

swrl_type_of(_, O, O) :- !.
swrl_type_of(_, literal(type(_,X)), O) :- strip_literal_type(O,X),!.
swrl_type_of(_, O, literal(type(_,X))) :- strip_literal_type(O,X),!.
swrl_type_of(_Head :- Body, Var, Type) :-
  member(class(Type_rule, Var), Body),
  once(owl_subclass_of(Type, Type_rule)).
