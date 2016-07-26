/** <module> Prediction of action effects

  Copyright (C) 2011 Moritz Tenorth, 2016 Daniel Beßler
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

@author Moritz Tenorth, Daniel Beßler
@license BSD

*/
:- module(action_effects,
    [
      project_action_effects/1,
      action_preconditions_fulfilled/1,
      action_project_effects/1,
      action_project_effects/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('owl')).
:- use_module(library('fluents')).


:- rdf_db:rdf_register_ns(knowrob,       'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(action_effects,       'http://knowrob.org/kb/action-effects.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).

:-  rdf_meta
    action_preconditions_fulfilled(r),
    action_project_effects(r),
    action_project_effects(r,?),
    action_project_effect(r,r,?),
    query_entity(r,r,r,r),
    query_predicate(r,r,r,r),
    project_action_effects(r),
    unlink_object(r),
    remove_object_properties(r,r).


action_preconditions_fulfilled(Action) :-
  true. % TODO: implement


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Action effect projection based on OWL description of effects
%

%% action_project_effects(+Action)
%% action_project_effects(+Action, -Effects)
%
% Project the effects of @Action into the KB.
% This boils down to manipulation of the rdf triple store.
% Since we want to reason about past states we generally use
% fluent properties and never retract from the KB.
% The projection method uses the action-effect and fluent
% ontologies which are part of the KnowRob core KB.
% Preconditions are not checked.
%
% @Action The action of which effects should be projected in the KB
% @Effects Descriptions of projected effects
%
action_project_effects(Action) :- action_project_effects(Action, _).

action_project_effects(Action, Effects) :-
  once( owl_individual_of(Action, knowrob:'Action') ),
  % TODO(daniel): Support ordering constraints of effects?
  findall(EffectDescription, (
    rdf_has(Action, knowrob:'hasEffect', Effect),
    action_project_effect(Action, Effect, EffectDescription)
  ), Effects).

action_project_effect(Action, Effect, integrate(Subject,Predicate,Object)) :-
  once(owl_individual_of(Effect, action_effects:'Integrate')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'subject', Subject),
  query_predicate(Effect, Predicate),
  query_entity(Action, Effect, action_effects:'object', Object),
  % Assert temporal parts and link via @Predicate
  assert_fluent_begin(Subject, Predicate, Object), !.

action_project_effect(Action, Effect, decompose(Subject,Predicate)) :-
  once(owl_individual_of(Effect, action_effects:'Decompose')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'subject', Subject),
  query_predicate(Effect, Predicate),
  % Read domain type and create new instance
  rdf_has(Predicate, rdfs:range, DomainClass),
  rdf_instance_from_class(DomainClass, DomainEntity),
  % Assert temporal parts and link via @Predicate
  assert_fluent_begin(Subject, Predicate, DomainEntity), !.

action_project_effect(Action, Effect, parametrize(Subject,Predicate,Object)) :-
  once(owl_individual_of(Effect, action_effects:'Parametrize')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'subject', Subject),
  query_predicate(Effect, Predicate),
  % Read the data value
  rdf_has(Effect, action_effects:'object', literal(type(_,Object))),
  % Assert temporal parts and link via @Predicate
  assert_fluent_begin(Subject, Predicate, Object), !.
  
action_project_effect(Action, Effect, create(Object)) :-
  once(owl_individual_of(Effect, action_effects:'Create')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'object', ObjectClass),
  rdf_instance_from_class(ObjectClass, Object), !.

action_project_effect(Action, Effect, clear(Subject,Predicate)) :-
  once(owl_individual_of(Effect, action_effects:'ClearProperty')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'subject', Subject),
  query_predicate(Effect, Predicate),
  % Assert temporal parts and link via @Predicate
  assert_fluent_end(Subject, Predicate), !.

action_project_effect(Action, Effect, separate(Subject,Predicate)) :-
  once(owl_individual_of(Effect, action_effects:'Separate')),
  % parse arguments
  query_entity(Action, Effect, action_effects:'subject', Subject),
  query_predicate(Effect, Predicate),
  query_entity(Action, Effect, action_effects:'object', Domain),
  % Assert temporal parts and link via @Predicate
  assert_fluent_end(Subject, Predicate, Domain), !.

action_project_effect(_, Effect, noop) :-
  write('Unable to project action effect individual "'), write(Effect), writeln('".'), !.


query_predicate(Effect, Predicate) :-
  rdf_has(Effect, action_effects:'predicate', literal(type(_,Predicate))).

query_entity(Action, Effect, Predicate, Entity) :-
  rdf_has(Effect, Predicate, literal(type(_,EntityQuery))),
  atomic_list_concat(EntityQueryList, ' ', EntityQuery),
  query_entity0(Action, null, EntityQueryList, Entity).

query_entity0(_, Entity, [], Entity).

query_entity0(Action, null, ['?action'|Rest], Entity) :-
  query_entity0(Action, Action, Rest, Entity), !.

query_entity0(Action, null, [Head|Rest], Entity) :-
  query_entity0(Action, Head, Rest, Entity), !.

query_entity0(Action, Parent, [Prop|Rest], Entity) :-
  % FIXME(daniel): What if cardinality > 1 ?
  % TODO(daniel): support fluents here?
  once((rdf_has(Parent, Prop, Child)
  -> query_entity0(Action, Child, Rest, Entity)
  ;  (
    write('Unable to project action "'), write(Action), writeln('".')
  ))).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
%

% utility predicate: remove reference to an object (e.g. if it has been destroyed)
% @deprecated
%
unlink_object(Obj) :-
  ((findall(Prop, (rdfs_subproperty_of(Prop, knowrob:topologicalRelations)), Props),
    findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _),!) ; true,!),
  ((findall(Prop, (rdfs_subproperty_of(Prop, knowrob:orientation)), Props),
    findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _),!) ; true,!).

% utility predicate: remove all assertions of sub-properties of Property from Obj
% @deprecated
remove_object_properties(Obj, Property) :-
(findall(Prop, (rdfs_subproperty_of(Prop, Property)), Props),
   findall(P,    (member(P, Props), rdf_retractall(Obj, P, _)), _)),!.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Joining substances
%


% % % % % % % % % % % % % % % %
% Mixing baking mix to a dough
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Mixing'),
  \+ owl_has(Action, knowrob:outputsCreated, _),

  % at least one objectActedOn is a MixForBakedGoods or WheatFlour
  owl_has(Action, knowrob:objectActedOn, Mix),
  (owl_individual_of(Mix, knowrob:'MixForBakedGoods');
   owl_individual_of(Mix, knowrob:'WheatFlour') ;
   owl_individual_of(Mix, knowrob:'Dough') ),

  findall(Obj, owl_has(Action, knowrob:objectActedOn, Obj), Objs), !,

  % new objects: Dough is created, things are added to Dough
  rdf_instance_from_class(knowrob:'Dough', knowrob_projection, Dough),
  rdf_assert(Action, knowrob:objectAddedTo,  Dough, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Dough, knowrob_projection),

  % new relations
  findall(O, (member(O, Objs),
              rdf_assert(Action, knowrob:thingIncorporated, O, knowrob_projection),
              print(O),print(' added to -> '), print(Dough), print('\n') ), _).



% % % % % % % % % % % % % % % %
% Adding something to something else
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Incorporation-Physical'),
  \+ owl_individual_of(Action, knowrob:'Mixing'),
  \+ owl_has(Action, knowrob:'', _).  % TODO: implement!





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Heating up and cooking
%


% % % % % % % % % % % % % % % %
% Heat up food
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'HeatingFood'),
  \+ owl_has(Action, knowrob:'', _).  % TODO: implement!

% temperature: 100 degree?



% % % % % % % % % % % % % % % %
% Boil food
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'BoilingFood'),
  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_individual_of(Obj, knowrob:'Boiled'),!,

  rdf_assert(Obj, rdf:type, knowrob:'Boiled', knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Obj, knowrob_projection),

  rdf_assert(Obj, knowrob:temperatureOfObject, literal(type('http://www.w3.org/2001/XMLSchema#integer', '100')), knowrob_projection),

  print(Obj),print(' boiled '), print('\n').




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Separation actions (cutting, cracking, chopping)
%


% % % % % % % % % % % % % % % %
% Chopping something into small pieces
% project_action_effects(Action) :-
%
%   owl_individual_of(Action, knowrob:'ChoppingSomething'),
%   \+ owl_has(Action, knowrob:'', _).  % TODO: implement!


% % % % % % % % % % % % % % % %
% Taking something out of a container (sugar, coffee grains)
% project_action_effects(Action) :-
%
%   owl_individual_of(Action, knowrob:'RemovingMaterialFromAContainer'),
%   \+ owl_has(Action, knowrob:'', _).  % TODO: implement!



% % % % % % % % % % % % % % % %
% Cleaning something
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Cleaning'),
  \+ owl_has(Action, knowrob:'objectOfStateChange', _),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'Clean'),!,

  % new relations
  remove_object_properties(Obj, knowrob:'stateOfObject'),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'Clean', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'Clean', knowrob_projection),

  % optionally: fromState
  ((owl_has(Obj, knowrob:'stateOfObject', PreState),
    rdf_assert(Action, knowrob:'fromState', PreState, knowrob_projection)) ; (true)),

  print(Obj),print(' cleaned '), print('\n').



% % % % % % % % % % % % % % % %
% Cutting off a piece
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'CuttingOffAPiece'),
  \+ owl_has(Action, knowrob:outputsCreated, _),


  owl_has(Action, knowrob:objectActedOn, Obj),
  rdf_has(Obj, rdf:type, ObjType),
  ObjType \= 'http://www.w3.org/2002/07/owl#NamedIndividual',!,

  % new objects
  rdf_instance_from_class(ObjType, knowrob_projection, Slice),

  % new relations
  rdf_assert(Action, knowrob:outputsRemaining, Obj, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Slice, knowrob_projection),

  print(Obj),print(' -> '), print(Slice), print('\n').



% % % % % % % % % % % % % % % %
% Cracking an egg
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'Cracking'),
  \+ owl_has(Action, knowrob:outputsCreated, _),

  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_individual_of(Obj, knowrob:'Egg-Chickens'),!,

  % new objects
  rdf_instance_from_class(knowrob:'EggShell', knowrob_projection, Shell),
  rdf_instance_from_class(knowrob:'EggYolk-Food', knowrob_projection, Yolk),

  % new relations
  rdf_assert(Action, knowrob:inputsDestroyed, Obj, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Shell, knowrob_projection),
  rdf_assert(Action, knowrob:outputsCreated, Yolk, knowrob_projection),

  print(Obj),print(' -> '), print(Shell), print('\n'),
  print(Obj),print(' -> '), print(Yolk), print('\n').






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Moving something to a location
%



% % % % % % % % % % % % % % % %
% Applying something to a surface (e.g. spreading butter)
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'ApplyingSomethingToSurface'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').



% % % % % % % % % % % % % % % %
% Putting something onto something
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingOnto'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').


% % % % % % % % % % % % % % % %
% Putting something into a container
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingIntoSomething'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'in-ContGeneric', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').




% % % % % % % % % % % % % % % %
% Putting something to some location
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PuttingSomethingSomewhere'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  owl_has(Action, knowrob:toLocation, To),!,

  % predict the object to be at the toLocation of the action
  rdf_instance_from_class(knowrob:'ThoughtExperimenting', knowrob_projection, Pred),
  rdf_assert(Pred, knowrob:'objectActedOn', Obj, knowrob_projection),

  get_timepoint(NOW), % TODO: add predicted action duration
  rdf_assert(Pred, knowrob:'startTime', NOW, knowrob_projection),

  ((owl_individual_of(To, knowrob:'RotationMatrix'),!) -> (
    % if toLocation is given as pose matrix, use this one
    rdf_assert(Pred, knowrob:'eventOccursAt', To, knowrob_projection)
   ) ; (
    % otherwise use the knowrob:orientation of the toLocation
    rdf_triple(knowrob:orientation, To, ToPose),
    rdf_assert(Pred, knowrob:'eventOccursAt', ToPose, knowrob_projection)
  )),

  print(Obj),print(' at location '), print(To), print('\n').



% % % % % % % % % % % % % % % %
% Pouring something onto something
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'PouringSomethingOnto'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'on-Physical', _),

  owl_has(Action, knowrob:toLocation, To),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all topological relations between OBJ and something else
  remove_object_properties(Obj, knowrob:topologicalRelations),
  rdf_assert(Obj, knowrob:'on-Physical', To, knowrob_projection),

  print(Obj),print(' on top of '), print(To), print('\n').




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Changing device states (turning sth on or off, opening or closing sth)
%

% % % % % % % % % % % % % % % %
% Switching on a device
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOnPoweredDevice'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:'stateOfObject'),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOff', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOn', knowrob_projection),

  print(Obj),print(' switched on '), print('\n').



% % % % % % % % % % % % % % % %
% Switching off a device
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOffPoweredDevice'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOn', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOff', knowrob_projection),

  print(Obj),print(' switched off'), print('\n').



% % % % % % % % % % % % % % % %
% Turning on water (objectActedOn is a tap)
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOnWater'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOn', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOff', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOn', knowrob_projection),

  print(Obj),print(' switched on'), print('\n').



% % % % % % % % % % % % % % % %
% Turning off water (objectActedOn is a tap)
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'TurningOffWater'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'DeviceStateOff', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'DeviceStateOn', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'DeviceStateOff', knowrob_projection),

  print(Obj),print(' switched off'), print('\n').



% % % % % % % % % % % % % % % %
% Opening a container (change state to open)
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'OpeningAContainerArtifact'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateOpen'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateOpen', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'ObjectStateClosed', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'ObjectStateOpen', knowrob_projection),

  print(Obj),print(' opened'), print('\n').



% % % % % % % % % % % % % % % %
% Closing a container (change state to closed)
% @deprecated
project_action_effects(Action) :-

  owl_individual_of(Action, knowrob:'ClosingAContainerArtifact'),

  owl_has(Action, knowrob:objectActedOn, Obj),
  \+ owl_has(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateClosed'),!,

  % new relations
  % TODO: qualify these relations to hold only for limited time
  % Hack: retract all asserted states of OBJ
  remove_object_properties(Obj, knowrob:stateOfObject),
  rdf_assert(Obj, knowrob:'stateOfObject', knowrob:'ObjectStateClosed', knowrob_projection),

  rdf_assert(Action, knowrob:'objectOfStateChange', Obj, knowrob_projection),
  rdf_assert(Action, knowrob:'fromState', knowrob:'ObjectStateOpen', knowrob_projection),
  rdf_assert(Action, knowrob:'toState',   knowrob:'ObjectStateClosed', knowrob_projection),

  print(Obj),print(' opened'), print('\n').



