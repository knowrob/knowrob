/*  robcog-flanagan.pl

    Author:        Daniel Beßler
    E-mail:        danielb@informatik.uni-bremen.de
    WWW:           http://www.ease.org
    Copyright (C): 2018, University of Bremen

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    As a special exception, if you link this library with other files,
    compiled with a Free Software compiler, to produce an executable, this
    library does not by itself cause the resulting executable to be covered
    by the GNU General Public License. This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

:- module(robcog_flanagan, [
     flanaganize_map/1,
     flanaganize_episode/2
]).
/** <module> Translating RobCoG episodes to EASE ontology.

This code is very specific to RobCoG logs.
This will need to change everytime this format changes.
The purpose is to translate to EASE ontology such that
tokenizer does not have to deal with RobCoG specific representations.
The functionality this module provides is:
   - it asserts a 'Physical agent' symbol and two 'Arm' symbols and specifies
     'has part' for agent and its parts (in the map RDF graph)
   - it asserts proper EASE ontology types for objects in the map (in the map RDF graph)
   - it asserts proper EASE ontology types for events in episodes (in the episode RDF graph)
   - it generates missing motion symbols for grasping (in the episode RDF graph)
   - it generates an episode lasting arm motion symbol (in the episode RDF graph)
   - it classifies supporting contact, currently using a bad heuristic
   - it tries to overcome a bug in robcog logging by adjusting the
      begin/end of some contact events

@author Daniel Beßler
@license LGPL
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/temporal')). % `interval/2`

:- rdf_db:rdf_register_ns(knowrob_u, 'http://knowrob.org/kb/knowrob_u.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:- rdf_db:rdf_register_ns(ease, 'http://www.ease.org/ont/ease.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(force_dynamics, 'http://www.ease.org/ont/force-dynamics.owl#', [keep(true)]).

:- rdf_meta classify_contact_event(r,r),
            classify_object(r,r).

% RobCoG does not log agent symbol
robcog_agent(Map,Agent) :-
  rdfs_individual_of(Agent,dul:'PhysicalAgent'),
  rdf_has(Agent,knowrob:describedInMap,Map),!.
robcog_agent(Map,Agent) :-
  flanaganize_agent(Map,Agent).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Classification of support contact

% all contacts starting at t=0 and stopping at t=max are support contacts
is_support_contact(_Episode,Evt) :- interval(Evt,[0.0,_]), !.
is_support_contact(Episode,Evt)  :- interval(Evt,[_,End]),
                                    interval(Episode,[_,End]), !.
is_support_contact(_Episode,Evt) :-
  rdf_has(Evt, knowrob_u:inContact, Obj1),
  rdf_has(Evt, knowrob_u:inContact, Obj2), Obj2\=Obj1,
  % hand is never 'supported' and never 'supports'
  \+ rdfs_individual_of(Obj1,knowrob:'Hand'),
  \+ rdfs_individual_of(Obj2,knowrob:'Hand'),
  interval(Evt,[Begin,End]),
  Duration is End - Begin,
  % FIXME: bad heuristic
  %%       - consider velocities
  %%       - consider forces
  %%       - find out who is supporting whom
  %%       - classify support using the parser instead?
  Duration>3.0. % all contacts with duration above 3s are support events

is_effector_contact(_Episode,Evt) :-
  rdf_has(Evt, knowrob_u:inContact, Obj),
  rdfs_individual_of(Obj,knowrob:'Hand'), !.

classify_contact_event(Evt,Type) :-
  rdf(Evt, rdf:type, _, G),
  rdf_retractall(Evt,rdf:type,force_dynamics:'Contact'),
  rdf_assert(Evt,rdf:type,Type,G).

classify_contact_events(Episode) :- 
  forall((rdf_has(Episode, knowrob:subAction, Evt),
          rdf(Evt, rdf:type, force_dynamics:'Contact')), (
    ( is_effector_contact(Episode,Evt) -> 
      classify_contact_event(Evt,force_dynamics:'EffectorContact') ;
    ( is_support_contact(Episode,Evt)  ->
      classify_contact_event(Evt,force_dynamics:'SupportingContact') ;
      true )))
  ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Flanaganize semantic map

flanaganize_agent(Map,Agent) :-
  once(rdf(Map,_,_,G)),
  rdf_instance_from_class(dul:'PhysicalAgent',G,Agent),
  forall((
    rdfs_individual_of(Hand,knowrob:'Hand'),
    \+ rdfs_individual_of(Hand, ease:'PhysicalEffector'),
    rdf_has(Hand, knowrob:describedInMap, Map)), (
    rdf_assert(Hand,rdf:type,ease:'Hand',G),
    rdf_assert(Agent,dul:hasPart,Hand,G),
    rdf_instance_from_class(ease:'Arm',G,Arm),
    rdf_assert(Agent,dul:hasPart,Arm,G),
    rdf_assert(Arm,dul:hasPart,Hand,G)
  )).

classify_object(Obj,Type) :-
  rdf(Obj, knowrob:describedInMap, _, G),
  rdf_assert(Obj,rdf:type,Type,G).

flanaganize_objects(Map) :-
  forall((
    rdf(Obj, knowrob:describedInMap, Map),
    rdf_has(Obj, knowrob:depthOfObject, _)),
    classify_object(Obj,dul:'PhysicalArtifact')).
    % TODO: classify more objects? many different object types in logs...
    %( is_furniture(Obj) -> classify_object(Obj,objects:'Furniture') ;
    %  classify_object(Obj,dul:'PhysicalArtifact') )).

%% flanaganize_map(+Map).
%
% Modify an RobCoG semantic map such that it is a proper ABOX
% of the flanagan model.
%
% @param Map The semantic map IRI.
%
flanaganize_map(Map) :-
  flanaganize_agent(Map,_),
  flanaganize_objects(Map).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% Flanaganize episode

% HACK In the logs grasping/ungrasping for picking/putting meets support contact.
%      It should overlap instead. Here we make it overlap...
fix_grasping_overlap(Episode) :-
  forall((
    rdf(Episode,knowrob:subAction,Support,G),
    rdfs_individual_of(Support,knowrob_u:'TouchingSituation'),
    rdf(Episode,knowrob:subAction,Grasp,G),
    rdfs_individual_of(Grasp,knowrob:'GraspingSomething'),
    interval(Support,[_,Instant]),
    interval(Grasp,[Instant,_])), (
    X is Instant + 0.5,
    owl_instance_from_class(knowrob:'TimePoint', [instant=X], TimePoint),
    rdf_retractall(Support,knowrob:endTime,_),
    rdf_assert(Support,knowrob:endTime,TimePoint)
  )),
  forall((
    rdf(Episode,knowrob:subAction,Support,G),
    rdfs_individual_of(Support,knowrob_u:'TouchingSituation'),
    rdf(Episode,knowrob:subAction,Grasp,G),
    rdfs_individual_of(Grasp,knowrob:'GraspingSomething'),
    interval(Support,[Instant,_]),
    interval(Grasp,[_,Instant])), (
    X is Instant + 0.5,
    owl_instance_from_class(knowrob:'TimePoint', [instant=X], TimePoint),
    rdf_retractall(Grasp,knowrob:endTime,_),
    rdf_assert(Grasp,knowrob:endTime,TimePoint)
  )).

%% flanaganize_episode(+Map,+Episode).
%
% Modify an RobCoG episode such that it is a proper ABOX
% of the flanagan model.
%
% @param Map The semantic map IRI.
% @param Episode Episode IRI.
%
flanaganize_episode(Map,Episode) :-
  fix_grasping_overlap(Episode),
  % flanaganize time intervals of events
  forall(rdf(Episode,knowrob:subAction,Evt,G),(
         rdf_assert(Episode,dul:hasConstituent,Evt,G),
         interval(Evt,Interval),
         assert_has_interval(G,Evt,Interval))),
  % map episode terms to flanagan ontology
  flanaganize_touching_situations(Episode),
  flanaganize_grasping_situations(Episode),
  flanaganize_states(Episode),
  % we assume arm motion during the whole episode
  interval(Episode,EpisodeInterval),
  robcog_agent(Map,Agent),
  forall(( rdf_has(Agent,dul:hasPart,Arm),
            rdfs_individual_of(Arm,ease:'Arm')),(
    assert_limb_motion(G,Arm,EpisodeInterval,ArmMotion),
    rdf_assert(Episode, knowrob:subAction, ArmMotion, G))),
  % finally we want to know which events are supporting contacts
  classify_contact_events(Episode).

flanaganize_touching_situations(Episode) :-
  forall((rdf_has(Episode, knowrob:subAction, Evt),
          rdf(Evt, rdf:type, knowrob_u:'TouchingSituation', G)), (
    % TouchingSituation --> Contact
    rdf_assert(Evt, rdf:type, force_dynamics:'Contact', G),
    % knowrob_u:inContact --> dul:hasParticipant
    forall( rdf_has(Evt, knowrob_u:inContact, Obj),
             rdf_assert(Evt, dul:hasParticipant, Obj, G))
  )).

flanaganize_grasping_situations(Episode) :-
  forall((rdf_has(Episode, knowrob:subAction, Evt),
           rdf(Evt, rdf:type, knowrob:'GraspingSomething', G)), (
    % TouchingSituation --> Contact
    rdf_assert(Evt, rdf:type, force_dynamics:'Contact', G),
    % knowrob_u:inContact --> dul:hasParticipant
    rdf_has(Evt, knowrob:performedBy, Hand),
    rdf_has(Evt, knowrob:objectActedOn, Obj),
    rdf_assert(Evt, dul:hasParticipant, Hand, G),
    rdf_assert(Evt, dul:hasParticipant, Obj, G),
    %% assert grasping/releasing motions
    interval(Evt,[EvtBegin,EvtEnd]),
    GraspBegin is EvtBegin - 0.5, % HACK assume half a second, make it overlap
    GraspEnd is EvtBegin + 0.1,
    ReleaseBegin is EvtEnd - 0.1,
    ReleaseEnd is EvtEnd + 0.5,
    assert_grasping_motion(G, Hand, [GraspBegin,GraspEnd], GraspMotion),
    assert_releasing_motion(G, Hand, [ReleaseBegin,ReleaseEnd], ReleaseMotion),
    rdf_assert(Episode, knowrob:subAction, GraspMotion, G),
    rdf_assert(Episode, knowrob:subAction, ReleaseMotion, G)
  )).

% state type mapping (from RobCoG representation to flanagan model)
flanaganize_state('http://knowrob.org/kb/knowrob_u.owl#FurnitureStateClosed',
                  'http://www.ease.org/ont/states.owl#ClosedState').
flanaganize_state('http://knowrob.org/kb/knowrob_u.owl#FurnitureStateHalfClosed',
                  'http://www.ease.org/ont/states.owl#HalfClosedState').
flanaganize_state('http://knowrob.org/kb/knowrob_u.owl#FurnitureStateOpened',
                  'http://www.ease.org/ont/states.owl#OpenedState').
flanaganize_state('http://knowrob.org/kb/knowrob_u.owl#FurnitureStateHalfOpened',
                  'http://www.ease.org/ont/states.owl#HalfOpenedState').

flanaganize_states(Episode) :- 
  forall((rdf(Episode, knowrob:subAction, Evt, G),
          rdf(Evt, rdf:type, X),
          flanaganize_state(X, StateType)), (
    rdf_assert(Evt, rdf:type, StateType, G),
    forall( rdf_has(Evt, knowrob:objectActedOn, Obj),
            rdf_assert(Evt, dul:hasParticipant, Obj, G))
  )).

