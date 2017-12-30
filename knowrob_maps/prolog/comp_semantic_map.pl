/*
Copyright (c) 2011, Lars Kunze <kunzel@cs.tum.edu>
Copyright (c) 2017, Daniel Beßler
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Intelligent Autonomous Systems Group/
      Technische Universitaet Muenchen nor the names of its contributors 
      may be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

:- module(comp_semantic_map,
    [
     building_number_of_levels/2,  % number of floors
     building_number_of_stories/2, % number of levels above ground
     building_has_rooms/2,         % true if a room is part of the building
     building_path_cost/3,         % cost estimate for navigation between two different rooms/levels
     building_best_path_cost/3,
     map_object_in_level/2,        % true if object is inside of level bounding box
     map_object_in_level/3,
     map_object_in_room/2,         % true if object is inside of room bounding box
     map_object_in_room/3,
     map_object_most_similar/2,    % find (most) similar object
     map_object_similar/2,
     map_object_similar/3
     ]).
/** <module> Computables related to a semantic map.

@author Lars Kunze
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/computable')).

:- owl_parser:owl_parse('package://knowrob_maps/owl/comp_semantic_map.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_sem_map, 'http://knowrob.org/kb/comp_semantic_map.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
  building_number_of_levels(r,-),
  building_number_of_stories(r,-),
  building_has_rooms(r,-),
  building_path_cost(r,r,-),
  building_best_path_cost(r,t,-),
  map_object_in_room(r, r),
  map_object_in_level(r, r),
  map_object_most_similar(r,r),
  map_object_similar(r,r),
  map_object_similar(r,r,+).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Spatial layout of maps

%% building_number_of_levels(?Map:iri, ?N:int) is semidet
%
% True if N is the number of levels in Map.
%
% @param Map The semantic map 
% @param N The number of levels
%
building_number_of_levels(Map, N):-
  rdfs_individual_of(Map, rdf:type, knowrob:'Building'),
  setof(L, rdf_triple(knowrob:'hasLevels', Map, L), Ls),
  length(Ls, N).

%% building_number_of_stories(?Map:iri, ?N:int) is semidet
%
% True if N is the number of stories in Map.
%
% @param Map The semantic map 
% @param N The number of stories
%
building_number_of_stories(Map, N) :-
  rdfs_individual_of(Map, rdf:type, knowrob:'Building'),
  setof(L, (rdf_triple(knowrob:'hasLevels', Map, L),
            owl_has(L, rdf:type, knowrob:'AboveGroundLevelInAConstruction')), Ls),
  length(Ls, N).

%% building_has_rooms(+Map:iri, ?Room:iri) is semidet
%
% True if Room is a Room of some level in Map.
%
% @param Map The semantic map 
% @param Room The room in Map
%
building_has_rooms(Map, Room) :-
  rdfs_individual_of(Map, rdf:type, knowrob:'Building'),
  once((
    rdf_triple(knowrob:'hasLevels', Map, Level),
    rdf_triple(knowrob:'hasRooms', Level, Room))).

building_floor_number(Floor, Number):-
  rdfs_individual_of(Floor, knowrob:'LevelOfAConstruction'),
  rdf_triple(knowrob:floorNumber, Floor, NA),
  strip_literal_type(NA, AtomA),
  atom_number(AtomA, Number).

%% building_path_cost(+From:iri, +To:iri, -Cost:float) is semidet
%
% Cost is the estimatated cost for navigating from From to To.
%
% @param From Some semantic map entity
% @param To Some semantic map entity
% @param Cost The cost for navigation
%
building_path_cost(From, To, Cost):-
  map_object_same_level(From, To),!,
  intra_level_cost(From, To, Cost).

building_path_cost(From, To, Cost):-
  setof(E, owl_individual_of(E,knowrob:'Elevator'), Es),
  member(E1, Es), map_objects_same_level(From,E1),
  member(E2, Es), map_objects_same_level(To,E2),
  building_path_cost(From,E1,C1),
  building_path_cost(To,E2,C2),
  map_object_in_level(From,FromLevel),
  map_object_in_level(To,ToLevel),
  inter_level_cost(FromLevel, ToLevel, C3),
  Cost is C1 + C2 + C3.
 
%% building_best_path_cost(+From:iri, +GoalList:list, -Best:iri) is semidet
%
% Best is the map object out of GoalList with lowest navigation cost starting from
% the map object From.
%
% @param From Some semantic map entity
% @param GoalList List of semantic map entities
% @param Best Semantic map entity with lowest navigation cost
%
building_best_path_cost(From, GoalList, Best):-
  findall([Cost,G], (
    member(G, GoalList),
    building_path_cost(From, G, Cost)
  ), PCs),
  sort(PCs,SortedGoals),
  member([_,Best],SortedGoals), !.
  
intra_level_cost(A,B,D) :- object_distance(A,B,D).
inter_level_cost(A,B,Cost) :-
  building_floor_number(A, A_num),
  building_floor_number(B, B_num),
  LevelDistance is abs(A_num - B_num),
  object_dimensions(A, _, _, LevelHeight),
  LevelCost is LevelDistance * LevelHeight,
  % + waiting time for calling, intermediate stops, etc
  Cost is LevelCost.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Searching for objects in semantic maps

%% map_object_in_room(?Object:iri, ?Room:iri) is nondet
%
% True if Object is inside of Room.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param Room Instance of knowrob:RoomInAConstruction
%
map_object_in_room(Object, Room):-
  rdfs_individual_of(Room, knowrob:'RoomInAConstruction'),
  rdfs_individual_of(Object, knowrob:'HumanScaleObject'),
  holds(Object, knowrob:'in-ContGeneric', Room).

map_object_in_room(Object, Room, Interval):-
  rdfs_individual_of(Room, knowrob:'RoomInAConstruction'),
  rdfs_individual_of(Object, knowrob:'HumanScaleObject'),
  holds(Object, knowrob:'in-ContGeneric', Room, Interval).

%% map_object_in_level(?Object:iri, ?Level:iri) is nondet
%
% True if Object is inside of Level.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param Level Instance of knowrob:LevelOfAConstruction
%
map_object_in_level(Place, Level):-
  rdfs_individual_of(Level, knowrob:'LevelOfAConstruction'),
  rdfs_individual_of(Place, knowrob:'SpatialThing-Localized'),
  holds(Object, knowrob:'in-ContGeneric', Level).

map_object_in_level(Place, Level):-
  rdfs_individual_of(Level, knowrob:'LevelOfAConstruction'),
  rdfs_individual_of(Place, knowrob:'SpatialThing-Localized'),
  holds(Object, knowrob:'in-ContGeneric', Level, Interval).

map_objects_same_room(A, B)  :- map_object_in_room(A,R),  map_object_in_room(B,R).
map_objects_same_level(A, B) :- map_object_in_level(A,L), map_object_in_level(B,L).

%% map_object_most_similar(+Object:iri, -MostSimilarObject:iri) is semidet
%
% True if MostSimilarObject is the most similar semantic map object
% compared to Object.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param MostSimilarObject Instance of knowrob:HumanScaleObject
%
map_object_most_similar(Object, MostSimilarObject) :-
  map_object_similar(Object, [_,SimilarObject], 0), !.

%% map_object_similar(+Object:iri, -SimilarObject:iri) is semidet
%
% True if SimilarObject is a similar to Object.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param SimilarObject Instance of knowrob:HumanScaleObject
%
map_object_similar(Object, SimilarObject) :-
  map_object_similar(Object, [_,SimilarObject], 1).

%% map_object_similar(+Object:iri, -Similar:term, +Threshold:float) is semidet
%
% True if Similar is a list [Similarity,SimilarObject] and Similarity is
% the similarity estimate between Object and SimilarObject.
% Similarity must be higher then Threshold.
% This predicate will yield the most similar objects first.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param SimilarObject Instance of knowrob:HumanScaleObject
%
map_object_similar(Object, [Similarity,SimilarObject], Threshold) :-
  map_root_object(Map, Object),
  map_object_type(Object, Type), !,
  % for each similar type
  map_object_wup_similarity(Type, Threshold, SimilarTypes),
  member([Similarity,SimilarType], SimilarTypes),
  rdfs_individual_of(SimilarObject, SimilarType),
  map_root_object(Map, SimilarObject).

%% map_object_wup_similarity
map_object_wup_similarity(ObjT, Threshold, DecreasingSimTypes) :-
  % find existing types
  findall(T, (
    map_root_object(_, O),
    map_object_type(O, T)), TypesL),
  list_to_set(TypesL, TypesS),
  % compute similarity
  findall([Sim,T], (
    member(T,TypesS),
    rdf_wup_similarity(ObjT, T, Sim),
    Sim > Threshold
  ), SimTypes),
  sort(SimTypes, IncreasingSimTypes),
  reverse(IncreasingSimTypes, DecreasingSimTypes).
