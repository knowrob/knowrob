/** <module> comp_semantic_map

This module contains computables related to a semantic map.

Copyright (c) 2011, Lars Kunze <kunzel@cs.tum.edu>
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

@author Lars Kunze
@license BSD
*/

:- module(comp_semantic_map,
    [
     comp_number_of_levels/2,
     comp_number_of_stories/2,
     comp_has_rooms/2,
     nth_level_of_building/3,
     
     lookForObj/2,
     lookForObjT/2,

     lookForSimilarObjT/2,
     lookForMostSimilarObjT/2,
     lookForKMostSimilarObjT/3,
     lookForSimilarObjTGtWup/3,
     lookForSimilarObjTBecause/3,
     lookForMostSimilarObjTBecause/3,
     lookForKMostSimilarObjTBecause/4,
     lookForSimilarObjTGtWupBecause/4,

     searchFoodOrDrinkTypeAtLocation/2,
     searchFoodOrDrinkTypeAtLocationType/2,
     searchFoodOrDrinkTypeInStorageConstruct/2,
     searchFoodOrDrinkTypeAtServiceLocation/2,


     object_info/2,
     get_url/2,
     get_room/2,
     get_level/2,
    
     euclidean_distance/3,
     in_room/2,
     same_room/2,
     in_level/2,
     same_level/2,
     get_sorted_path_costs/3,
     path_cost/3,
     intra_level_cost/3,
     inter_level_cost/3,
     level_distance/3,
    
     create_perception_instance/1,
     create_object_instance/2,
     set_object_perception/2,
     set_perception_pose/2,
     update_pose/2

    
     ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).

:- owl_parser:owl_parse('../owl/comp_semantic_map.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_sem_map, 'http://ias.cs.tum.edu/kb/comp_semantic_map.owl#', [keep(true)]).


% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
  comp_number_of_levels(r,-),
  comp_number_of_stories(r,-),
  nth_level_of_building(@,r,r),
  
  searchFoodOrDrinkTypeAtLocation(r,r),
  searchFoodOrDrinkTypeAtLocationType(r,r),
  searchFoodOrDrinkTypeInStorageConstruct(r,r),
  searchFoodOrDrinkTypeAtServiceLocation(r,r),

  lookForObj(r, r),
  lookForObjT(r, r),

  lookForSimilarObjT(r, r),
  lookForMostSimilarObjT(r, r),
  lookForKMostSimilarObjT(r, -, r),
  lookForSimilarObjTGtWup(r, -, r),

  lookForSimilarObjTBecause(r, r, r),
  lookForMostSimilarObjTBecause(r, r, r),
  lookForKMostSimilarObjTBecause(r, -, r, r),
  lookForSimilarObjTGtWupBecause(r, -, r, r),

  object_info(r,-),
  get_url(r, -),
  get_room(r, -),
  get_level(r, -),
  
  euclidean_distance(r,r,-),
  in_room(r, r),
  same_room(r, r),
  in_level(r, r),
  same_level(r, r),
  get_sorted_path_costs(r, -, -),
  path_cost(r,r,-),
  intra_level_cost(r,r,-),
  inter_level_cost(r,r,-),
  level_distance(r,r,-),

  create_object_instance(r,r),
  update_pose(r,-),
  create_perception_instance(-),
  set_object_perception(r,r),
  set_perception_pose(r,-),

  data_value(r,r,-).

% Example query:
% setof(_O, lookForObjT(knowrob:'Cup', _O), _Objs),
% get_sorted_path_costs('http://www.jsk.t.u-tokyo.ac.jp/jsk_map.owl#cup-3', _Objs, _Sorted),
% member([_Cost,_Obj], _Sorted),
% object_info(_Obj,_Info),
% append([_Obj,_Cost],_Info,Result).


comp_number_of_levels(B, N):-
  setof(L, (owl_has(B, rdf:type,knowrob:'Building'),
            rdf_triple(knowrob:'hasLevels',B, L)), Ls),
  length(Ls, N).

comp_number_of_stories(B, N) :-
  setof(L, (owl_has(B, rdf:type, knowrob:'Building'),
            rdf_triple(knowrob:'hasLevels',B, L),
            owl_has(L, rdf:type, knowrob:'AboveGroundLevelInAConstruction')), Ls),
  length(Ls, N).

comp_has_rooms(I, R) :-
  rdf_reachable(C, rdfs:subClassOf, knowrob:'ConstructionArtifact'),
  rdf_has(I, rdf:type, C),
  rdf_triple(knowrob:'hasLevels',I, L),
  rdf_triple(knowrob:'hasRooms',L, R).

nth_level_of_building(Number, Building, Level) :-
  owl_has(Building, knowrob:hasLevels, Level), 
  owl_has(Level, knowrob:floorNumber, literal(N)),
  atom_number(N, Number).

% todo add computable
%comp_in_room(_O, _R).
%comp_in_level(_O, _L).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lookForObj(Obj,Obj):-
  owl_individual_of(Obj, knowrob:'SpatialThing-Localized').
  %rdf_triple(knowrob:orientation, Obj, Loc).

% computable wrappers for all predicates below obj -> objT? 

lookForObjT(ObjT, Obj):-
  owl_subclass_of(ObjT, knowrob:'SpatialThing-Localized'),
  owl_has(Obj, rdf:type, ObjT).
  %rdf_triple(knowrob:orientation, Obj, Loc).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% locations of objects with similar type using WUP 

lookForSimilarObjT(ObjT, Loc):-
  lookForSimilarObjTBecause(ObjT, Loc, _).

lookForSimilarObjTBecause(ObjT, Loc, [S,T]):-
  similarObjTypes(ObjT,SimTypes),
  member([S,T], SimTypes),
  lookForObjT(T,Loc).

lookForMostSimilarObjT(ObjT, Loc):-
  lookForMostSimilarObjTBecause(ObjT, Loc, _).

lookForMostSimilarObjTBecause(ObjT, Loc, Because):-
  lookForKMostSimilarObjTBecause(ObjT, 1, Loc, Because).

lookForKMostSimilarObjT(ObjT, K, Loc):-
  lookForKMostSimilarObjTBecause(ObjT, K, Loc, _).
  
lookForKMostSimilarObjTBecause(ObjT, K, Loc, [S, T]):-
  similarObjTypes(ObjT,SimTypes),
  nth0(N, SimTypes, [S,T]),
  N < K,
  lookForObjT(T,Loc).

lookForSimilarObjTGtWup(ObjT, Wup, Loc):-
  lookForSimilarObjTGtWupBecause(ObjT, Wup, Loc, _).

lookForSimilarObjTGtWupBecause(ObjT, Wup, Loc, [S,T]):-
  similarObjTypes(ObjT,SimTypes),
  member([S,T], SimTypes),
  S >= Wup,
  lookForObjT(T,Loc).
  
similarObjTypes(ObjT, DecreasingSimTypes):- 
  % find all located instances (with orientation)
  % in the map and compare their types with ObjT
  % return sorted list of [Similarity, Type] tuples
  findall(T, (lookForObj(O,_L), owl_has(O, rdf:type, T)), TypesL),
  list_to_set(TypesL, TypesS),
  findall([Sim,T], (member(T,TypesS), rdf_wup_similarity(ObjT, T, Sim)), SimTypes),
  sort(SimTypes, IncreasingSimTypes),
  reverse(IncreasingSimTypes, DecreasingSimTypes).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% searchSpaceForObj(Obj, Loc):-
%   owl_individual_of(Obj, knowrob:'SpatialThing-Localized'),
%   owl_has(Obj, rdf:type, ObjT),
%   searchSpaceForObjT(ObjT,Loc).

% searchSpaceForObjT(ObjT, Loc):-
%   searchSpaceForObjTBecauseEvtT(ObjT, _, _, Loc).

% look at places where instances of this objT are created.
lookForCreationEvtOfObjT(ObjT, Loc):-
  lookForCreationEvtOfObjTBecause(ObjT, _, _, Loc).

lookForCreationEvtOfObjTBecause(ObjT, EvtT, LocT, Loc):-
  createdAtLocationTBecause(ObjT, LocT, EvtT),
  lookForObjT(LocT,Loc).

createdAtLocationTBecause(ObjT, LocT, EvtT) :-
  createdInEventT(ObjT, EvtT),
  findall(SubL, (owl_restriction_on(EvtT, restriction(knowrob:eventOccursAt, some_values_from(L))), owl_subclass_of(SubL,L) ), Locations),
  list_to_set(Locations, LocationsSet),
  member(LocT, LocationsSet).

createdInEventT(ObjT, EvtT) :-
  owl_subclass_of(ObjT, knowrob:'SpatialThing-Localized'),
  owl_subclass_of(EvtT, knowrob:'ActionOnObject'),
  owl_restriction_on(EvtT, restriction(knowrob:'outputsCreated', some_values_from(ObjT))).


% prepareFoodAt(Obj, LocT) :-
%   foodCreatedAtLocationType(Obj, LocT),
%   owl_subclass_of(R, knowrob:'Kitchen').
% all ingredients and tools available
% have  capability to prepare item

% not working as is
% servedAtLocBy(ObjT,MAT,Res):-
%                                 %  createdInRestaurant(Obj, Res, EvT):-
%   owl_subclass_of(ObjT, knowrob:'FoodOrDrink'),
%   createdAtLocationTBecause(ObjT, LocT, _EvT),
%                                 % look for food in a restaurant
%   owl_subclass_of(R, knowrob:'Restaurant-Organization'),
%   owl_restriction_on(R, restriction(knowrob:servesCuisineAtLocation, some_values_from(LocT))),
%   owl_individual_of(Loc,LoT),
%   lookForObj(Loc,MAT),
%   owl_restriction_on(R, restriction(knowrob:servesCuisine, some_values_from(CuisineT))),
%   owl_restriction_on(ObjT, restriction(knowrob:commonFoodTypeOfCuisine, some_values_from(CuisineT))),
%   findall(Ri, owl_individual_of(Ri, R), Rs),
%   list_to_set(Rs,RsSet),
%   member(Res, RsSet).
  


lookForStoragePlaceForObjT(ObjT, Loc):-
  lookForStoragePlaceForObjTBecause(ObjT, _, _ ,Loc).

lookForStoragePlaceForObjTBecause(ObjT, SPForType, LocT, Loc):-
  owl_subclass_of(LocT,    knowrob:'StorageConstruct'),
  owl_restriction_on(LocT, restriction(knowrob:'typePrimaryFunction-StoragePlaceFor', some_values_from(SPForType))),
  owl_subclass_of(ObjT, SPForType),
  lookForObjT(LocT, Loc).


searchFoodOrDrinkTypeAtLocation(FoodOrDrink, Location) :-
  searchFoodOrDrinkTypeAtLocationType(FoodOrDrink, LocationT),
  owl_individual_of(Location, LocationT).
  

searchFoodOrDrinkTypeAtLocationType(FoodOrDrinkT, LocationT) :-
  ( searchFoodOrDrinkTypeInStorageConstruct(FoodOrDrinkT, LocationT) ;
    searchFoodOrDrinkTypeAtServiceLocation(FoodOrDrinkT, LocationT)).

searchFoodOrDrinkTypeInStorageConstruct(FoodOrDrinkT, LocationT) :-
  owl_subclass_of(FoodOrDrinkT, knowrob:'FoodOrDrink'),
  owl_subclass_of(LocationT,    knowrob:'StorageConstruct'),
  owl_restriction_on(LocationT, restriction(knowrob:'typePrimaryFunction-StoragePlaceFor', some_values_from(ObjT))),
  owl_subclass_of(FoodOrDrinkT, ObjT).

searchFoodOrDrinkTypeAtServiceLocation(FoodOrDrinkT, LocationT) :-
  owl_subclass_of(FoodOrDrinkT, knowrob:'FoodOrDrink'),
  owl_restriction_on(FoodOrDrinkT, restriction(knowrob:'commonFoodTypeOfCuisine', some_values_from(CuisineT))),
  owl_subclass_of(FoodAndBeverageOrganizationT, knowrob:'FoodAndBeverageOrganization'),
  owl_restriction_on(FoodAndBeverageOrganizationT, restriction(knowrob:'servesCuisine', some_values_from(CuisineT))),
  owl_subclass_of(LocationT, knowrob:'MultiRoomUnit'),
  owl_restriction_on(FoodAndBeverageOrganizationT, restriction(knowrob:'servesCuisineAtLocation', some_values_from(LocationT))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helper predicates
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


object_info(Obj,[Url, Room, Level]):-
  get_url(Obj, Url),
  get_room(Obj, Room),
  get_level(Obj, Level).
  

get_url(Obj, Url):-
  ( owl_has(Obj, knowrob:'linkToImageFile', URL)
  -> strip_literal_type(URL,Url)
  ; Url = false
  ).

get_room(Obj, Room):-
  ( in_room(Obj, ROOM )
  -> Room = ROOM
  ; Room = false
  ).

get_level(Obj, Level):-
  ( in_level(Obj, LEVEL)
  -> Level = LEVEL
  ; Level = false
  ).

get_cost(Cur, Goal, Cost):-
  path_cost(Cur,Goal,Cost).
  
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Precidates for calculating distances between poses
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% this precidate should be in a package like knowrob_owl_addons
% somehow not working, but why?
data_value(DataProperty,Ind,Value):-
  rdf_triple(DataProperty, Ind, LiteralType),
  strip_literal_type(LiteralType,AtomVal),
  atom_to_term(AtomVal,Value,_).
  

% distance in 3d
euclidean_distance(A,B,D):-
  data_value(knowrob:m03, A, AX),
  data_value(knowrob:m13, A, AY),
  data_value(knowrob:m23, A, AZ),
  data_value(knowrob:m03, B, BX),
  data_value(knowrob:m13, B, BY),
  data_value(knowrob:m23, B, BZ),
  DX is AX - BX,
  DY is AY - BY,
  DZ is AZ - BZ,
  D is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)).

in_room(Place, Room):-
  setof(R, owl_individual_of(R, knowrob:'RoomInAConstruction'), Rs),
  member(Room, Rs),
  setof(P, owl_individual_of(P, knowrob:'HumanScaleObject'), Ps),
  member(Place,Ps),
  rdf_triple(knowrob:'in-ContGeneric',Place, Room).

same_room(A, B):-
  in_room(A,R),
  in_room(B,R).

in_level(Place, Level):-
  setof(L, owl_individual_of(L, knowrob:'LevelOfAConstruction'), Ls),
  member(Level, Ls),
  setof(P, owl_individual_of(P, knowrob:'SpatialThing-Localized'), Ps),
  member(Place,Ps),
  rdf_triple(knowrob:'in-ContGeneric',Place, Level).

same_level(A, B):-
  in_level(A,L),
  in_level(B,L).
 
get_sorted_path_costs(Current, GoalList, SortedGoals):-
  findall([Cost,G], ( member(G, GoalList), path_cost(Current,G,Cost)), PCs),
  sort(PCs,SortedGoals).

path_cost(Current,Goal,Cost):-
  same_level(Current,Goal),
  intra_level_cost(Current,Goal,Cost),!.

path_cost(Current,Goal,Cost):-
  \+same_level(Current,Goal),
  setof(E, owl_individual_of(E,knowrob:'Elevator'), Es),
  member(E1, Es),
  same_level(Current,E1),
  path_cost(Current,E1,C1),
  member(E2, Es),
  same_level(Goal,E2),
  path_cost(Goal,E2,C2),
  in_level(Current,CurrentLevel),
  in_level(Goal,GoalLevel),
  inter_level_cost(CurrentLevel, GoalLevel, C3),
  Cost is C1 + C2 + C3.
  
intra_level_cost(A,B,D):-
  % todo: replace by more accurate heuristic
  % call to path planner? only if available, otherwise heuristic
  rdf_triple(knowrob:orientation,A,RA),
  rdf_triple(knowrob:orientation,B,RB),
  euclidean_distance(RA,RB,D).

inter_level_cost(A,B,Cost):-
  level_distance(A,B,LevelDistance),
  data_value(knowrob:heightOfObject,A, LevelHeight),
  LevelCost is LevelDistance * LevelHeight,
  % + waiting time for calling, intermediate stops, etc
  Cost is LevelCost.
  
level_distance(A,B,D):-
  setof(L, owl_individual_of(L,knowrob:'LevelOfAConstruction'), Ls),
  member(A,Ls),
  member(B,Ls),
  rdf_triple(knowrob:floorNumber, A, NA),
  rdf_triple(knowrob:floorNumber, B, NB),
  strip_literal_type(NA,AtomA), atom_to_term(AtomA, TermA, _),
  strip_literal_type(NB,AtomB), atom_to_term(AtomB, TermB, _),
  D is abs(TermA -TermB).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Predicates to create new instances, set and update their poses.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% create_perception_instance(-Perception) is det.
%
% Create perception instance 
%
create_perception_instance(Perception) :-

  rdf_instance_from_class('http://ias.cs.tum.edu/kb/knowrob.owl#Perceiving', Perception),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).


%% create_object_instance(+Type, +ID) is det.
%
% Create object instance having type 
%
create_object_instance(Type, Obj) :-

  (rdf_has(Obj, rdf:type, Type), !) ;

  (rdf_assert(Obj, rdf:type, Type),
   rdf_assert(Obj, knowrob:'widthOfObject',  literal(type('http://www.w3.org/2001/XMLSchema#float','0.0'))),
   rdf_assert(Obj, knowrob:'heightOfObject', literal(type('http://www.w3.org/2001/XMLSchema#float','0.0'))),
   rdf_assert(Obj, knowrob:'depthOfObject',  literal(type('http://www.w3.org/2001/XMLSchema#float','0.0')))).
   

%% set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
set_object_perception(Object, Perception) :-
  rdf_assert(Perception, knowrob:objectActedOn, Object).


%% set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  % set the pose
  atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
  rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
  rdf_assert(Perception, knowrob:eventOccursAt, Loc).

%% set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
update_pose(Object, Pose):-

  % Object should already exist to update a pose,
  % i.e. have at least a type
  % it would also be possible to remove this check, but then,
  % if object does not exist it would be asserterd
  owl_has(Object, rdf:type, _Type),!,
 
  create_perception_instance(Perception),

  set_perception_pose(Perception, Pose),
  
  set_object_perception(Object, Perception).

    
