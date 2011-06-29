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
     
     locOfObj/2,
     locOfObjT/2,
     locOfSimilarObjT/2,
     locOfMostSimilarObjT/2,
     locOfKMostSimilarObjT/3,
     locOfSimilarObjTGtWup/3,
     locOfSimilarObjTBecause/3,
     locOfMostSimilarObjTBecause/3,
     locOfKMostSimilarObjTBecause/4,
     locOfSimilarObjTGtWupBecause/4,

     searchFoodOrDrinkTypeAtLocation/2,
     searchFoodOrDrinkTypeAtLocationType/2,
     searchFoodOrDrinkTypeInStorageConstruct/2,
     searchFoodOrDrinkTypeAtServiceLocation/2
    ]).

%%  hasRooms(L|B,R), hasLevels(B,L), objLoc(O,L), objLocWrtMap(O,L,M)

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

  locOfObj(r, r),

  locOfObjT(r, r),
  
  locOfSimilarObjT(r, r),
  locOfMostSimilarObjT(r, r),
  locOfKMostSimilarObjT(r, -, r),
  locOfSimilarObjTGtWup(r, -, r),

  locOfSimilarObjTBecause(r, r, r),
  locOfMostSimilarObjTBecause(r, r, r),
  locOfKMostSimilarObjTBecause(r, -, r, r),
  locOfSimilarObjTGtWupBecause(r, -, r, r).



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

locOfObj(Obj, Loc):-
  owl_individual_of(Obj, knowrob:'SpatialThing-Localized'),
  rdf_triple(knowrob:orientation, Obj, Loc).

% computable wrappers for all predicates below obj -> objT? 

locOfObjT(ObjT, Loc):-
  owl_subclass_of(ObjT, knowrob:'SpatialThing-Localized'),
  owl_has(Obj, rdf:type, ObjT),
  rdf_triple(knowrob:orientation, Obj, Loc).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% locations of objects with similar type
% object is known or unknown
% if unknown look at locations of similar object types (using WUP)


locOfSimilarObjT(ObjT, Loc):-
  locOfSimilarObjTBecause(ObjT, Loc, _).

locOfSimilarObjTBecause(ObjT, Loc, [S,T]):-
  similarObjTypes(ObjT,SimTypes),
  member([S,T], SimTypes),
  locOfObjT(T,Loc).

locOfMostSimilarObjT(ObjT, Loc):-
  locOfMostSimilarObjTBecause(ObjT, Loc, _).

locOfMostSimilarObjTBecause(ObjT, Loc, Because):-
  locOfKMostSimilarObjTBecause(ObjT, 1, Loc, Because).

locOfKMostSimilarObjT(ObjT, K, Loc):-
  locOfKMostSimilarObjTBecause(ObjT, K, Loc, _).
  
locOfKMostSimilarObjTBecause(ObjT, K, Loc, [S, T]):-
  similarObjTypes(ObjT,SimTypes),
  nth0(N, SimTypes, [S,T]),
  N < K,
  locOfObjT(T,Loc).

locOfSimilarObjTGtWup(ObjT, Wup, Loc):-
  locOfSimilarObjTGtWupBecause(ObjT, Wup, Loc, _).

locOfSimilarObjTGtWupBecause(ObjT, Wup, Loc, [S,T]):-
  similarObjTypes(ObjT,SimTypes),
  member([S,T], SimTypes),
  S >= Wup,
  locOfObjT(T,Loc).
  
similarObjTypes(ObjT, DecreasingSimTypes):- 
  % find all located instances (with orientation)
  % in the map and compare their types with ObjT
  % return sorted list of [Similarity, Type] tuples
  findall(T, (locOfObj(O,_L), owl_has(O, rdf:type, T)), TypesL),
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
locOfCreationEvtOfObjT(ObjT, Loc):-
  locOfCreationEvtOfObjTBecause(ObjT, _, _, Loc).

locOfCreationEvtOfObjTBecause(ObjT, EvtT, LocT, Loc):-
  createdAtLocationTBecause(ObjT, LocT, EvtT),
  locOfObjT(LocT,Loc).

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
%   locOfObj(Loc,MAT),
%   owl_restriction_on(R, restriction(knowrob:servesCuisine, some_values_from(CuisineT))),
%   owl_restriction_on(ObjT, restriction(knowrob:commonFoodTypeOfCuisine, some_values_from(CuisineT))),
%   findall(Ri, owl_individual_of(Ri, R), Rs),
%   list_to_set(Rs,RsSet),
%   member(Res, RsSet).
  


locOfStoragePlaceForObjT(ObjT, Loc):-
  locOfStoragePlaceForObjTBecause(ObjT, _, _ ,Loc).

locOfStoragePlaceForObjTBecause(ObjT, SPForType, LocT, Loc):-
  owl_subclass_of(LocT,    knowrob:'StorageConstruct'),
  owl_restriction_on(LocT, restriction(knowrob:'typePrimaryFunction-StoragePlaceFor', some_values_from(SPForType))),
  owl_subclass_of(ObjT, SPForType),
  locOfObjT(LocT, Loc).



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
  

