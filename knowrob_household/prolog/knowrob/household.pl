/*
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

:- module(household,
    [
     searchFoodOrDrinkTypeAtLocation/2
     ]).
/** <module> Utility predicates for the household domain.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/computable')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
  searchFoodOrDrinkTypeAtLocation(r,r).

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
