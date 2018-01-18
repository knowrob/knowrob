/*
  Copyright (C) 2011 Martin Schuster
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
:- module(comp_orgprinciples,
    [
		best_location_maxMaxWup/2,
		best_location_maxMaxWup/3,
		best_location_dtree/2,
		avg_similarity_object_location/4,
		max_similarity_object_location/4
    ]).
/** <module> Reasoning about organizational principles in household environments

  This module contains all computables that compute the location where
  an object should be placed given other objects at different locations
  in the environment. Can also be used to infer where an object could
  probably be found, given earlier (outdated) observations of other
  objects in the environment. Uses the WUP similartiy from
  comp_similartiy module.

@author Martin Schuster
@license BSD
*/
:- use_module(library('semweb/rdf_db')).
:-  rdf_meta
          best_location_maxMaxWup(r,-),
          best_location_maxMaxWup(r,-,-),
	  best_location_dtree(r,-),
	  objects_at_location(r,-),
	  class_of_object(r,r),
	  classes_of_objects(t,t).


%% best_location_maxMaxWup(+Object, -BestLocation).
%% best_location_maxMaxWup(+Object, -BestLocation, -MaxMaxSim).
%
% Computes the best location where to place a certain object
% may give multiple possible solutions in case there is a tie
%
% Example: best_location_maxMaxWup(orgprinciples_demo:'Muesli_Schoko_1', L).
% 
% Example: best_location_maxMaxWup(knowrob:'Knife', L).
%
% @param Object Object or Class
% @param BestLocation according to max. maxWup similarity
% 
best_location_maxMaxWup(Object, BestLocation)	:-
  best_location_maxMaxWup(Object, BestLocation, _).

best_location_maxMaxWup(Object, BestLocation, MaxMaxSim)   :-
    to_global(Object, ObjectGlobal),
    (class_of_object(Class1, ObjectGlobal) -> Class = Class1 ; Class = ObjectGlobal),
    all_locations(Locations),
    findall(MaxSim, (
    	member(Location, Locations),
    	objects_at_location(Location, ObjectsAtLocation),
    	classes_of_objects(ClassesAtLocation, ObjectsAtLocation),
    	max_similarity_object_location(comp_similarity:rdf_wup_similarity, Class, ClassesAtLocation, MaxSim)%,
    	%format('maxWup ~w at location ~w',[MaxSim, Location]), nl %debug output
    ), MaxSimList),
    max_list(MaxSimList, MaxMaxSim),
    nth0(Index, MaxSimList, MaxMaxSim),
    nth0(Index, Locations, BestLocation).


%% best_location_dtree(+Object, -BestLocation)
%
% Computes the best location where to place a certain object
%
% Example: best_location_dtree(orgprinciples_demo:'Muesli_Schoko_1', L).
% 
% Example: best_location_dtree(knowrob:'Knife', L).
%
% @param Object Object or Class
% @param BestLocation according to Decision Trees, features are maxWup and avgWup similaritites. 
% Uses the WEKA C4.5 Decision Tree (J48) classifier (unpruned, min. number of instances per leaf = 0)
% 
best_location_dtree(Object, BestLocation) :-
    to_global(Object, ObjectGlobal),
    (class_of_object(Class1, ObjectGlobal) -> Class = Class1 ; Class = ObjectGlobal),
    all_locations(Locations),
    objects_at_location(_, AllObjects),
    %prepare training data:
    %compute similarties of every object with every location
    findall(Trainingsdatum, (
    	member(TrainingsObject, AllObjects),
    	class_of_object(TrainingsClass, TrainingsObject),
    	findall(Similarities, (
    		member(Location, Locations),
    		objects_at_location(Location, ObjectsAtLocation),
    		% the following exclusion does not perform well in case there is only one object at a location
	    		%exclude classes of TrainingObject from location:
	    		subtract(ObjectsAtLocation, [TrainingsObject], ObjectsAtLocationWithoutT),
	    		classes_of_objects(ClassesAtLocation, ObjectsAtLocationWithoutT),
    		%classes_of_objects(ClassesAtLocation, ObjectsAtLocation),
    		max_similarity_object_location(comp_similarity:rdf_wup_similarity, TrainingsClass, ClassesAtLocation, MaxSim),
    		avg_similarity_object_location(comp_similarity:rdf_wup_similarity, TrainingsClass, ClassesAtLocation, AvgSim),
    		%format('maxWup ~w, avgWup ~w at location ~w',[MaxSim, AvgSim, Location]), nl, %debug output
    		Similarities = [MaxSim, AvgSim]
    	),SimilaritiesAllLocations),
    	rdf_triple(knowrob:'in-ContGeneric', TrainingsObject, TrainingsObjectLocation),
    	flatten(SimilaritiesAllLocations, SimilaritiesAllLocationsFlat),
    	Trainingsdatum = [TrainingsObjectLocation | SimilaritiesAllLocationsFlat]
    ), Trainingsdata),
    %write(Trainingsdata),nl, %debug output
    %train classifier: (options: -U: unpruned, -M 0: min. number of instances per leaf = 0)
    classifier_trained(Classifier, 'weka.classifiers.trees.J48', '-U -M 0', Trainingsdata),

    %prepare test data:
    %compute similarities of test object with every location
	findall(TestSimilarities, (
		member(TestLocation, Locations),
		objects_at_location(TestLocation, ObjectsAtTestLocation),
		classes_of_objects(ClassesAtTestLocation, ObjectsAtTestLocation),
		max_similarity_object_location(comp_similarity:rdf_wup_similarity, Class, ClassesAtTestLocation, TestMaxSim),
		avg_similarity_object_location(comp_similarity:rdf_wup_similarity, Class, ClassesAtTestLocation, TestAvgSim),
		%format('Test: maxWup ~w, avgWup ~w at location ~w',[TestMaxSim, TestAvgSim, TestLocation]), nl, %debug output
		TestSimilarities = [TestMaxSim, TestAvgSim]
	),TestSimilaritiesAllLocations),
	flatten(TestSimilaritiesAllLocations, TestSimilaritiesAllLocationsFlat),
	%write(TestSimilaritiesAllLocationsFlat),nl, %debug output
    %classify Object:
    classify_instances(Classifier, [TestSimilaritiesAllLocationsFlat] ,BestLocationList),
    nth0(0, BestLocationList, BestLocation).


%% classes_of_objects(-Classes, +Objects)
%
% Get all classes, one for each object for a list of objects
% in case of multiple classes per object, return first (random?)
% only returns classes that are subclasses of knowrob:'SpatialThing'
%
% @param Classes List of classes
% @param Objects List of instances
% 
classes_of_objects(Classes, Objects) :-
    findall(Class, (
	    member(Object, Objects),
	    class_of_object(Class, Object)
    ), Classes).


%% class_of_object(-Class, +Object)
%
% Get all class for object (instance)
% in case of multiple classes, return first (random?)
% only returns classes that are subclasses of knowrob:'SpatialThing'
% 
class_of_object(Class, Object) :-
	owl_has(Object, rdf:type, Class),
	owl_subclass_of(Class, 'http://knowrob.org/kb/knowrob.owl#SpatialThing'), !.

%% all_locations(-Locations)
%
% get all locations defined in the environment through in-ContGeneric relations
% 
all_locations(Locations) :-
    findall(L, rdfs_individual_of(L, knowrob:'ContainerArtifact'), Ls),
    list_to_set(Ls, Locations).


%% objects_at_location(+Location, -Objects)
%
% get all objects at a location
% 
objects_at_location(Location, Objects) :-
    findall(O, (rdf_triple(knowrob:'in-ContGeneric', O, Location) ;
                rdf_triple(knowrob:'on-Physical', O, Location)), ObjectsD),
    list_to_set(ObjectsD, Objects).

%% avg_similarity_object_location(:SimFct:predicate, +Class:rdf_class, +List:list, -Average:float).
%
% uses SimFct to compute the similarity between a class and a list of classes (=location)
% averages the similarities of all classes in the list
%
% @param SimFct similarity function SimFct(+Class1, +Class2, -Similartiy)
% @param Class class to compute similiarities with location
% @param List list of classes that define the location
% @param Average similarity of Class with List
% 
avg_similarity_object_location(SimFct, Class, List, Average) :-
    (List = [] -> Average is 0 ; (
	similarities(SimFct, Class, List, Similarities),
	average(Similarities, Average))).

%% max_similarity_object_location(:SimFct:predicate, +Class:rdf_class, +List:list, -Average:float).
%
% uses SimFct to compute the similarity between a class and a list of classes (=location)
% takes the maximum similartiy of all classes in the list
%
% @param SimFct similarity function SimFct(+Class1, +Class2, -Similartiy)
% @param Class class to compute similiarities with location
% @param List list of classes that define the location
% @param Average similarity of Class with List
max_similarity_object_location(SimFct, Class, List, Max) :-
    (List = [] -> Max is 0 ; (
	similarities(SimFct, Class, List, Similarities),
	max_list(Similarities, Max))).


%% similarities(:SimFct:predicate, +Class:rdf_class, +List:list, -Similarities:list).
%
% compute similarity of an object to each element of a list of object, using SimFct/3
%
% @param SimFct similarity function SimFct(+Class1, +Class2, -Similartiy)
% @param Class class to compute similiarities with location
% @param List list of classes that define the location
% @param Similarities similarities of Class with each element of [H|T]
similarities(_, _,[], []).
similarities(SimFct, Class, [H|T], Similarities) :-
    call(SimFct, Class, H, Similarity),
    similarities(SimFct, Class, T, SimilaritiesTail),
    append([Similarity] , SimilaritiesTail, Similarities).




% utility functions:
average(List, Average) :-
	count_sum(List, Count, Sum),
	Average is Sum / Count.

count_sum([], 0, 0).
count_sum([H|T], Count, Sum) :-
	count_sum(T, C1, S1),
	Count is C1 + 1,
	Sum is S1 + H.

to_global(Class, Global) :-
    (rdf_global_id(Class,Long) -> Global = Long ; Global = Class).

to_local(Class, Local) :-
	(rdf_global_id(Short,Class) -> Local = Short ; Local = Class).
