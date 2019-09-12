/*
  Copyright (C) 2009-13 Lars Kunze, Lorenz Moesenlechner, Moritz Tenorth
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

:- module(semmap,
    [
      map_instance/1,
      map_object_info/1,
      map_object_type/2,
      map_root_objects/2,
      map_root_object/2,
      map_object_dimensions/4,
      map_child_object/2,
      map_child_objects/2,
      map_object_label/2,
      map_object_most_similar/2,
      map_object_similar/2,
      map_object_similar/3
    ]).
/** <module> Predicates for reading elements of a semantic map

@author Lars Kunze
@author Lorenz Moesenlechner
@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob/wup_similarity')).

:-  rdf_meta
      map_instance(r),
      map_root_object(r,r),
      map_root_objects(r,?),
      map_child_object(r,r),
      map_child_objects(r,?),
      map_object_info(?),
      map_object_type(r,r),
      map_object_dimensions(r,?,?,?),
      map_object_label(r,?),
      map_object_most_similar(r,r),
      map_object_similar(r,r),
      map_object_similar(r,r,+).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Map root and child objects

%% map_root_objects(Map, Objs) is nondet.
%
% Read the 'root objects' in the map, i.e. those that are asserted as
% direct children of the map instance (e.g. cupboards, drawers)
%
% @param Map   Instance of a knowrob:SemanticEnvironmentMap
% @param Objs  List of instances of the root objects in the map
% 
map_root_objects(Map, Objs) :-
    setof( Obj, map_root_object(Map, Obj), Objs).

%% map_root_object(Map, Obj) is nondet.
%
% Read the 'root objects' in the map, i.e. those that are asserted as
% direct children of the map instance (e.g. cupboards, drawers)
% 
% @param Map  Instance of a knowrob:SemanticEnvironmentMap
% @param Objs Instance of a root object in the map
% 
map_root_object(Map, Obj) :-
    kb_triple(Obj, knowrob:describedInMap, Map).
    
%% map_child_objects(+Parent, -Children) is nondet.
%
% Read all object instances asserted as properPhysicalParts of Parent
% 
% @param Parent    Object instance
% @param Children  List of object instances asserted as physical part of Parent
% 
map_child_objects(Parent, Children) :-
    setof(Child, map_child_object(Parent, Child), Children).

%% map_child_object(?Parent, ?Child) is nondet.
%
% Read all object instances asserted as properPhysicalParts of Parent
% 
% @param Parent   Object instance
% @param Child    Object instance
% 
map_child_object(Parent, Child) :-
    rdf_reachable(Parent, dul:hasPart, Child),
    Parent \= Child.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read object properties

%% map_object_info(-Info) is nondet.
%
% Read information about all objects in the map.
% Returning a structured list for
% each object consisting of the instance ID, the class, the pose, and
% a list of the object dimensions [D, W, H].
% 
% @param Inst  Object instance
% 
map_object_info([Inst, Type, Pose, [D, W, H]]) :-
    map_object_type(Inst, Type),
    current_object_pose(Inst, Pose),
    map_object_dimensions(Inst, D, W, H).

%% map_object_type(?Obj, ?Type) is nondet.
%
% Read the type of an object instance
% 
% @param Obj   Object instance
% @param Type  Class of Obj
%  
map_object_type(Obj, Type) :-
    kb_type_of(Obj, Type).

%% map_object_label(?Obj, ?Label) is nondet.
%
% Reads the rdfs:label of an instance
% 
% @param Obj    Object instance
% @param Label  Value of the rdfs:label annotation
%  
map_object_label(Obj, Label) :-
    kb_triple(Obj, rdfs:label, Label).

%% map_object_dimensions(Obj, D, W, H)is nondet.
%
% Read the dimensions (depth, width, height) of an object.
% 
% @param Obj    Object instance
% @param D      Object depth (x dimension)
% @param W      Object Width (y dimension)
% @param H      Object height (z dimension)
%  
map_object_dimensions(Obj, D, W, H) :-
    object_dimensions(Obj, D, W, H).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read map instance and content

%% map_instance(Map) is nondet.
%
% Read instances of a SemanticEnvironmentMap
% 
% @param Map  Instance of a knowrob:SemanticEnvironmentMap
% 
map_instance(Map) :-
    kb_type_of(Map, knowrob:'SemanticEnvironmentMap').

%% map_object_most_similar(+Object:iri, -MostSimilarObject:iri) is semidet
%
% True if MostSimilarObject is the most similar semantic map object
% compared to Object.
%
% @param Object Instance of knowrob:HumanScaleObject
% @param MostSimilarObject Instance of knowrob:HumanScaleObject
%
map_object_most_similar(Object, MostSimilarObject) :-
  map_object_similar(Object, [_,MostSimilarObject], 0), !.

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
  map_object_wup_similarity(Map, Object, Type, Threshold, SimilarTypes),
  member([Similarity,SimilarType], SimilarTypes),
  rdfs_individual_of(SimilarObject, SimilarType),
  map_root_object(Map, SimilarObject).

%% map_object_wup_similarity
map_object_wup_similarity(Map, Object, ObjT, Threshold, DecreasingSimTypes) :-
  % find existing types
  findall(T, (
    map_root_object(Map, O),
    O \= Object,
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
