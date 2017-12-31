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

:- module(semantic_map_utils,
    [
      map_instance/1,
      map_name/1,
      map_read_all/2,
      map_object_info/1,
      map_object_type/2,
      map_root_objects/2,
      map_root_object/2,
      map_object_dimensions/4,
      map_child_object/2,
      map_child_objects/2,
      map_object_label/2,
      map_connection_type/3,
      map_joint_name/3
    ]).
/** <module> Predicates for reading elements of a semantic map

@author Lars Kunze
@author Lorenz Moesenlechner
@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob/perception')).

:-  rdf_meta
      map_instance(r),
      map_name(r),
      map_read_all(-,r),
      map_root_object(r,r),
      map_root_objects(r,?),
      map_child_object(r,r),
      map_child_objects(r,?),
      map_object_info(?),
      map_object_type(r,r),
      map_object_dimensions(r,?,?,?),
      map_object_label(r,?),
      map_connection_type(r,r,+),
      map_joint_name(r,r,r).

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
    owl_has(Obj, knowrob:describedInMap, Map).
    
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
    rdf_reachable(Parent, knowrob:properPhysicalParts, Child),
    Parent \= Child,
    not( owl_individual_of(Child, knowrob:'Connection-Physical') ).

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
    owl_individual_of(Inst, knowrob:'SpatialThing'),
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
    holds(Obj, rdf:type, Type),
    once(owl_subclass_of(Type, knowrob:'SpatialThing')).

%% map_object_label(?Obj, ?Label) is nondet.
%
% Reads the rdfs:label of an instance
% 
% @param Obj    Object instance
% @param Label  Value of the rdfs:label annotation
%  
map_object_label(Obj, Label) :-
    owl_has(Obj, rdfs:label, literal(type(_, Label)) ).

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
% Read joints and connections between object parts


%% map_connection_type( Parent, Child, ConnectionType ) is nondet.
%
% Determine the type of connection between Parent and Child, e.g. the type of joint.
%
% @param Parent   Object instance
% @param Child    Object instance
% @param ConnectionType Type of the connecting element, e.g. 'HingedJoint', 'PrismaticJoint', or 'Fixed'
% 
% @tbd The computation does not seem correct, but I am not sure what these predicates
%       are to do after all, so I let them like this for now (MT).
% 
map_connection_type( Parent, Child, 'HingedJoint' ) :-
    holds( Parent, knowrob:'properPhysicalParts', Child ),
    holds( Child, knowrob:'properPhysicalParts', Joint ),
    owl_individual_of(Joint, knowrob:'HingedJoint'), !.

map_connection_type( Parent, Parent, 'PrismaticJoint' ) :-
    holds( Parent, rdf:type, knowrob:'Drawer' ), !.

map_connection_type( Parent, Child, 'Fixed' ) :-
    holds( Parent, knowrob:'properPhysicalParts', Child ).

%% map_joint_name(Parent, Child, Name) is nondet.
%
% Read the ID Name of a joint connecting Parent and Child
%
% @param Object instance
% @param Object instance
% @param Instance of a Connection-Physical
% 
map_joint_name( Parent, Child, Name ) :-
    holds( Parent, knowrob:'properPhysicalParts', Child ),
    holds( Child, knowrob:'properPhysicalParts', Name ),
    owl_individual_of( Name, knowrob:'Connection-Physical' ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read map instance and content

%% map_instance(Map) is nondet.
%
% Read instances of a SemanticEnvironmentMap
% 
% @param Map  Instance of a knowrob:SemanticEnvironmentMap
% 
map_instance(Map) :-
    rdf_has(Map, rdf:type, knowrob:'SemanticEnvironmentMap').

%% map_name(Map) is nondet.
%
% Read the 'map_name' out of the map,
% 
% @param Map  Instance of a knowrob:SemanticEnvironmentMap
% 
map_name(Map) :-
    rdf_has(Map,rdf:type,knowrob:'SemanticEnvironmentMap').

%% map_read_all(-MapElems, ?MapInstance) is nondet.
%
% Read all objects in a map into a list of lists of form [Element, Parent,
% PoseList].
% 
% @param ObjList  List of all objects
% @param Map      Instance of a knowrob:SemanticEnvironmentMap
% 
map_read_all([R0|EntityList], MapInstance) :-
    map_read_entity(MapInstance, 'null', R0),
    setof(TopLevelObj, owl_has(TopLevelObj, knowrob:describedInMap, MapInstance), TopLevelObjs),
    map_read_entities(TopLevelObjs, MapInstance, EntityList).

map_read_entities([], _, []).
map_read_entities([E|Es], P, Res) :-
  ((setof(Part, holds(E, knowrob:properPhysicalParts, Part), Parts)) -> (
    map_read_entity(E, P, R0),
    map_read_entities(Parts, E, Rs),
    append([R0], Rs, R)
  );(
    map_read_entity(E, P, R1),
    R=[R1]
  )),
  map_read_entities(Es, P, Res1),
  append(R, Res1, Res).

map_read_entity(E, Parent, [E_no_ns, Parent_no_ns, Pose]) :-
    rdf_split_url(_, E_no_ns, E),
    ((Parent = 'null') ->
     (Parent_no_ns = Parent) ;
     rdf_split_url(_, Parent_no_ns, Parent)),
    current_object_pose( E, Pose ).
