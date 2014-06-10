%%
%% Copyright (C) 2009-13 by Lars Kunze, Lorenz Moesenlechner, Moritz Tenorth
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- module(semantic_map_utils,
    [
      map_instance/1,
      map_read_all/1,
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
      map_joint_name/3,
      rdf_atom_no_ns/2
    ]).


:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_perception')).


:-  rdf_meta
      map_instance(r),
      map_read_all(-),
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
      map_joint_name(r,r,r),
      rdf_atom_no_ns(r,?).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Map root and child objects

map_root_objects( Map, Objs ) :-
    setof( Obj, map_root_object(Map, Obj), Objs).

map_root_object( Map, Obj ) :-
    owl_has(Obj, knowrob:describedInMap, Map),
    (owl_individual_of(Obj, knowrob:'StorageConstruct');
    owl_individual_of(Obj, knowrob:'WallOfAConstruction');
    owl_individual_of(Obj, knowrob:'FurniturePiece');
    owl_individual_of(Obj, knowrob:'CounterTop')).


map_child_objects( Parent, Children ) :-
    setof( Child, map_child_object(Parent, Child), Children).

map_child_object( Parent, Child ) :-
    owl_has( Parent, knowrob:'properPhysicalParts', Child ),
    not( owl_individual_of(Child, knowrob:'Connection-Physical') ).

    

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read object properties


map_object_info([Inst, Type, Pose, [D, W, H]]) :-
    map_instance(Map),
    map_root_object(Map, Inst),
    map_object_type(Inst, Type),
    current_object_pose(Inst, Pose),
    map_object_dimensions(Inst, W, D, H).

map_object_type(Obj, Type) :-
    rdf_has( Obj, rdf:type, Type ),
    owl_subclass_of( Type, knowrob:'SpatialThing').

map_object_label( Obj, Label ) :-
    owl_has( Obj, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', Label)) ).

map_object_dimensions( Obj, W, D, H ) :-
    rdf_has( Obj, knowrob:'widthOfObject', literal(type(_, W_)) ),
    atom_number(W_, W),
    rdf_has( Obj, knowrob:'depthOfObject', literal(type(_, D_)) ),
    atom_number(D_, D),
    rdf_has( Obj, knowrob:'heightOfObject', literal(type(_, H_)) ),
    atom_number(H_, H).

map_object_dimensions( Obj, W, D, H ) :-
    % The depth of a knob defaults to 3cm here. This information
    % should either be asserted somewhere else or be set as a property
    % when importing the semantic map.
    rdf_has( Obj, knowrob:'radius', literal(type(_, Radius_)) ),
    atom_number(Radius_, Radius),
    W is 2 * Radius,
    H is 2 * Radius,
    D is 0.03.





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read joints and connections between object parts


% TODO: This does not seem correct, but I am not sure what these predicates
%       are to do after all, so I let them like this for now (MT).
map_connection_type( Parent, Child, 'HingedJoint' ) :-
    rdf_has( Parent, knowrob:'properPhysicalParts', Child ),
    rdf_has( Child, knowrob:'properPhysicalParts', Joint ),
    owl_individual_of(Joint, knowrob:'HingedJoint'), !.

map_connection_type( Parent, Parent, 'PrismaticJoint' ) :-
    rdf_has( Parent, rdf:type, knowrob:'Drawer' ), !.

map_connection_type( Parent, Child, 'Fixed' ) :-
    rdf_has( Parent, knowrob:'properPhysicalParts', Child ).

map_joint_name( Parent, Child, Name ) :-
    rdf_has( Parent, knowrob:'properPhysicalParts', Child ),
    rdf_has( Child, knowrob:'properPhysicalParts', Name ),
    owl_individual_of( Name, knowrob:'Connection-Physical' ).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Read map instance and content

map_instance( Map ) :-
    rdf_has(Map, rdf:type, knowrob:'SemanticEnvironmentMap').

map_read_all([R0|EntityList]) :-
    map_read_all([R0|EntityList], 'http://ias.cs.tum.edu/kb/ias_semantic_map.owl#SemanticEnvironmentMap0').

map_read_all([R0|EntityList], MapInstance) :-
    map_read_entity(MapInstance, 'null', R0),
    setof(TopLevelObj, owl_has(TopLevelObj, knowrob:describedInMap, MapInstance), TopLevelObjs),
    map_read_entities(TopLevelObjs, MapInstance, EntityList).

map_read_entities([], _, []).
map_read_entities([E|Es], P, Res) :-
  ((setof(Part, rdf_has(E, knowrob:properPhysicalParts, Part), Parts)) -> (
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
    rdf_atom_no_ns(E, E_no_ns),
    ((Parent = 'null') ->
     (Parent_no_ns = Parent) ;
     rdf_atom_no_ns(Parent, Parent_no_ns)),
    current_object_pose( E, Pose ).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Utility methods

rdf_atom_no_ns( IRI, LocalName ) :-
    rdf_split_url(_, LocalName, IRI ).


