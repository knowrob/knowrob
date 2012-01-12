%%
%% Copyright (C) 2009 by Lars Kunze, Lorenz Moesenlechner, Moritz Tenorth
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

:- module(ias_semantic_map,
    [
      readMap/1,
      rdf_atom_no_ns/2,
      withoutNamespace/2,
      objectType/2,
      rootObjects/1,
      rootObject/1,
      objectDimensions/4,
      objectPose/2,
      childObject/2,
      childObjects/2,
      objectLabel/2,
      semanticMap/1,
      connectionType/3,
      jointName/3
    ]).

readMap([R0|EntityList]) :-
    readEntity('http://ias.cs.tum.edu/kb/knowrob.owl#SemanticEnvironmentMap0', 'null', R0),
    setof(TopLevelObj,
          ( owl_individual_of(TopLevelObj, knowrob:'StorageConstruct');
            owl_individual_of(TopLevelObj, knowrob:'FurniturePiece');
            owl_individual_of(TopLevelObj, knowrob:'CounterTop') ),
          TopLevelObjs),
    readEntities(TopLevelObjs, 'http://ias.cs.tum.edu/kb/knowrob.owl#SemanticEnvironmentMap0', EntityList).
    

readEntities([], _, []).
readEntities([E|Es], P, Res) :-
  ((setof(Prt, rdf_has(E, knowrob:properPhysicalParts, Prt), Prts)) -> (
    readEntity(E, P, R0),
    readEntities(Prts, E, Rs),
    append([R0], Rs, R)
  );(
    readEntity(E, P, R1),
    R=[R1]
  )),
  readEntities(Es, P, Res1),
  append(R, Res1, Res).

  
readEntity(E, Parent, [E_no_ns, Parent_no_ns, Pose]) :-
    rdf_atom_no_ns(E, E_no_ns),
    ((Parent = 'null') ->
     (Parent_no_ns = Parent) ;
     rdf_atom_no_ns(Parent, Parent_no_ns)),
    objectPose( E, Pose ).

rdf_atom_no_ns( Tin, Tout ) :-
    rdf_split_url(_, Tout, Tin ).

withoutNamespace( Tin, Tout ) :-
    rdf_split_url(_, Tout, Tin).

objectType(O, T) :-
    rdf_has( O, rdf:type, T ),
    owl_subclass_of( T, knowrob:'SpatialThing').

semanticMap( Map ) :-
    rdf_has(Map, rdf:type, knowrob:'SemanticEnvironmentMap').

rootObjects( Os ) :-
    setof( O, rootObject(O), Os).

rootObject( O ) :-
    owl_individual_of(O, knowrob:'StorageConstruct');
    owl_individual_of(O, knowrob:'WallOfAConstruction');
    owl_individual_of(O, knowrob:'FurniturePiece');
    owl_individual_of(O, knowrob:'CounterTop').

objectInfo([Uri, Type, Pose, [D, W, H]]) :-
    rootObject(Uri),
    objectType(Uri, Type),
    objectPose(Uri, Pose),
    objectDimensions(Uri, W, D, H).

objectDimensions( O, W, D, H ) :-
    rdf_has( O, knowrob:'widthOfObject', literal(type(_, W_)) ),
    atom_number(W_, W),
    rdf_has( O, knowrob:'depthOfObject', literal(type(_, D_)) ),
    atom_number(D_, D),
    rdf_has( O, knowrob:'heightOfObject', literal(type(_, H_)) ),
    atom_number(H_, H).

objectDimensions( O, W, D, H ) :-
    % The depth of a knob defaults to 3cm here. This information
    % should either be asserted somewhere else or be set as a property
    % when importing the semantic map.
    rdf_has( O, knowrob:'radius', literal(type(_, R_)) ),
    atom_number(R_, R),
    W is 2 * R,
    H is 2 * R,
    D is 0.03.

childObject( P, C ) :-
    rdf_has( P, knowrob:'properPhysicalParts', C ),
    not( owl_individual_of(C, knowrob:'Connection-Physical') ).

objectLabel( O, L ) :-
    owl_has( O, rdfs:label, literal(type('http://www.w3.org/2001/XMLSchema#string', L)) ).

connectionType( P, C, 'HingedJoint' ) :-
    rdf_has( P, knowrob:'properPhysicalParts', C ),
    rdf_has( C, knowrob:'properPhysicalParts', J ),
    owl_individual_of(J, knowrob:'HingedJoint'), !.

connectionType( P, P, 'SliderJoint' ) :-
    rdf_has( P, rdf:type, knowrob:'Drawer' ), !.

connectionType( P, C, 'Fixed' ) :-
    rdf_has( P, knowrob:'properPhysicalParts', C ).

jointName( P, C, Name ) :-
    rdf_has( P, knowrob:'properPhysicalParts', C ),
    rdf_has( C, knowrob:'properPhysicalParts', Name ),
    owl_individual_of( Name, knowrob:'Connection-Physical' ).

childObjects( P, Cs ) :-
    setof( C, childObject(P, C), Cs).

objectPose( O, P ) :-
    rdf_triple(knowrob:orientation, O, Pobj ),
    bagof( V,
           X^Y^(member(Y, [0, 1, 2, 3]),
                member(X, [0, 1, 2, 3]),
                matrixValue( Pobj, X, Y, V )),
           P ).

matrixValue( M, X, Y, V ) :-
    concat( m, Y, Tmp ),
    concat( Tmp, X, Name ),
    rdf_global_id(knowrob:Name, FullName),
    rdf_triple( FullName, M, literal(type(_, V_)) ),
    atom_number( V_, V ).
