/** <module> knowrob_mesh_reasoning

  Description:
    Module providing mesh reasoning capabilities


  Copyright (C) 2012 by Stefan Profanter

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Stefan Profanter
@license GPL
*/

:- module(knowrob_mesh_reasoning,
    [
	mesh_reasoning_init/1,
	mesh_reasoning_init/2,
	mesh_reasoning_identifier/2,
	mesh_reasoning_path/2,
	mesh_reasoning_find_annotations/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Main
%

%% mesh_reasoning_init(-MeshReasoning,+WithCanvas) is det.
%% mesh_reasoning_init(-MeshReasoning) is det.
%
% Create mesh reasoning object. WithCanvas indicates if you want to show canvas window.
% WithCanvas defaults to true if not indicated
%
:- assert(mesh_reasoning(fail)).
mesh_reasoning_init(MeshReasoning, WithCanvas) :-
    mesh_reasoning(fail),
    jpl_call('edu.tum.cs.vis.model.MeshReasoning', 'initMeshReasoning', [WithCanvas], MeshReasoning),
    retract(mesh_reasoning(fail)),
    assert(mesh_reasoning(MeshReasoning)),!.
mesh_reasoning_init(MeshReasoning) :-
    jpl_call('edu.tum.cs.vis.model.MeshReasoning', 'initMeshReasoning', [@(true)], MeshReasoning),
    retract(mesh_reasoning(fail)),
    assert(mesh_reasoning(MeshReasoning)),!.
mesh_reasoning_init(MeshReasoning) :-
    mesh_reasoning(MeshReasoning).

%% mesh_reasoning_identifier(+Identifier, +MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given identifier.
%
% @param Identifier 	   eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning_identifier(Identifier, MeshReasoning) :-
    ((var(MeshReasoning)) -> (mesh_reasoning(MeshReasoning));(true)),
    jpl_call(MeshReasoning, 'analyzeByIdentifier', [Identifier], _).


%% mesh_reasoning_path(+Path, +MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given path (supported: local, package://, http://, ftp://).
%
% @param Path eg.   "/home/user/model.dae" or "http://example.com/model.kmz"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning_path(Path, MeshReasoning) :-
    ((var(MeshReasoning)) -> (mesh_reasoning(MeshReasoning));(true)),
    jpl_call(MeshReasoning, 'analyzeByPath', [Path], _).

%% mesh_reasoning_find_annotations(+Cas,+Type, -AnnotationsList) is det.
%
% Get a list of all annotations with given type
%
% @param MeshReasoning		reasoning container
% @param Type		String indicating annotation type (Plane,Sphere,Cone,Container)
% @param AnnotationsList List with found annotations
%
mesh_reasoning_find_annotations(MeshReasoning,Type,AnnotationsList) :-
    	((var(MeshReasoning)) -> (mesh_reasoning(MeshReasoning));(true)),
	jpl_call(MeshReasoning, 'findAnnotations', [Type], AnnotationsList).
