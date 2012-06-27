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
	mesh_reasoning/2,
	mesh_reasoning_path/2,
	mesh_element_types/2,
	mesh_find_annotations/3,
	mesh_is_supporting_plane/1,
	mesh_is_supporting_plane/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_objects')).
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
mesh_reasoning_init(MeshReasoning, WithCanvas) :-
    jpl_call('edu.tum.cs.vis.model.MeshReasoning', 'initMeshReasoning', [WithCanvas], MeshReasoning).
mesh_reasoning_init(MeshReasoning) :-
	mesh_reasoning_init(MeshReasoning, @(false)).

%% mesh_reasoning(+Identifier, -MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given identifier.
%
% @param Identifier 	   eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls" or "knowrob:'Spoon'"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning(Identifier, MeshReasoning) :-
	mesh_reasoning_init(MeshReasoning),
    jpl_call(MeshReasoning, 'analyzeByIdentifier', [Identifier], _).


%% mesh_reasoning_path(+Path, -MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given path (supported: local, package://, http://, ftp://).
%
% @param Path eg.   "/home/user/model.dae" or "http://example.com/model.kmz"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning_path(Path, MeshReasoning) :-
	mesh_reasoning_init(MeshReasoning),
    jpl_call(MeshReasoning, 'analyzeByPath', [Path], _).
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Annotations
%

%% mesh_element_types(+MeshReasoning,-TypeList) is det
%
% Get list of all found annotation types for current model in MeshReasoning
%
% @param MeshReasoning		reasoning container
% @param TypeList			List with annotation types eg: ['Plane','Sphere','Cone','Container']. Values can be directly used in mesh_find_annotations as Type
%
mesh_element_types(MeshReasoning, TypeList) :-
	jpl_call(MeshReasoning, 'getAnnotationTypes', [], TypeListFlat),
    jpl_call(TypeListFlat, toArray, [], TypeListArr),
	jpl_array_to_list(TypeListArr, TypeList).
	

%% mesh_find_annotations(+MeshReasoning,+Type, -AnnotationsList) is det.
%
% Get a list of all annotations with given type
%
% @param MeshReasoning		reasoning container
% @param Type		String indicating annotation type (Plane,Sphere,Cone,Container)
% @param AnnotationsList List with found annotations
%
mesh_find_annotations(MeshReasoning,Type,AnnotationsList) :-
	concat('findAnnotations', Type, Method),
	jpl_call(MeshReasoning, Method, [], AnnotationsList).
	
%% mesh_is_supporting_plane(+PlaneAnnotation) is det.
%% mesh_is_supporting_plane(+PlaneAnnotation, +Identifier) is det.
%
% Check if plane annotation surface normal is upwards.
% Means checking if abs(acos(n.z)) < 10 degree = PI/18 rad 
%
% @param PlaneAnnotation	A Java Reference Object for a plane annotation
% @param Identifier			If Identifier is instantiated the current pose of the object is also considered when checking if normal is upwards
%
mesh_is_supporting_plane(PlaneAnnotation, Identifier) :-
	jpl_call(PlaneAnnotation,'getPlaneNormal',[],Norm),
	jpl_get(Norm,'z',NormZ),
	(nonvar(Identifier) -> 
		% we only need M22 because '(rot) * (0,0,NormZ,0) = (_, _, NewNormZ, _)' for calculating angle
		current_object_pose(Identifier,[_, _, _, _, _, _, _, _, _, _, M22, _, _, _, _, _]),
		NormZ is M22 * NormZ,
		true
	; 
		true
	),
	abs(acos(NormZ)) < pi / 18.
mesh_is_supporting_plane(PlaneAnnotation) :-
	mesh_is_supporting_plane(PlaneAnnotation,_).
