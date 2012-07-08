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
	mesh_find_supporting_planes/2,
	mesh_is_supporting_plane/1,
	mesh_is_supporting_plane/2,
	mesh_reasoning_highlight/2,
	mesh_reasoning_clear_highlight/1,
	mesh_find_handle/2,
	mesh_find_handle/4,
	listsplit/3
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
	mesh_reasoning_init(MeshReasoning, @(true)).

%% mesh_reasoning(+Identifier, -MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given identifier.
%
% @param Identifier 	   eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls" or "knowrob:'Spoon'"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning(Identifier, MeshReasoning) :-
	mesh_reasoning_init(MeshReasoning),
    jpl_call(MeshReasoning, 'analyseByIdentifier', [Identifier], _).


%% mesh_reasoning_path(+Path, -MeshReasoning) is det.
%
% Do mesh reasoning on cad model with given path (supported: local, package://, http://, ftp://).
%
% @param Path eg.   "/home/user/model.dae" or "http://example.com/model.kmz"
% @param MeshReasoning     MeshReasoning object
%
mesh_reasoning_path(Path, MeshReasoning) :-
	mesh_reasoning_init(MeshReasoning),
    jpl_call(MeshReasoning, 'analyseByPath', [Path], _).
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Annotations
%

%% mesh_reasoning_highlight(+MeshReasoning,+[AnnotationHead|AnnotationTail]) is det
%% mesh_reasoning_highlight(+MeshReasoning,+[]) is det
%% mesh_reasoning_highlight(+MeshReasoning,+Annotation) is det
%
% Highlight/select the specified annotation in GUI
%
% @param MeshReasoning		reasoning container
% @param Annotation			annotation to highlight
mesh_reasoning_highlight(_,[]).
mesh_reasoning_highlight(MeshReasoning,[AnnotationHead|AnnotationTail]) :-
	mesh_reasoning_highlight(MeshReasoning, AnnotationHead),
	mesh_reasoning_highlight(MeshReasoning, AnnotationTail).
mesh_reasoning_highlight(MeshReasoning, Annotation) :-
	jpl_call(MeshReasoning, 'highlightAnnotation', [Annotation], _).
	
%% mesh_reasoning_clear_hightlight(+MeshReasoning) is det
%
% Clear all annotation highlights in GUI
%
% @param MeshReasoning		reasoning container
%
mesh_reasoning_clear_highlight(MeshReasoning) :-
	jpl_call(MeshReasoning, 'clearHightlight', [], _).
	

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
	
	
%% mesh_find_supporting_planes(+MeshReasoning, -PlaneList) is det.
%
% Get list of all supporting planes
%
% @param MeshReasoning		reasoning container
% @param PlaneList			returning list which contains all supporting planes
%
mesh_find_supporting_planes(MeshReasoning, PlaneList) :-
	mesh_find_annotations(MeshReasoning,'Plane',AnnSet),
	findall(P,(jpl_set_element(AnnSet,P),mesh_is_supporting_plane(P)),PlaneList).


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
		% we only need M20,M21,M22 because '(rot) * (NormX,NormY,NormZ,0) = (_, _, NewNormZ, _)' for calculating angle
		jpl_get(Norm,'x',NormX),
		jpl_get(Norm,'y',NormY),
		current_object_pose(Identifier,[_, _, _, _, _, _, _, _, M20, M21, M22, _, _, _, _, _]),
		NormZ is M20 * NormX+M21 * NormY+M22 * NormZ,
		true
	; 
		true
	),
	abs(acos(NormZ)) < pi / 18.
mesh_is_supporting_plane(PlaneAnnotation) :-
	mesh_is_supporting_plane(PlaneAnnotation,_).

mesh_handle_comparator(Comp, W1, W2) :-
	jpl_call(W1,'getAreaCoverage',[],Cov1),
	jpl_call(W2,'getAreaCoverage',[],Cov2),
	jpl_call(W1,'getRadiusAvgUnscaled',[],Rad1),
	jpl_call(W2,'getRadiusAvgUnscaled',[],Rad2),
	( nonvar(MeshHandleCompareMinRadius) -> Rad1Ok = (Rad1 < MeshHandleCompareMinRadius , Rad1 > MeshHandleCompareMaxRadius)
	; Rad1Ok = false
	),
	( nonvar(MeshHandleCompareMinRadius) -> Rad2Ok = (Rad2 < MeshHandleCompareMinRadius , Rad2 > MeshHandleCompareMaxRadius)
	; Rad2Ok = false
	),
	(nonvar(MeshHandleCompareMinRadius) -> write('minrad defined\n') ; write('minrad NOT\n')),
	write('rad1:'),write(Rad1Ok),
	write('\nrad2:'),write(Rad2Ok),
	write('\n'),
	(	Cov1 < 0.6 , Cov2 >= 0.6 -> Comp = '>'
	;	Cov1 >= 0.6, Cov2 < 0.6 -> Comp = '<'
	;	not(Rad1Ok), Rad2Ok -> Comp = '>'
	;	Rad1Ok, not(Rad2Ok) -> Comp = '<'
	;	jpl_call(W1,'getHeightUnscaled',[],H1),
		jpl_call(W2,'getHeightUnscaled',[],H2),
		(   H1 < H2 -> Comp = '>'
		;   H1 >= H2 -> Comp = '<')
	).
	
listsplit([H|T], H, T).
	
%% mesh_find_handle(+MeshReasoning, -HandleAnnotations) is det.
%% mesh_find_handle(+MeshReasoning, -HandleAnnotations, +MinRadius, +MaxRadius) is det.
%
% Returns a list which contains annotations sorted by its probability that they are the object handle.
% Sorting is archeived by calling mesh_handle_comparator which compares two annotations by its probability.
%
% @param MeshReasoning			reasoning container
% @param HandleAnnotations		the resulting sorted annotation list
% @param MinRadius				minimum radius which the handle should have
% @param MaxRadius				maximum radius which the handle should have
%
mesh_find_handle(MeshReasoning, HandleAnnotations) :-
	mesh_find_annotations(MeshReasoning,'Cone',AnnSet),
	findall(P,jpl_set_element(AnnSet,P),AnnList),
	predsort(mesh_handle_comparator, AnnList, HandleAnnotations).
mesh_find_handle(MeshReasoning, HandleAnnotations, MinRadius, MaxRadius) :-
	MeshHandleCompareMinRadius = MinRadius,
	MeshHandleCompareMaxRadius = MaxRadius,
	mesh_find_handle(MeshReasoning, HandleAnnotations).
