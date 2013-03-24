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
      mesh_annotator/2,
      mesh_annotator_path/2,
      mesh_element_types/2,
      mesh_find_annotations/3,
      mesh_find_supporting_planes/2,
      mesh_is_supporting_plane/1,
      mesh_is_supporting_plane/2,
      mesh_annotator_highlight/2,
      mesh_annotator_clear_highlight/1,
      mesh_find_handle/2,
      mesh_find_handle/4,
      mesh_find_handle/6,
      listsplit/3,
      jpl_set_to_list/2,
      comp_physical_parts/2,
      annotation_area/2,
      annotation_area_coverage/2,
      annotation_pose_list/2,
      annotation_cone_radius_avg/2,
      annotation_cone_radius_max/2,
      annotation_cone_radius_min/2,
      annotation_cone_volume/2,
      annotation_cone_height/2,
      annotation_cone_direction/2,
      annotation_sphere_radius/2,
      annotation_sphere_is_concave/2,
      annotation_sphere_volume/2,
      annotation_plane_normal/2,
      annotation_plane_longside/2,
      annotation_plane_shortside/2,
      annotation_container_direction/2,
      annotation_container_volume/2,
      annotation_supporting_plane/2,
      annotation_handle/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('jpl')).


:- rdf_meta comp_physical_parts(r,r),
            mesh_annotator_java_obj(r,?),
            mesh_annotation_java_obj(r,?),
            mesh_annotator(r,-),
            comp_physical_parts(r,r),
            annotation_handle(r,r),
            annotation_supporting_plane(r,r),
            annotation_area(r,?),
            annotation_area_coverage(r,?),
            annotation_pose_list(r,?),
            annotation_cone_radius_avg(r,?),
            annotation_cone_radius_max(r,?),
            annotation_cone_radius_min(r,?),
            annotation_cone_volume(r,?),
            annotation_cone_height(r,?),
            annotation_cone_direction(r,?),
            annotation_sphere_radius(r,?),
            annotation_sphere_is_concave(r,?),
            annotation_sphere_volume(r,?),
            annotation_plane_normal(r,?),
            annotation_plane_longside(r,?),
            annotation_plane_shortside(r,?),
            annotation_container_direction(r,?),
            annotation_container_volume(r,?),
            object_main_cone(r,r),
            object_main_axis(r,?).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% predicates for paper eval:

object_main_cone(Inst, MainCone) :-

  % read all cone annotations
  findall(P, (rdf_has(Inst, knowrob:properPhysicalParts, P),
              owl_individual_of(P, knowrob:'Cone')), Parts),

  % read volume for each of them
  findall(V-P, (member(P, Parts),
                rdf_triple(knowrob:volumeOfObject, P, Vlit),
                strip_literal_type(Vlit, V)), PartVol),

  % sort list by volume and pick last element (largest volume)
  keysort(PartVol, PartVolSorted),
  last(PartVolSorted, _-MainCone).

object_main_axis(Obj, [X,Y,Z]) :-
  object_main_cone(Obj, C),
  rdf_triple(knowrob:longitudinalDirection, C, Dir),
  rdf_has(Dir, knowrob:vectorX, literal(type(xsd:'float',X))),
  rdf_has(Dir, knowrob:vectorY, literal(type(xsd:'float',Y))),
  rdf_has(Dir, knowrob:vectorZ, literal(type(xsd:'float',Z))).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Main
%

%% mesh_annotator(+Identifier, -MeshAnnotator) is det.
%
% Do mesh reasoning on cad model with given identifier.
%
% @param Identifier 	   eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls" or "knowrob:'Spoon'"
% @param MeshAnnotator     MeshAnnotator object
%
mesh_annotator(Identifier, MeshAnnotator) :-
  get_model_path(Identifier,Path),
  mesh_annotator_path(Path, MeshAnnotator).

%% mesh_annotator_path(+Path, -MeshAnnotator) is det.
%
% Do mesh reasoning on cad model with given path (supported: local, package://, http://, ftp://).
%
% @param Path eg.   "/home/user/model.dae" or "http://example.com/model.kmz"
% @param MeshAnnotator     MeshAnnotator object
%
mesh_annotator_path(Path, MeshAnnotator) :-
  mesh_annotator_init(MeshAnnotator),
  jpl_call(MeshAnnotator, 'analyseByPath', [Path], _).


%% mesh_annotator_init(-MeshAnnotator,+WithCanvas) is det.
%% mesh_annotator_init(-MeshAnnotator) is det.
%
% Create mesh reasoning object. WithCanvas indicates if you want to show canvas window.
% WithCanvas defaults to true if not indicated
%
mesh_annotator_init(MeshAnnotator, WithCanvas) :-
  jpl_call('edu.tum.cs.vis.model.MeshReasoning', 'initMeshReasoning', [WithCanvas], MeshAnnotator).
mesh_annotator_init(MeshAnnotator) :-
  mesh_annotator_init(MeshAnnotator, @(true)).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Annotations
%

%% mesh_annotator_highlight(+MeshAnnotator,+[AnnotationHead|AnnotationTail]) is det
%% mesh_annotator_highlight(+MeshAnnotator,+[]) is det
%% mesh_annotator_highlight(+MeshAnnotator,+Annotation) is det
%
% Highlight/select the specified annotation in GUI
%
% @param MeshAnnotator		reasoning container
% @param Annotation			annotation to highlight
mesh_annotator_highlight(MeshAnnotator,[AnnotationHead|AnnotationTail]) :-
	mesh_annotator_highlight(MeshAnnotator, AnnotationHead),!,
	mesh_annotator_highlight(MeshAnnotator, AnnotationTail),!.
mesh_annotator_highlight(_,[]).
mesh_annotator_highlight(MeshAnnotator, Annotation) :-
	jpl_call(MeshAnnotator, 'highlightAnnotation', [Annotation], _).

%% mesh_annotator_clear_hightlight(+MeshAnnotator) is det
%
% Clear all annotation highlights in GUI
%
% @param MeshAnnotator		reasoning container
%
mesh_annotator_clear_highlight(MeshAnnotator) :-
	jpl_call(MeshAnnotator, 'clearHightlight', [], _).


%% mesh_element_types(+MeshAnnotator,-TypeList) is det
%
% Get list of all found annotation types for current model in MeshAnnotator
%
% @param MeshAnnotator		reasoning container
% @param TypeList			List with annotation types eg: ['Plane','Sphere','Cone','Container']. Values can be directly used in mesh_find_annotations as Type
%
mesh_element_types(MeshAnnotator, TypeList) :-
	jpl_call(MeshAnnotator, 'getAnnotationTypes', [], TypeListFlat),
    jpl_call(TypeListFlat, toArray, [], TypeListArr),
	jpl_array_to_list(TypeListArr, TypeList).


%% mesh_find_annotations(+MeshAnnotator,+Type, -AnnotationsList) is det.
%
% Get a list of all annotations with given type
%
% @param MeshAnnotator		reasoning container
% @param Type		String indicating annotation type (Plane,Sphere,Cone,Container)
% @param AnnotationsList List with found annotations
%
mesh_find_annotations(MeshAnnotator,Type,AnnotationsList) :-
	concat('findAnnotations', Type, Method),
	jpl_call(MeshAnnotator, Method, [], AnnotationsSet),
  jpl_set_to_list(AnnotationsSet, AnnotationsList).


%% mesh_find_supporting_planes(+MeshAnnotator, -PlaneList) is det.
%
% Get list of all supporting planes
%
% @param MeshAnnotator		reasoning container
% @param PlaneList			returning list which contains all supporting planes
%
mesh_find_supporting_planes(MeshAnnotator, PlaneList) :-
	mesh_find_annotations(MeshAnnotator,'Plane',AnnList),
	findall(P,(member(P, AnnList),mesh_is_supporting_plane(P)),PlaneList).


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

listsplit([H|T], H, T).

%% jpl_set_to_list(+Set,-List) is det.
%
%  Extracts all elements from a JPL set to a prolog list by calling jpl_set_element
%
% @param Set	The jpl set
% @param List	returning list
%
jpl_set_to_list(Set,List) :-
  findall(P,jpl_set_element(Set,P),List).

%% mesh_find_handle(+MeshAnnotator, -HandleAnnotations) is det.
%% mesh_find_handle(+MeshAnnotator, -HandleAnnotations, +MinRadius, +MaxRadius) is det.
%% mesh_find_handle(+MeshAnnotator, -HandleAnnotations, +MinRadius, +MaxRadius, +MinLength, +MaxLength) is det.
%
% Returns a list which contains annotations sorted by its probability that they are the object handle.
% Sorting is based on a weight function defined in Java. If no boundaries for radius and length are given,
% the weight is proportional to the radius. If only radius is given, the length is ignored. If radius and length
% is given, both are used to calculate the weight.
%
% @param MeshAnnotator			reasoning container
% @param HandleAnnotations		the resulting sorted annotation list
% @param MinRadius			minimum radius which the handle should have
% @param MaxRadius			maximum radius which the handle should have
% @param MinLength			minimum length which the handle should have
% @param MaxLength			maximum length which the handle should have
%
mesh_find_handle(MeshAnnotator, HandleAnnotations) :-
  jpl_call(MeshAnnotator, 'getHandle', [], HandleArr),
  jpl_array_to_list(HandleArr, HandleAnnotations).

mesh_find_handle(MeshAnnotator, HandleAnnotations, MinRadius, MaxRadius) :-
  jpl_call(MeshAnnotator, 'getHandle', [MinRadius, MaxRadius], HandleArr),
  jpl_array_to_list(HandleArr, HandleAnnotations).

mesh_find_handle(MeshAnnotator, HandleAnnotations, MinRadius, MaxRadius, MinLength, MaxLength) :-
  jpl_call(MeshAnnotator, 'getHandle', [MinRadius, MaxRadius, MinLength, MaxLength], HandleArr),
  jpl_array_to_list(HandleArr, HandleAnnotations).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Computable definitions
%

%
% check if mesh annotator has already been initialized for this instance
% if not, initialize and assert for this object
%

:- dynamic
   mesh_annotator_java_obj/2,
   mesh_annotation_java_obj/2.

mesh_annotator_for_obj(Obj, MeshAnnotator) :-
  ((mesh_annotator_java_obj(Obj, MeshAnnotator),!) ;
   (get_model_path(Obj, Path),
    mesh_annotator_init(MeshAnnotator, @(false)),
    jpl_call(MeshAnnotator, 'analyseByPath', [Path], _),
    assert(mesh_annotator_java_obj(Obj, MeshAnnotator)))).



comp_physical_parts(Obj, PartInst) :-

  % avoid re-creation of object parts
  (\+ mesh_annotator_java_obj(Obj,_)),

  % locates mesh for the object
  % determines object components
  % assert java reference to mesh reasoning object
  mesh_annotator_for_obj(Obj, MeshAnnotator),
  mesh_element_types(MeshAnnotator, ContainedTypes),
  member(Type, ContainedTypes),

  mesh_find_annotations(MeshAnnotator,Type,AnnotationsList),
  member(Annotation, AnnotationsList),

  % TODO: transform into global coordinates!!
  annotation_to_knowrob_class(Type, KnowRobClass),

  % TODO workaround: container does not have a pose yet
  (annotation_pose_list(Annotation, PoseList) -> (
   create_object_perception(KnowRobClass, PoseList, ['MeshSegmentation'], PartInst)) ;
   rdf_instance_from_class(KnowRobClass, PartInst)),

  % assert sub-parts
  rdf_assert(Obj, knowrob:properPhysicalParts, PartInst),

  % assert Java annotation ID for each plane/sphere/... in order to retrieve their properties
  assert(mesh_annotation_java_obj(PartInst, Annotation)).


annotation_to_knowrob_class('Cone', 'Cone').
annotation_to_knowrob_class('Sphere', 'Sphere').
annotation_to_knowrob_class('Plane', 'FlatPhysicalSurface').
annotation_to_knowrob_class('Container', 'ContainerArtifact').


annotation_pose_list(PrimitiveAnnotation, PoseList) :-
  jpl_datum_to_type(PrimitiveAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],[_])),
  jpl_call(PrimitiveAnnotation,'getPoseMatrix',[],PoseMatrix),
  knowrob_coordinates:matrix4d_to_list(PoseMatrix, PoseList).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Computables for reading information about annotation properties
%

annotation_area(PartInst, Area) :-
  mesh_annotation_java_obj(PartInst, PrimitiveAnnotation),
  jpl_datum_to_type(PrimitiveAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],[_])),
  jpl_call(PrimitiveAnnotation,'getPrimitiveArea',[],Area).

annotation_area_coverage(PartInst, AreaCoverage) :-
  mesh_annotation_java_obj(PartInst, PrimitiveAnnotation),
  jpl_datum_to_type(PrimitiveAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],[_])),
  jpl_call(PrimitiveAnnotation,'getAreaCoverage',[],AreaCoverage).




% % % % % % % % % % % % % % % % % % % % % % %
% CONES

annotation_cone_radius_avg(PartInst, RadiusAvg) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getRadiusAvg',[],RadiusAvg).

annotation_cone_radius_max(PartInst, RadiusMax) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getRadiusLarge',[],RadiusMax).

annotation_cone_radius_min(PartInst, RadiusMin) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getRadiusSmall',[],RadiusMin).

annotation_cone_volume(PartInst, Volume) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getVolume',[],Volume).

annotation_cone_height(PartInst, Height) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getHeight',[],Height).

annotation_cone_direction(PartInst, Direction) :-
  mesh_annotation_java_obj(PartInst, ConeAnnotation),
  jpl_datum_to_type(ConeAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['ConeAnnotation'])),
  jpl_call(ConeAnnotation,'getDirection',[],DirVec),
  knowrob_coordinates:vector3d_to_list(DirVec, VecList),

  VecList = [VX, VY, VZ],

  rdf_instance_from_class(knowrob:'Vector', Direction),
  rdf_assert(Direction, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
  rdf_assert(Direction, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
  rdf_assert(Direction, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))).




% % % % % % % % % % % % % % % % % % % % % % %
% SPHERES

annotation_sphere_radius(PartInst, RadiusAvg) :-
  mesh_annotation_java_obj(PartInst, SphereAnnotation),
  jpl_datum_to_type(SphereAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['SphereAnnotation'])),
  jpl_call(SphereAnnotation,'getRadius',[],RadiusAvg).

annotation_sphere_is_concave(PartInst, ConcaveObjClass) :-
  owl_subclass_of(ConcaveObjClass, 'http://ias.cs.tum.edu/kb/knowrob.owl#ConcaveTangibleObject'),
  mesh_annotation_java_obj(PartInst, SphereAnnotation),
  jpl_datum_to_type(SphereAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['SphereAnnotation'])),
  jpl_call(SphereAnnotation,'isConcave',[],@(true)).

annotation_sphere_volume(PartInst, Volume) :-
  mesh_annotation_java_obj(PartInst, SphereAnnotation),
  jpl_datum_to_type(SphereAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['SphereAnnotation'])),
  jpl_call(SphereAnnotation,'getVolume',[],Volume).




% % % % % % % % % % % % % % % % % % % % % % %
% Planes

annotation_plane_normal(PartInst, NormalVec) :-
  mesh_annotation_java_obj(PartInst, PlaneAnnotation),
  jpl_datum_to_type(PlaneAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['PlaneAnnotation'])),
  jpl_call(PlaneAnnotation,'getPlaneNormal',[],NormalVec3d),
  knowrob_coordinates:vector3d_to_list(NormalVec3d, VecList),

  VecList = [VX, VY, VZ],

  rdf_instance_from_class(knowrob:'Vector', NormalVec),
  rdf_assert(NormalVec, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
  rdf_assert(NormalVec, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
  rdf_assert(NormalVec, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))).


annotation_plane_longside(PartInst, LongSide) :-
  mesh_annotation_java_obj(PartInst, PlaneAnnotation),
  jpl_datum_to_type(PlaneAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['PlaneAnnotation'])),
  jpl_call(PlaneAnnotation,'getLongSide',[],LongSideVec),
  knowrob_coordinates:vector3d_to_list(LongSideVec, VecList),

  VecList = [VX, VY, VZ],

  rdf_instance_from_class(knowrob:'Vector', LongSide),
  rdf_assert(LongSide, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
  rdf_assert(LongSide, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
  rdf_assert(LongSide, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))).


annotation_plane_shortside(PartInst, ShortSide) :-
  mesh_annotation_java_obj(PartInst, PlaneAnnotation),
  jpl_datum_to_type(PlaneAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation,primitive],['PlaneAnnotation'])),
  jpl_call(PlaneAnnotation,'getShortSide',[],ShortSideVec),
  knowrob_coordinates:vector3d_to_list(ShortSideVec, VecList),

  VecList = [VX, VY, VZ],

  rdf_instance_from_class(knowrob:'Vector', ShortSide),
  rdf_assert(ShortSide, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
  rdf_assert(ShortSide, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
  rdf_assert(ShortSide, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))).


annotation_supporting_plane(PartInst, SuppPlaneClass) :-

  once(owl_subclass_of(SuppPlaneClass, 'http://ias.cs.tum.edu/kb/knowrob.owl#SupportingPlane')),

  findall(Plane, owl_individual_of(Plane, knowrob:'FlatPhysicalSurface'), Planes),
  member(PartInst, Planes),

  mesh_annotation_java_obj(PartInst, PlaneAnnotation),
  mesh_is_supporting_plane(PlaneAnnotation, PartInst).


% % % % % % % % % % % % % % % % % % % % % % %
% Handles

% check if an existing object part (e.g. cylinder) is a handle
annotation_handle(PartInst, HandleClass) :-

  var(PartInst),
  once(owl_subclass_of(HandleClass, 'http://ias.cs.tum.edu/kb/knowrob.owl#Handle')),

  % find mesh annotator stored for the parent object of this part
  once((owl_has(Obj, knowrob:properPhysicalParts, PartInst),
   mesh_annotator_java_obj(Obj, MeshAnnotator))),

  % this will likely return many candidates if we only sort, but do not filter candidates
  mesh_find_handle(MeshAnnotator, HandleAnnotations),
  member(HandleAnn, HandleAnnotations),
  mesh_annotation_java_obj(PartInst, HandleAnn). % TODO: find respective cone instance and assing class




% % % % % % % % % % % % % % % % % % % % % % %
% Containers

annotation_container_direction(PartInst, OpeningDirection) :-
  mesh_annotation_java_obj(PartInst, ContainerAnnotation),
  jpl_datum_to_type(ContainerAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation],['ContainerAnnotation'])),
  jpl_call(ContainerAnnotation,'getDirection',[], VecList),

  VecList = [VX, VY, VZ],

  rdf_instance_from_class(knowrob:'Vector', OpeningDirection),
  rdf_assert(OpeningDirection, knowrob:vectorX, literal(type('http://www.w3.org/2001/XMLSchema#float', VX))),
  rdf_assert(OpeningDirection, knowrob:vectorY, literal(type('http://www.w3.org/2001/XMLSchema#float', VY))),
  rdf_assert(OpeningDirection, knowrob:vectorZ, literal(type('http://www.w3.org/2001/XMLSchema#float', VZ))).

annotation_container_volume(PartInst, Volume) :-
  mesh_annotation_java_obj(PartInst, ContainerAnnotation),
  jpl_datum_to_type(ContainerAnnotation,
      class([edu,tum,cs,vis,model,uima,annotation],['ContainerAnnotation'])),
  jpl_call(ContainerAnnotation,'getVolume',[],Volume).

