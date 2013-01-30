%%
%% Copyright (C) 2011-2012 by Moritz Tenorth
%%
%% This module provides utilities for handling units of measure and the
%% conversion between different units in KnowRob.
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


:- module(knowrob_coordinates,
    [
      transform_relative_to/3,
      pose_into_relative_coord/3,
      pose_into_global_coord/3,
      angle_between_vector_normalized/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('tf_prolog')).
:- use_module(library('knowrob_objects')).
:- use_module(library('owl_export')).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_coordinates, 'http://ias.cs.tum.edu/kb/knowrob_coordinates.owl#', [keep(true)]).



%% transform_relative_to(+In, +Ref, -PoseListRel) is nondet.
%
% Compute relative pose of an object or pose matrix with respect to a
% reference object or pose
%
% @param In          Object or pose to be transformed relative to Ref
% @param Ref         Reference object or pose instance
% @param PoseListRel Resulting pose of In relative to Ref as list[16]
%
transform_relative_to(In, Ref, PoseListRel) :-

    ((owl_individual_of(In, knowrob:'RotationMatrix3D'), rotmat_to_list(In, InPose)) ;
      knowrob_objects:current_object_pose(In, InPose) ),

    ((owl_individual_of(Ref, knowrob:'RotationMatrix3D'), rotmat_to_list(Ref, RefPose)) ;
     knowrob_objects:current_object_pose(Ref, RefPose) ),

    list_to_matrix4d(InPose,  MatIn),
    list_to_matrix4d(RefPose, MatRef),

    pose4d_into_relative_coord(MatIn, MatRef, MatRel),
    matrix4d_to_list(MatRel,PoseListRel).





%% pose_into_relative_coord(+GlobalPose, +ReferencePose, -RelativePose) is nondet.
%
% Transform from a global pose to a relative position (from the reference
% object's parent coordinate frame into coordinates relative to the reference
% object)
%
% @param GlobalPose     Pose matrix in global coordinate frame as row-based float[16]
% @param ReferencePose  Reference pose matrix (to which RelativePose is to be relative) as row-based float[16]
% @param RelativePose   Pose matrix with coordinates relative to ReferencePose, as row-based float[16]
%
pose_into_relative_coord(GlobalPose, ReferencePose, RelativePose) :-

    is_list(GlobalPose), is_list(ReferencePose),

    list_to_matrix4d(GlobalPose,  GlobalPose4d),
    list_to_matrix4d(ReferencePose, ReferencePose4d),

    pose_into_relative_coord(GlobalPose4d, ReferencePose4d, RelativePose4d),

    matrix4d_to_list(RelativePose4d,RelativePose).



%% pose_into_relative_coord(+GlobalPose, +ReferencePose, -RelativePose) is nondet.
%
% Transform from a global pose to a relative position (from the reference
% object's parent coordinate frame into coordinates relative to the reference
% object)
%
% @param GlobalPose     Pose matrix in global coordinate frame as row-based float[16]
% @param ReferencePose  Reference pose matrix (to which RelativePose is to be relative) as row-based float[16]
% @param RelativePose   Pose matrix with coordinates relative to ReferencePose, as row-based float[16]
%

pose4d_into_relative_coord(GlobalPose4d, ReferencePose4d, RelativePose4d) :-

    jpl_new('javax.vecmath.Matrix4d', [], InvPoseRef4d),
    jpl_call(InvPoseRef4d, 'invert', [ReferencePose4d], _),

    jpl_new('javax.vecmath.Matrix4d', [], RelativePose4d),
    jpl_call(RelativePose4d, 'mul', [InvPoseRef4d, GlobalPose4d], _),!. % MT 040711: swapped order inv, global



% tf-version: transform into reference object's frame
pose4d_into_relative_coord(GlobalPose4d, ReferencePose4d, RelativePose4d) :-

    rdf_has(RelativePose4d,  knowrob:tfFrame, SourceTfFrame),
    rdf_has(ReferencePose4d, knowrob:tfFrame, ReferenceTfFrame),

    transform_pose_prolog(SourceTfFrame, RelativePose4d, ReferenceTfFrame, GlobalPose4d),!.  % TODO: source frame for _prolog predicates






%% pose_into_global_coord(+RelativePose, +ReferencePose, -GlobalPose) is nondet.
%
% Transform from relative pose to a global position (into the
% reference object's parent coordinate frame)
%
% @param RelativePose   Pose matrix with coordinates relative to ReferencePose, as row-based float[16]
% @param ReferencePose  Reference pose matrix (to which RelativePose is relative) as row-based float[16]
% @param GlobalPose     Pose matrix in global coordinate frame as row-based float[16]
%
pose_into_global_coord(RelativePose, ReferencePose, GlobalPose) :-

    list_to_matrix4d(RelativePose,  RelativePose4d),
    list_to_matrix4d(ReferencePose, ReferencePose4d),

    pose4d_into_global_coord(RelativePose4d, ReferencePose4d, GlobalPose4d),

    matrix4d_to_list(GlobalPose4d,GlobalPose).




%% pose4d_into_global_coord(+RelativePose4d, +ReferencePose4d, -GlobalPose4d) is nondet.
%
% Transform from relative pose to a global position (into the
% reference object's parent coordinate frame)
%
% There are three options:
% - ReferencePose4d bound: transform into parent coordinate system of ReferencePose4d
% - ReferencePose4d unbound, but poseRelativeTo set: use the specified relative pose as reference
% - ReferencePose4d unbound, but tfFrame set: use tf to convert the coordinates into the /map frame
%
% @param RelativePose   Pose matrix with coordinates relative to ReferencePose, as reference to Java Matrix4d
% @param ReferencePose  Reference pose matrix (to which RelativePose is relative) as reference to Java Matrix4d
% @param GlobalPose     Pose matrix in global coordinate frame as reference to Java Matrix4d
%

pose4d_into_global_coord(RelativePose4d, ReferencePose4d, GlobalPose4d) :-

    jpl_new('javax.vecmath.Matrix4d', [], GlobalPose4d),
    jpl_call(GlobalPose4d, 'mul', [RelativePose4d, ReferencePose4d], _),!.


% if ReferencePose unset but poseRelativeTo given, use this one instead
pose4d_into_global_coord(RelativePose4d, ReferencePose4d, GlobalPose4d) :-
    var(ReferencePose4d),
    rdf_has(RelativePose4d,  knowrob:poseRelativeTo, ReferencePose4d),
    pose_into_global_coord(RelativePose4d, ReferencePose4d, GlobalPose4d),!.


% tf-version: transform into /map if tfFrame is set
pose4d_into_global_coord(RelativePose4d, ReferencePose4d, GlobalPose4d) :-

    var(ReferencePose4d),
    rdf_has(RelativePose4d,  knowrob:tfFrame, TfFrame),

    matrix4d_to_list(RelativePose4d,RelativePoseList),
    transform_pose_prolog(RelativePoseList, TfFrame, '/map', GlobalPoseList),
    list_to_matrix4d(GlobalPoseList, GlobalPose4d),!.







% utility functions to translate from and to Java vectors/matrices

list_to_matrix4d(List, Matrix) :-
    nonvar(List), var(Matrix),
    jpl_new('javax.vecmath.Matrix4d', List, Matrix).



matrix4d_to_list(Matrix,List) :-

    var(List), nonvar(Matrix),

    jpl_get(Matrix, m00, M00),
    jpl_get(Matrix, m01, M01),
    jpl_get(Matrix, m02, M02),
    jpl_get(Matrix, m03, M03),

    jpl_get(Matrix, m10, M10),
    jpl_get(Matrix, m11, M11),
    jpl_get(Matrix, m12, M12),
    jpl_get(Matrix, m13, M13),

    jpl_get(Matrix, m20, M20),
    jpl_get(Matrix, m21, M21),
    jpl_get(Matrix, m22, M22),
    jpl_get(Matrix, m23, M23),

    jpl_get(Matrix, m30, M30),
    jpl_get(Matrix, m31, M31),
    jpl_get(Matrix, m32, M32),
    jpl_get(Matrix, m33, M33),

    List = [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33].




list_to_vector3d(List, Vector) :-
    nonvar(List), var(Vector),
    jpl_new('javax.vecmath.Vector3d', List, Vector).


vector3d_to_list(Vector, List) :-

    var(List), nonvar(Vector),

    jpl_get(Vector, x, X),
    jpl_get(Vector, y, Y),
    jpl_get(Vector, z, Z),

    List = [X, Y, Z].

list_to_vector3f(List, Vector) :-
    nonvar(List), var(Vector),
    jpl_new('javax.vecmath.Vector3f', List, Vector).

%% angle_between_vector_normalized(+Vector1, +Vector2, -Angle) is det.
%
% calculates angle between two normalized vectors in radiants
%
% @param Vector1	normalized vector 1
% @param Vector1	normalized vector 2
% @param Angle		Angle between the vectors in radiants
%
angle_between_vector_normalized(Vector1, Vector2, Angle) :-
	vector3d_to_list(Vector1,[X1,Y1,Z1]),
	vector3d_to_list(Vector2,[X2,Y2,Z2]),
	Angle is acos(X1 * X2 + Y1 * Y2 + Z1 * Z2).


