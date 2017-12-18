/** 

  Copyright (C) 2013 Moritz Tenorth, 2015 Daniel Beßler
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

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- module(knowrob_mongo_tf,
    [
      mng_lookup_transform/4,
      mng_lookup_position/4,
      mng_transform_pose/5,
      %mng_robot_pose/2,
      %mng_robot_pose/3,
      %mng_robot_pose_at_time/4,
      mng_comp_pose/2,
      mng_comp_pose/3,
      mng_comp_pose_at_time/4,
      comp_mng_pose/2,
      comp_mng_pose/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_mongo_interface')).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:-  rdf_meta
    mng_lookup_transform(+,+,r,-),
    mng_lookup_position(+,+,r,-),
    %mng_robot_pose(r, r),
    %mng_robot_pose(r, r,r),
    %mng_robot_pose_at_time(r, +, r, r),
    mng_comp_pose(r, r),
    mng_comp_pose(r, r,r),
    mng_comp_pose_at_time(r, +, r, r),
    comp_mng_pose(r,-),
    comp_mng_pose_at_time(r,-,+).

%% mng_lookup_transform(+Target, +Source, +TimePoint, -Transform) is nondet.
%
% Determine the transform from Source to Target at TimePoint based on the logged
% tf data.
% 
% @param Target     Target frame ID
% @param Source     Source frame ID
% @param TimePoint  Instance of knowrob:TimePoint
% @param Transform  Transformation matrix as list[16]
%
mng_lookup_transform(Target, Source, TimePoint, Transform) :-
  atom(TimePoint), time_term(TimePoint, T),
  mng_lookup_transform(Target, Source, T, Transform).

mng_lookup_transform(Target, Source, TimePoint, Transform) :-
  number(TimePoint),

  mongo_interface(DB),
  jpl_call(DB, 'lookupTransform', [Target, Source, TimePoint], StampedTransform),
  % Make sure transform is not null!
  not( jpl_null(StampedTransform) ),

  jpl_call(StampedTransform, 'getMatrix4', [], TransformMatrix4d),
  knowrob_coordinates:matrix4d_to_list(TransformMatrix4d, Transform).

%% mng_lookup_position(+Target, +Source, +TimePoint, -Position) is nondet.
%
% Determine the position from Source to Target at TimePoint based on the logged
% tf data.
% 
% @param Target     Target frame ID
% @param Source     Source frame ID
% @param TimePoint  Instance of knowrob:TimePoint
% @param Position   Position as list[3]
%
mng_lookup_position(Target, Source, TimePoint, Position) :-
  mng_lookup_transform(Target, Source, TimePoint, Transform),
  nth0( 3, Transform, X),
  nth0( 7, Transform, Y),
  nth0(11, Transform, Z),
  Position = [ X, Y, Z ].

%% mng_transform_pose(+PoseListIn, +SourceFrame, +TargetFrame, +TimePoint, -PoseListOut) is nondet.
% 
% Transform PoseListIn from SourceFrame into TargetFrame based on the logged tf data.
% 
% @param PoseListIn    Pose matrix in SourceFrame to be transformed into TargetFrame, as row-based list[16]
% @param SourceFrame   Source frame ID
% @param TargetFrame   Target frame ID
% @param TimePoint     Instance of knowrob:TimePoint
% @param PoseListOut   Pose matrix as row-based list[16]
%
mng_transform_pose(PoseListIn, SourceFrame, TargetFrame, TimePoint, PoseListOut) :-
  
  time_term(TimePoint, Time),
  number(Time),
  TimeInt is round(Time),

  knowrob_coordinates:list_to_matrix4d(PoseListIn, MatrixIn),
  jpl_new('tfjava.Stamped', [MatrixIn, SourceFrame, TimeInt], StampedIn),

  knowrob_coordinates:list_to_matrix4d([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1], MatrixOut),
  % create intermediate matrix 
  jpl_new('tfjava.Stamped', [MatrixOut, '/', TimeInt], StampedOut),

  mongo_interface(DB),
  jpl_call(DB, 'transformPose', [TargetFrame, StampedIn, StampedOut], @(true)),
  
  jpl_call(StampedOut, 'getData', [], MatrixOut2),
  knowrob_coordinates:matrix4d_to_list(MatrixOut2, PoseListOut).

%% mng_robot_pose(+Robot, -Pose) is nondet.
%
% Compute the pose of all components of the robot at the current point in time.
%
% @param Robot        Instance of a robot in SRDL
% @param Pose         Instance of a knowrob:RotationMatrix3D with the pose data
%
%mng_robot_pose(Robot, Pose) :-
  %mng_robot_pose(Robot, Pose, 'map').
  
%mng_robot_pose(Robot, Pose, Target) :-
  %get_timepoint(TimePoint),
  %mng_robot_pose_at_time(Robot, Target, TimePoint, Pose).


%% mng_robot_pose_at_time(Robot, TargetFrame, TimePoint, Pose) is nondet.
%
% Compute the pose of all components of the robot at the given point in time.
%
% @param Robot        Instance of a robot in SRDL
% @param TargetFrame  Atom with tf frame ID in which the pose shall be returned (e.g. '/map')
% @param TimePoint    Instance of knowrob:TimePoint
% @param Pose         Instance of a knowrob:RotationMatrix3D with the pose data
%
%mng_robot_pose_at_time(Robot, TargetFrame, TimePoint, Pose) :-
  %findall(S, (sub_component(Robot, S),
              %owl_individual_of(S, srdl2comp:'UrdfLink')), Ss),

  %sort(Ss, Ssorted),
  %findall(P, (member(Sub, Ssorted),mng_comp_pose_at_time(Sub, TargetFrame, TimePoint, P)), Ps),

  %nth0(0, Ps, Pose).



%% mng_comp_pose(+RobotPart, -Pose) is nondet.
%
% Read the pose of RobotPart in /map coordinates from logged tf data, default to 'now'
%
% @param RobotPart  Instance of a robot part with the 'urdfName' property set
% @param Pose       Instance of a knowrob:RotationMatrix3D with the pose data
%
mng_comp_pose(RobotPart, Pose) :-
  mng_comp_pose(RobotPart,  Pose , '/map' ).

mng_comp_pose(RobotPart, Pose, Target) :-
  get_timepoint(TimePoint),
  mng_comp_pose_at_time(RobotPart, Target, TimePoint, Pose).

  
%% mng_comp_pose_at_time(+RobotPart, +TargetFrame, +TimePoint, -Pose) is nondet.
%
% Read the pose of RobotPart in the given coordinate frame from logged tf data
%
% @param RobotPart    Instance of a robot part with the 'urdfName' property set
% @param TargetFrame  Atom with tf frame ID in which the pose shall be returned (e.g. '/map')
% @param TimePoint    Instance of knowrob:TimePoint
% @param Pose         Instance of a knowrob:RotationMatrix3D with the pose data
%
mng_comp_pose_at_time(RobotPart, TargetFrame, TimePoint, Pose) :-

  owl_has(RobotPart, 'http://knowrob.org/kb/knowrob.owl#frameName', literal(SourceFrameID)),
  ( atom_prefix(SourceFrameID,'/') ->
    SourceResolved = SourceFrameID      
    ; atom_concat('/',SourceFrameID, SourceResolved) 
  ),    
  ( robot_part_tf_prefix(RobotPart, TfPrefix) ->
    ( atom_prefix(TfPrefix,'/') ->
      ( TfPrefix == '/' ->
    TfResolved = ''
    ;TfResolved = TfPrefix      
      )
      ; atom_concat('/',TfPrefix, TfResolved) 
    ),
    atom_concat(TfResolved, SourceResolved,SourceFrame)
    ;SourceFrame = SourceResolved
  ),
  
  %%FIXME @Bender this should be replaced with the tfPrefix SPEED THIS UP
  mng_obj_pose_at_time(RobotPart, SourceFrame, TargetFrame, TimePoint, Pose).

  
  

%% mng_obj_pose_at_time(+Obj, +SourceFrame, +TargetFrame, +TimePoint, -Pose) is nondet.
%
% Read the pose of Obj and transform it into the coordinates given by
% TargetFrame  based on logged tf data
%
% @param Obj          Object instance
% @param SourceFrame  Atom with tf frame ID in what the object's pose is given
% @param TargetFrame  Atom with tf frame ID in which the pose shall be returned (e.g. '/map')
% @param TimePoint    Instance of knowrob:TimePoint
% @param Pose         Instance of a knowrob:RotationMatrix3D with the pose data
%
% @deprecated
mng_obj_pose_at_time(Obj, SourceFrame, TargetFrame, TimePoint, Pose) :-

  % read object pose in original coordinates at TimePoint
  % MT: deactivated since, when called the second time, this will return different
  %     results because the pose is asserted below
%   (object_pose_at_time(Obj, TimePoint, PoseListIn)
%      -> true ;
        PoseListIn = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
%         ),

  mng_transform_pose(PoseListIn, SourceFrame, TargetFrame, TimePoint, PoseListOut),
  create_pose(mat(PoseListOut), Pose),
  rdf_assert(Pose, 'http://knowrob.org/kb/knowrob.owl#frameName', TargetFrame),

  rdf_instance_from_class('http://knowrob.org/kb/knowrob.owl#Proprioception', Perception),
  
  ( number(TimePoint) ->
    create_timepoint(TimePoint, TimePoint_) ;
    TimePoint_ = TimePoint ),
  rdf_assert(Perception, knowrob:startTime, TimePoint_),

  set_object_perception(Obj, Perception),
  rdf_assert(Perception, knowrob:eventOccursAt, Pose).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% hook mongo TF data into computable property knowrob:pose by expanding `holds`

%% comp_mng_pose
comp_mng_pose(Obj, Pose) :-
  get_timepoint(Instant),
  comp_mng_pose_at_time(Obj, Pose, Instant).

%% comp_mng_pose_at_time
comp_mng_pose_at_time(Obj, Pose, Instant) :-
  nonvar(Obj),
  past_instant(Instant),
  mng_object_pose_at_time(Obj, Instant, MngPose, Instant),
  object_pose(MngPose, Instant, PoseTerm),
  create_pose(PoseTerm, Pose).

%% comp_mng_pose_during
% TODO(daniel): support interval queries: yield all different poses in given time interval.
%               smart mongo query with stepwise computation could nicely hook into Prolog backtracking.
comp_mng_pose_during(Obj, Pose, [Instant,Instant]) :-
  comp_mng_pose_during(Obj, Pose, Instant).

knowrob_temporal:holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', Pose, Interval) :- comp_mng_pose_during(Obj, Pose, Instant).

past_instant(Instant) :-
  get_timepoint(Now),
  % TODO(daniel): how far can we "travel back" with c++ TF listener?
  Now > Instant + 60.0.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob_owl entity descriptions

% TODO(daniel): howto unit test?
% ?- register_ros_package(knowrob_mongo).
% ?- mng_db('Pick-and-Place_pr2-general-pick-and-place_0').
% ?- entity(Pose, [a, pose, [urdf_name, '/laser_tilt_mount_link'], [temporal_extend, [a, timepoint, 1396512420.0]]]).

knowrob_owl:entity_compute(Entity, [a,pose|Descr]) :-
  entity_has(Descr, urdf_name, UrdfName),
  entity_has(Descr, temporal_extend, IntervalDescr),
  % TODO(daniel): allow specification of reference frame
  ReferenceFrame='http://knowrob.org/kb/knowrob.owl#MapFrame',
  TFMapFrame='/map', % FIXME bad assumption
  pose_compute(Pose, TFMapFrame, UrdfName, IntervalDescr),
  matrix_rotation(Pose, [QW,QX,QY,QZ]),
  matrix_translation(Pose, [X,Y,Z]),
  create_pose(pose(ReferenceFrame, [X,Y,Z], [QW,QX,QY,QZ]), Entity).

pose_compute(Pose, SourceFrame, UrdfName, [a,timepoint|Descr]) :-
  entity(TimeIri, [a,timepoint|Descr]),
  time_term(TimeIri, Time),
  mng_lookup_transform(SourceFrame, UrdfName, Time, Pose), !.

% FIXME: support looking up all poses that occur during given interval
%pose_compute(Pose, SourceFrame, UrdfName, [an|[interval|Descr]]) :-
%  entity(TimeIri, Descr), time_term(TimeIri, Interval),
%  mng_lookup_transform(SourceFrame, UrdfName, Interval, Pose).

% TODO: trajectory entity_compute
%knowrob_owl:entity_compute(Entity, [a|[trajectory|Descr]]) :-
