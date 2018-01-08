/*
  Copyright (C) 2013 Moritz Tenorth
  Copyright (C) 2015 Daniel Beßler
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

:- module(mongo_tf,
    [
      mng_lookup_transform/4,
      mng_transform_pose/5,
      mng_comp_pose/3
    ]).
/** <module> Looking up tf transforms in a mongo DB

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/mongo')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:-  rdf_meta
    mng_lookup_transform(+,+,-,+),
    mng_transform_pose(+,+,+,-,+),
    mng_comp_pose(r,r,+).

%% mng_lookup_transform(+TargetFrame, +SourceFrame, -Pose, +Instant) is nondet.
%
% Determine the transform from Source to Target at time Instant based on the logged
% tf data.
% 
% @param TargetFrame  Target frame ID
% @param SourceFrame  Source frame ID
% @param Pose         The pose in the form of pose([float x,y,z],[float qx,qy,qz,qw])
% @param Instant      The time instant
%
mng_lookup_transform(TargetFrame, SourceFrame, pose([X,Y,Z],[QX,QY,QZ,QW]), Instant) :-
  mng_interface(Mongo),
  time_term(Instant, T),
  jpl_call(Mongo, 'lookupTransform',
          [TargetFrame, SourceFrame, T], StampedTransform),
  \+ jpl_null(StampedTransform),
  jpl_call(StampedTransform, 'getTranslation', [], Vector3d),
  jpl_call(Vector3d, 'x', [], X),
  jpl_call(Vector3d, 'y', [], Y),
  jpl_call(Vector3d, 'z', [], Z),
  jpl_call(StampedTransform, 'getRotation', [], Quat4d),
  jpl_call(Quat4d, 'x', [], QX),
  jpl_call(Quat4d, 'y', [], QY),
  jpl_call(Quat4d, 'z', [], QZ),
  jpl_call(Quat4d, 'w', [], QW).

%% mng_transform_pose(+SourceFrame, +TargetFrame, +PoseIn, -PoseOut, -Instant) is nondet
%
% Transform a pose into TargetFrame at time Instant.
%
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PoseIn          Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param PoseOut         Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param Instant         Time point at which the transformation is to be determined
%
mng_transform_pose(SourceFrame,
                   TargetFrame,
                   pose(PosIn,RotIn),
                   pose(PosOut,RotOut),
                   Instant) :-
  mng_interface(Mongo),
  time_term(Instant, T),
  % create StampedMatIn
  jpl_list_to_array(PosIn,PosInArr),
  jpl_list_to_array(RotIn,RotInArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrix',
          [PosInArr,RotInArr], PoseInArray),
  jpl_call('tfjava.Utils', 'poseArrayToStampedMatrix4d',
          [PoseInArray, SourceFrame, T], StampedMatIn),
  % create StampedMatOut
  jpl_call('tfjava.Utils', 'getStampedIdentityMatrix4d',
          [], StampedMatOut),
  % transform the pose
  jpl_call(Mongo, 'transformPose',
          [TargetFrame, StampedMatIn, StampedMatOut], @(true)),
  % read position and orientation
  jpl_call('tfjava.Utils', 'stampedMatrix4dToPoseArray',
          [StampedMatOut], PoseOutArray),
  jpl_array_to_list(PoseOutArray, PoseOut),
  matrix(PoseOut,PosOut,RotOut).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% hook mongo TF data into computable property knowrob:pose by expanding `holds`

%% mng_comp_pose(+Obj, -Pose, +Interval).
%
% Pose is an OWL individual of knowrob:'Pose' that represents logged tf data.
%
mng_comp_pose(Obj, Pose, [Instant,Instant]) :-
  nonvar(Obj), nonvar(Instant),
  % only compute poses for time instants in the past
  current_time(Now), Now > Instant + 20.0,
  map_frame_name(MapFrame),
  rdf_has(Obj, knowrob:frameName, ObjFrame),
  mng_lookup_transform(MapFrame, ObjFrame, PoseTerm, Instant),
  owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose).
