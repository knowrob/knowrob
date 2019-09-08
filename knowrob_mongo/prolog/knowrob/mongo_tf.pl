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
      mng_transform_pose/5
    ]).
/** <module> Looking up tf transforms in a mongo DB

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/mongo')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:-  rdf_meta
    mng_lookup_transform(+,+,-,+),
    mng_transform_pose(+,+,+,-,+),
    mng_comp_pose(r,r,+).

%% mng_lookup_transform(+TargetFrame, +SourceFrame, -Pose, +Time) is nondet.
%
% Determine the transform from Source to Target at time Instant based on the logged
% tf data.
% 
% @param TargetFrame  Target frame ID
% @param SourceFrame  Source frame ID
% @param Pose         The pose in the form of pose([float x,y,z],[float qx,qy,qz,qw])
% @param Time         The time instant
%
mng_lookup_transform(TargetFrame, SourceFrame, pose([X,Y,Z],[QX,QY,QZ,QW]), Time) :-
  mng_interface(Mongo),
  jpl_call(Mongo, 'lookupTransform',
          [TargetFrame, SourceFrame, Time], StampedTransform),
  \+ jpl_null(StampedTransform),
  jpl_call(StampedTransform, 'getTranslation', [], Vector3d),
  \+ jpl_null(Vector3d),
  jpl_get(Vector3d, 'x', X),
  jpl_get(Vector3d, 'y', Y),
  jpl_get(Vector3d, 'z', Z),
  jpl_call(StampedTransform, 'getRotation', [], Quat4d),
  jpl_get(Quat4d, 'x', QX),
  jpl_get(Quat4d, 'y', QY),
  jpl_get(Quat4d, 'z', QZ),
  jpl_get(Quat4d, 'w', QW).

%% mng_transform_pose(+SourceFrame, +TargetFrame, +PoseIn, -PoseOut, -Time) is nondet
%
% Transform a pose into TargetFrame at time Instant.
%
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PoseIn          Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param PoseOut         Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param Time            Time point at which the transformation is to be determined
%
mng_transform_pose(SourceFrame,
                   TargetFrame,
                   pose(PosIn,RotIn),
                   pose(PosOut,RotOut),
                   Time) :-
  mng_interface(Mongo),
  % create StampedMatIn
  jpl_list_to_array(PosIn,PosInArr),
  jpl_list_to_array(RotIn,RotInArr),
  matrix(PoseInArray,PosInArr,RotInArr),
  jpl_call('org.knowrob.utils.Utils', 'poseArrayToStampedMatrix4d',
          [PoseInArray, SourceFrame, Time], StampedMatIn),
  % create StampedMatOut
  jpl_call('org.knowrob.utils.Utils', 'getStampedIdentityMatrix4d',
          [], StampedMatOut),
  % transform the pose
  jpl_call(Mongo, 'transformPose',
          [TargetFrame, StampedMatIn, StampedMatOut], @(true)),
  % read position and orientation
  jpl_call('org.knowrob.utils.Utils', 'stampedMatrix4dToPoseArray',
          [StampedMatOut], PoseOutArray),
  jpl_array_to_list(PoseOutArray, PoseOut),
  matrix(PoseOut,PosOut,RotOut).
