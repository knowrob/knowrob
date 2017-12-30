/*
  Copyright (C) 2011 Sjoerd van den Dries, Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
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

:- module(tfjava,
    [
        tfjava_start_listener/0,   % need to start TF listener before looking up transforms
        tfjava_lookup_transform/4, % looks up transform between target and source frame
        tfjava_transform_point/5,  % find transforms within some target frame
        tfjava_transform_pose/5 
    ]).
/** <module> Prolog-predicates as wrappers around tfjava

  Provides an interface to the tf system from within Prolog, allowing
  to transform coordinates and to lookup transformations.
  
@author Sjoerd van den Dries
@author Moritz Tenorth
@license BSD
*/

:- use_module(library('jpl')).

% tf_listener/1 contains a handle to the tf listener
:- dynamic tf_listener/1.

%% tfjava_start_listener is det
%
% Starts the tf listener, which runs in a separate thread.
%
tfjava_start_listener :-
    not(tf_listener(_)),
    jpl_call('tfjava.TFListener', 'getInstance', [], Client),
    assert(tf_listener(Client)).

tfjava_listener(Client) :-
    tf_listener(Client), !.
tfjava_listener(Client) :-
    tfjava_start_listener,
    tf_listener(Client).

%% tfjava_lookup_transform(+TargetFrame, +SourceFrame, -Pose, +Instant) is nondet.
%
% Lookup transform of SourceFrame relative to TargetFrame at time Instant.
%
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param SourceFrame     Tf frame the source matrix is described in
% @param Pose            Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param Instant         Time point at which the transformation is to be determined
%
tfjava_lookup_transform(TargetFrame,
                        SourceFrame,
                        pose([X,Y,Z],[QX,QY,QZ,QW]),
                        Instant) :-
  tfjava_listener(Client),
  jpl_call(Client, 'lookupTransform', [TargetFrame, SourceFrame, Instant], TF),
  \+ jpl_null(TF),
  jpl_call(TF, 'getTranslation', [], Vector3d),
  jpl_call(Vector3d, 'x', [], X),
  jpl_call(Vector3d, 'y', [], Y),
  jpl_call(Vector3d, 'z', [], Z),
  jpl_call(TF, 'getRotation', [], Quat4d),
  jpl_call(Quat4d, 'x', [], QX),
  jpl_call(Quat4d, 'y', [], QY),
  jpl_call(Quat4d, 'z', [], QZ),
  jpl_call(Quat4d, 'w', [], QW).

%% tfjava_transform_point(+SourceFrame, +TargetFrame, +PointIn, -PointOut, -Instant) is nondet
%
% Transform a point into TargetFrame at time Instant.
%
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PointIn         List of form [float x,y,z]
% @param PointOut        List of form [float x,y,z]
% @param Instant         Time point at which the transformation is to be determined
%
tfjava_transform_point(SourceFrame,
                       TargetFrame,
                       PointIn,
                       [X_Out,Y_Out,Z_Out],
                       Instant) :-
  tfjava_listener(Client),
  % create StampedPtIn
  jpl_list_to_array(PointIn, PointInArray),
  jpl_call('tfjava.Utils', 'pointArrayToStampedPoint3d',
          [PointInArray, SourceFrame, Instant], StampedPtIn),
  % create StampedPtOut
  jpl_call('tfjava.Utils', 'getStampedPoint3d', [], StampedPtOut),
  % transform the point
  jpl_call(Client, 'transformPoint',
          [TargetFrame, StampedPtIn, StampedPtOut], _),
  jpl_get(StampedPtOut, 'data', Data),
  jpl_get(Data, 'x', X_Out),
  jpl_get(Data, 'y', Y_Out),
  jpl_get(Data, 'z', Z_Out).


%% tfjava_transform_pose(+SourceFrame, +TargetFrame, +PoseIn, -PoseOut, -Instant) is nondet
%
% Transform a pose into TargetFrame at time Instant.
%
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PoseIn          Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param PoseOut         Term of form pose([float x,y,z],[float qx,qy,qz,qw])
% @param Instant         Time point at which the transformation is to be determined
%
tfjava_transform_pose(SourceFrame,
                      TargetFrame,
                      pose(PosIn,RotIn),
                      pose(PosOut,RotOut),
                      Instant) :-
  tfjava_listener(Client),
  % create StampedMatIn
  jpl_list_to_array(PosIn,PosInArr),
  jpl_list_to_array(RotIn,RotInArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrix',
          [PosInArr,RotInArr], PoseInArray),
  jpl_call('tfjava.Utils', 'poseArrayToStampedMatrix4d',
          [PoseInArray, SourceFrame, Instant], StampedMatIn),
  % create StampedMatOut
  jpl_call('tfjava.Utils', 'getStampedIdentityMatrix4d',
          [], StampedMatOut),
  % transform the pose
  jpl_call(Client, 'transformPose',
          [TargetFrame, StampedMatIn, StampedMatOut], _),
  % read position and orientation
  jpl_call('tfjava.Utils', 'stampedMatrix4dToPoseArray',
          [StampedMatOut], PoseOutArray),
  jpl_array_to_list(PoseOutArray, PoseOut),
  matrix(PoseOut,PosOut,RotOut).
