/** <module> tf_prolog

  Description:
    Prolog-predicates as wrappers around the tfjava methods

    Provide an interface to the tf system from within Prolog, allowing
    to transform coordinates and to lookup transformations.

  Copyright (C) 2011 by Sjoerd v.d. Dries, Moritz Tenorth
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Intelligent Autonomous Systems Group/
       Technische Universitaet Muenchen nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

@author Sjoerd v.d. Dries, Moritz Tenorth
@license BSD
*/

:- module(tf_prolog,
    [
        start_tf_listener/0,

        lookup_transform/4,
        lookup_transform_prolog/4,
        lookup_transform/6,
        lookup_transform_prolog/6,

        transform_point/3,
        transform_point/5,
        transform_point_prolog/4,
        transform_point_prolog/6,

        transform_pose/3,
        transform_pose/5,
        transform_pose_prolog/4,
        transform_pose_prolog/6,

        tf_transform_to_prolog/2,
        prolog_to_tf_transform/2,

        stamped_matrix_to_pose_list/4,
        pose_list_to_stamped_matrix/4,

        stamped_point_to_point_list/4,
        point_list_to_stamped_point/4,

        secs_to_rostime/2,
        rostime_to_secs/2,

        time_now/1
    ]).

:- use_module(library('jpl')).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% TF listener interface
%


% tf_listener/1 contains a handle to the tf listener
:- dynamic tf_listener/1.


%% start_tf_listener is det.
%
% starts the tf listener, which runs in a separate thread
%
start_tf_listener :-
    not(tf_listener(_)),
    jpl_call('tfjava.TFListener', 'getInstance', [], Client),
    assert(tf_listener(Client)).

% do not automatically start tf listener
% :- start_tf_listener.





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Lookup transformations
%


%% lookup_transform(+TargetFrame, +SourceFrame, +TimeSecs, -TF) is nondet.
%
% lookup transform for the time TimeSecs
%
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param SourceFrame     Tf frame the source matrix is described in
% @param TimeSecs        Time point at which the transformation is to be determined
% @param TF              Reference to a StampedTransform object
%
lookup_transform(TargetFrame, SourceFrame, TimeSecs, TF) :-
    tf_listener(Client),
    secs_to_rostime(TimeSecs, TimeRos),
    jpl_call(Client, 'lookupTransform', [TargetFrame, SourceFrame, TimeRos], TF).


%% lookup_transform(+TargetFrame, +TargetTimeSecs, +SourceFrame, +SourceTimeSecs, +FixedFrame, -TF) is nondet.
%
% lookup transform with time traveling
%
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param TargetTimeSecs  Time point at which the transformation of the target frame is to be determined
% @param SourceFrame     Tf frame the source matrix is described in
% @param SourceTimeSecs  Time point at which the transformation of the source frame is to be determined
% @param FixedFrame      Fixed frame that does not change over the course of the time traveling
% @param TF              Reference to a StampedTransform object
%
lookup_transform(TargetFrame, TargetTimeSecs, SourceFrame, SourceTimeSecs, FixedFrame, TF) :-
    tf_listener(Client),
    secs_to_rostime(TargetTimeSecs, TargetTimeRos),
    secs_to_rostime(SourceTimeSecs, SourceTimeRos),
    jpl_call(Client, 'lookupTransform', [TargetFrame, TargetTimeRos, SourceFrame, SourceTimeRos, FixedFrame], TF).


%% lookup_transform_prolog(+TargetFrame, +SourceFrame, +TimeSecs, -TFProlog) is nondet.
%
% Lookup transform for the current time from Prolog: Transformation as tf(FrameID, ChildFrameID, Secs, PoseList)
%
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param SourceFrame     Tf frame the source matrix is described in
% @param TimeSecs        Time point at which the transformation is to be determined
% @param TFProlog        Prolog structure of the form tf(FrameID, ChildFrameID, Secs, PoseList)
%
lookup_transform_prolog(TargetFrame, SourceFrame, TimeSecs, TFProlog) :-
    lookup_transform(TargetFrame, SourceFrame, TimeSecs, TF),
    tf_transform_to_prolog(TF, TFProlog).


%% lookup_transform_prolog(+TargetFrame, +TargetTimeSecs, +SourceFrame, +SourceTimeSecs, +FixedFrame, -TFProlog) is nondet.
%
% Lookup transform with time traveling from Prolog: Transformation as tf(FrameID, ChildFrameID, Secs, PoseList)
%
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param TargetTimeSecs  Time point at which the transformation of the target frame is to be determined
% @param SourceFrame     Tf frame the source matrix is described in
% @param SourceTimeSecs  Time point at which the transformation of the source frame is to be determined
% @param FixedFrame      Fixed frame that does not change over the course of the time traveling
% @param TFProlog        Prolog structure of the form tf(FrameID, ChildFrameID, Secs, PoseList)
%
lookup_transform_prolog(TargetFrame, TargetTimeSecs, SourceFrame, SourceTimeSecs, FixedFrame, TFProlog) :-
    lookup_transform(TargetFrame, TargetTimeSecs, SourceFrame, SourceTimeSecs, FixedFrame, TF),
    tf_transform_to_prolog(TF, TFProlog).







% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Transform points
%



%% transform_point(+StampedPtIn, +TargetFrame, -StampedMatOut) is nondet.
%
% transform a point into the TargetFrame
%
% @param StampedPtIn    Reference to a Stamped<Point3d> containing the input point
% @param TargetFrame    Tf frame the source matrix is to be transformed into
% @param StampedPtOut   Reference to a Stamped<Point3d> containing the transformed point
%
transform_point(StampedPtIn, TargetFrame, StampedPtOut) :-

    tf_listener(Client),
    jpl_call(Client, 'transformPoint', [TargetFrame, StampedPtIn, StampedPtOut], _).


%% transform_point(+StampedPtIn, +FixedFrame, +TargetFrame, +TargetTimeSecs, -StampedPtOut) is nondet.
%
% Transform a point into the TargetFrame with time traveling
%
% @param StampedPtIn    Reference to a Stamped<Point3d> containing the input point
% @param TargetFrame    Tf frame the source matrix is to be transformed into
% @param StampedPtOut   Reference to a Stamped<Point3d> containing the transformed point
%
transform_point(StampedPtIn, FixedFrame, TargetFrame, TargetTimeSecs, StampedPtOut) :-

    tf_listener(Client),
    secs_to_rostime(TargetTimeSecs, TargetTimeRos),
    jpl_call(Client, 'transformPoint', [TargetFrame, TargetTimeRos, StampedPtIn, FixedFrame, StampedPtOut], _).


%% transform_point_prolog(+PointListIn, +SourceFrame, +TargetFrame, -PointListOut) is nondet.
%
% point transformation from Prolog: point as [x,y,z]
%
% @param PointListIn     Point to be transformed as list of point coordinates [x,y,z]
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PointListOut    Transformed point as list of point coordinates [x,y,z]
%
transform_point_prolog(PointListIn, SourceFrame, TargetFrame, PointListOut) :-

    time_now(Now),
    point_list_to_stamped_point(PointListIn, SourceFrame, Now, StampedPtIn),
    jpl_call('tfjava.Utils', 'getStampedPoint3d', [], StampedPtOut),

    transform_point(StampedPtIn, TargetFrame, StampedPtOut),
    stamped_point_to_point_list(StampedPtOut, PointListOut, _, _).


%% transform_point_prolog(+PointListIn, +SourceFrame, +FixedFrame, +TargetFrame, +TargetTimeSecs, -PointListOut) is nondet.
%
% point transformation from Prolog including time traveling: point as [x,y,z]
%
% @param PointListIn     Point to be transformed as list of point coordinates [x,y,z]
% @param SourceFrame     Tf frame the source matrix is described in
% @param FixedFrame      Fixed frame that does not change over the course of the time traveling
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param TargetTimeSecs  Time in seconds at which the transformation is to be carried out
% @param PointListOut    Transformed point as list of point coordinates [x,y,z]
%
transform_point_prolog(PointListIn, SourceFrame, FixedFrame, TargetFrame, TargetTimeSecs, PointListOut) :-

    time_now(Now),
    point_list_to_stamped_point(PointListIn, SourceFrame, Now, StampedPtIn),
    jpl_call('tfjava.Utils', 'getStampedPoint3d', [], StampedPtOut),
    transform_point(StampedPtIn, FixedFrame, TargetFrame, TargetTimeSecs, StampedPtOut),
    stamped_point_to_point_list(StampedPtOut, PointListOut, _, _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Transform poses
%


%% transform_pose(+StampedMatIn, +TargetFrame, -StampedMatOut) is nondet.
%
% Transform a pose into the TargetFrame
%
% @param StampedMatIn    Reference to a Stamped<Matrix4d> containing the input pose
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param StampedMatOut   Reference to a Stamped<Matrix4d> containing the transformed pose
%
transform_pose(StampedMatIn, TargetFrame, StampedMatOut) :-

    tf_listener(Client),
    jpl_call(Client, 'transformPose', [TargetFrame, StampedMatIn, StampedMatOut], _).


%% transform_pose(+StampedMatIn, +FixedFrame, +TargetFrame, +TargetTimeSecs, -StampedMatOut) is nondet
%
% Transform a pose into the TargetFrame with time traveling
%
% @param StampedMatIn    Reference to a Stamped<Matrix4d> containing the input pose
% @param FixedFrame      Fixed frame that does not change over the course of the time traveling
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param TargetTimeSecs  Time in seconds at which the transformation is to be carried out
% @param StampedMatOut   Reference to a Stamped<Matrix4d> containing the transformed pose
%
transform_pose(StampedMatIn, FixedFrame, TargetFrame, TargetTimeSecs, StampedMatOut) :-

    tf_listener(Client),
    secs_to_rostime(TargetTimeSecs, TargetTimeRos),

    jpl_call(Client, 'transformPose', [TargetFrame, TargetTimeRos, StampedMatIn, FixedFrame, StampedMatOut], _).


%% transform_pose_prolog(+PoseListIn, +SourceFrame, +TargetFrame, -PoseListOut) is nondet.
%
% Pose transformation from Prolog for the current time
%
% @param PoseListIn      Pose matrix to be transformed in row-based representation [m00, m01, m02, m03, m11, ...]
% @param SourceFrame     Tf frame the source matrix is described in
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param PoseListOut     Transformed pose list
%
transform_pose_prolog(PoseListIn, SourceFrame, TargetFrame, PoseListOut) :-

    time_now(Now),
    pose_list_to_stamped_matrix(PoseListIn, SourceFrame, Now, StampedMatIn),
    jpl_call('tfjava.Utils', 'getStampedIdentityMatrix4d', [], StampedMatOut),
    transform_pose(StampedMatIn, TargetFrame, StampedMatOut),
    stamped_matrix_to_pose_list(StampedMatOut, PoseListOut, _, _).


%% transform_pose_prolog(+PoseListIn, +SourceFrame, +FixedFrame, +TargetFrame, +TargetTimeSecs, -PoseListOut) is nondet.
%
% Transform a pose into the TargetFrame with time traveling
%
% @param PoseListIn      Pose matrix to be transformed in row-based representation [m00, m01, m02, m03, m11, ...]
% @param SourceFrame     Tf frame the source matrix is described in
% @param FixedFrame      Fixed frame that does not change over the course of the time traveling
% @param TargetFrame     Tf frame the source matrix is to be transformed into
% @param TargetTimeSecs  Time in seconds at which the transformation is to be carried out
% @param PoseListOut     Transformed pose list
%
transform_pose_prolog(PoseListIn, SourceFrame, FixedFrame, TargetFrame, TargetTimeSecs, PoseListOut) :-

    time_now(Now),
    pose_list_to_stamped_matrix(PoseListIn, SourceFrame, Now, StampedMatIn),
    jpl_call('tfjava.Utils', 'getStampedIdentityMatrix4d', [], StampedMatOut),
    transform_pose(StampedMatIn, FixedFrame, TargetFrame, TargetTimeSecs, StampedMatOut),
    stamped_matrix_to_pose_list(StampedMatOut, PoseListOut, _, _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Utility predicates
%



%% tf_transform_to_prolog(+TF, -TFProlog) is det.
%
% convert tf transform object into a Prolog structure of the form
% tf(FrameID, ChildFrameID, Secs, PoseList)
%
% @param TF              Reference to a StampedTransform object
% @param TFProlog Prolog structure of the form tf(FrameID, ChildFrameID, Secs, PoseList)
%
tf_transform_to_prolog(TF, TFProlog) :-

    nonvar(TF), var(TFProlog),
    not(jpl_is_null(TF)),

    jpl_get(TF, 'frameID', FrameID),
    jpl_get(TF, 'childFrameID', ChildFrameID),
    jpl_get(TF, 'timeStamp', TimeRos),

    rostime_to_secs(TimeRos, Secs),

    jpl_call('tfjava.Utils', 'tfToPoseArray', [TF], PoseArray),
    jpl_array_to_list(PoseArray, PoseList),

    TFProlog = tf(FrameID, ChildFrameID, Secs, PoseList).


%% prolog_to_tf_transform(+TFProlog, -TF) is det.
%
% convert Prolog structure of the form tf(FrameID, ChildFrameID, Secs, PoseList)
% into a tf transform object
%
% @param TFProlog Prolog structure of the form tf(FrameID, ChildFrameID, Secs, PoseList)
% @param TF              Reference to a StampedTransform object
%
prolog_to_tf_transform(TFProlog, TF) :- % create TF from Prolog structure

    nonvar(TF), var(TFProlog),
    TFProlog = tf(FrameID, ChildFrameID, Secs, PoseList),

    jpl_list_to_array(PoseList, PoseArray),
    jpl_new('javax.vecmath.Matrix4d', [PoseArray], PoseMatrix),

    secs_to_rostime(Secs, TimeRos),
    jpl_new('tfjava.StampedTransform', [PoseMatrix, TimeRos, FrameID, ChildFrameID], TF).


%% stamped_matrix_to_pose_list(+StMatrix, -PoseList, -FrameID, -TimeSecs) is det.
%
% read information from a Stamped<Matrix4d> datastructure into Prolog
%
% @param StMatrix Reference to a Stamped<Matrix4d>
% @param PoseList Row-based matrix representation [m00, m01, m02, m03, m11, ...]
% @param FrameID  ID of the tf frame in which StMatrix is described
% @param TimeSecs Time stamp of StMatrix in seconds
%
stamped_matrix_to_pose_list(StMatrix, PoseList, FrameID, TimeSecs) :-

    nonvar(StMatrix), var(PoseList),

    jpl_call('tfjava.Utils', 'stampedMatrix4dToPoseArray', [StMatrix], PoseArray),
    jpl_array_to_list(PoseArray, PoseList),

    jpl_get(StMatrix, 'frameID', FrameID),
    jpl_get(StMatrix, 'timeStamp', TimeRos),

    rostime_to_secs(TimeRos, TimeSecs).


%% pose_list_to_stamped_matrix(+PoseList, +FrameID, +TimeSecs, -StMatrix) is det.
%
% create Stamped<Matrix4d> datastructure
%
% @param StMatrix Reference to a Stamped<Matrix4d>
% @param PoseList Row-based matrix representation [m00, m01, m02, m03, m11, ...]
% @param FrameID  ID of the tf frame in which StMatrix is described
% @param TimeSecs Time stamp of StMatrix in seconds
%
pose_list_to_stamped_matrix(PoseList, FrameID, TimeSecs, StMatrix) :-

    var(StMatrix), nonvar(PoseList),
    jpl_list_to_array(PoseList, PoseArray),
    jpl_call('tfjava.Utils', 'poseArrayToStampedMatrix4d', [PoseArray, FrameID, TimeSecs], StMatrix).


%% stamped_point_to_point_list(+StPoint, -PointList, -FrameID, -TimeSecs) is det.
%
% read information from a Stamped<Point3d> datastructure into Prolog
%
% @param StPoint Reference to a Stamped<Point3d>
% @param PointList List of point coordinates [x,y,z]
% @param FrameID  ID of the tf frame in which StPoint is described
% @param TimeSecs Time stamp of StPoint in seconds
%
stamped_point_to_point_list(StPoint, PointList, FrameID, TimeSecs) :-

    nonvar(StPoint), var(PointList),

    jpl_call('tfjava.Utils', 'stampedPoint3dToPointArray', [StPoint], PointArray),
    jpl_array_to_list(PointArray, PointList),

    jpl_get(StPoint, 'frameID', FrameID),
    jpl_get(StPoint, 'timeStamp', TimeRos),

    rostime_to_secs(TimeRos, TimeSecs).


%% point_list_to_stamped_point(+PointList, +FrameID, +TimeSecs, -StPoint) is det.
%
% create Stamped<Point3d> datastructure
%
% @param PointList List of point coordinates [x,y,z]
% @param FrameID  ID of the tf frame in which StPoint is described
% @param TimeSecs Time stamp of StPoint in seconds
% @param StPoint Reference to a Stamped<Point3d>
%
point_list_to_stamped_point(PointList, FrameID, TimeSecs, StPoint) :-

    var(StPoint), nonvar(PointList),
    jpl_list_to_array(PointList, PointArray),
    jpl_call('tfjava.Utils', 'pointArrayToStampedPoint3d', [PointArray, FrameID, TimeSecs], StPoint).


%% secs_to_rostime(+Secs, -TimeRos) is det.
%
% convert seconds to a ros.communication.Time object
%
% @param Secs    Numerical value: timestamp in seconds
% @param TimeRos Reference to a ros.communication.Time object
%
secs_to_rostime(Secs, TimeRos) :-
    jpl_call('tfjava.Utils', 'secondsToRosTime', [Secs], TimeRos).


%% rostime_to_secs(+TimeRos, -Secs) is det.
%
% convert a ros.communication.Time object into a numerical value in seconds
%
% @param TimeRos Reference to a ros.communication.Time object
% @param Secs    Numerical value: timestamp in seconds
%
rostime_to_secs(TimeRos, Secs) :-
    not(jpl_is_null(TimeRos)),
    jpl_call(TimeRos, 'totalNsecs', [], NSecs),
    Secs is NSecs / 1000000000.


%% time_now(-Secs) is det.
%
% get the current time in seconds
%
% @param Secs    Numerical value: timestamp in seconds
%
time_now(Secs) :-
    jpl_call('tfjava.Utils', 'getTimeNowSecs', [], Secs).

