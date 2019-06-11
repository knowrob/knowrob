/*
  Copyright (C) 2019 Daniel Beßler
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

:- module(ros_memory,
    [
        ros_logger_start/1,
        ros_logger_stop/0,
        ros_store_tf/2,
        comp_object_pose/3,
        comp_object_pose/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/objects')).

:-  rdf_meta
    comp_object_pose(r,?,+),
    comp_object_pose(r,?).

:- dynamic ros_logger_pid/1.

%% ros_logger_start(+Topics)
%
% Start ROS message logging.
% Only listen to given list of topics.
%
ros_logger_start(Topics) :-
  % stop any still running instance,
  ( ros_logger_pid(_) ->
    ros_logger_stop ; true ),
  mng_db(DBName),
  %
  findall(X, (
    member([T,_],Topics),
    atom_concat('/',T,X)
  ), TopicPaths),
  process_create(path('rosrun'),
      ['mongodb_log', 'mongodb_log.py',
       '--mongodb-name', DBName|TopicPaths],
      [process(PID)]),
  asserta(ros_logger_pid(PID)),
  %
  print_message(informational,
    process_running('mongodb_log',PID)).

%% ros_logger_stop
%
% Stop previously started ROS message logging.
%
ros_logger_stop :-
  \+ ros_logger_pid(_), !.

ros_logger_stop :-
  ros_logger_pid(PID),
  process_kill(PID, int),
  retractall(ros_logger_pid(PID)),
  process_wait(PID, _),
  print_message(informational,
    process_stopped('mongodb_log',PID)).

%% ros_store_tfros_store_tf(+Pose,+Stamp)
%
% Stores a single TF message.
%
ros_store_tf([RefFrame,ObjFrame,[X,Y,Z],[QX,QY,QZ,QW]],Stamp) :-
  Stamp_ms is Stamp * 1000.0,
%t.secs = (int)posix_ts;
%t.nsecs = (int) (1E9 * (posix_ts - ((int) posix_ts)));
  mng_store(tf, _{
    child_frame_id: RefFrame,
    header: _{
      frame_id: ObjFrame,
      seq: 0,
      stamp: Stamp_ms % FIXME: need a date type
      %stamp: _{
        %secs: XXX,
        %nsecs: YYYY
      %}
    },
    pose: _{
      translation: _{x:X,y:Y,z:Z},
      rotation:    _{x:QX,y:QY,z:QZ,w:QW}
    }
  }).

%% comp_object_pose(+Obj, ?Pose).
%% comp_object_pose(+Obj, ?Pose, +Interval).
%
% Compute pose from memory.
%
comp_object_pose(Obj, Pose) :-
  ground(Obj),
  current_object_pose(Obj,Pose),!.

comp_object_pose(Obj, Pose, [Stamp,Stamp]) :-
  ground(Obj),
  current_object_pose_stamp(X), X < Stamp,
  current_object_pose(Obj,Pose),!.

comp_object_pose(Obj, [RefFrame,ObjFrame,T,Q], [Stamp,Stamp]) :-
  ground(Obj),
  object_frame_name(Obj,ObjFrame),
  ( ground(RefFrame) ; map_frame_name(RefFrame) ),
  mng_lookup_transform(RefFrame,ObjFrame,pose(T,Q),Stamp),!.
