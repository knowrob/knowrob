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
        mem_store_tf/2,
        comp_object_pose/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/comp/object_pose')).

:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/memory'), [mem_db_name/1]).

:-  rdf_meta
    comp_object_pose(r,?,+).

:- rdfs_computable
    comp_object_pose(knowrob:'pose').

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
  mem_db_name(DB),
  %
  findall(X, (
    member([T,_],Topics),
    atom_concat('/',T,X)
  ), TopicPaths),
  process_create(path('rosrun'),
      ['mongodb_log', 'mongodb_log.py',
       '--mongodb-name', DB|TopicPaths],
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
  %process_create(path('rosnode'), ['kill', 'mongodb_log'], []),
  process_kill(PID, int),
  retractall(ros_logger_pid(PID)),
  process_wait(PID, _),
  print_message(informational,
    process_stopped('mongodb_log',PID)).

%% mem_store_tf(+Pose,+Stamp)
%
% Stores a single TF message.
%
mem_store_tf([RefFrame,ObjFrame,[X,Y,Z],[QX,QY,QZ,QW]],Stamp) :-
  mem_db_name(DB),
  mng_store(DB, tf, [
    [child_frame_id,string(RefFrame)],
    [header, [
      [frame_id,string(ObjFrame)],
      [seq,int(0)],
      [stamp,time(Stamp)]
    ]],
    [pose,[
      [translation,[
        [x,double(X)],
        [y,double(Y)],
        [z,double(Z)]
      ]],
      [rotation,[
        [x,double(QX)],
        [y,double(QY)],
        [z,double(QZ)],
        [w,double(QW)]
      ]]
    ]]
  ]).

%% comp_object_pose(+Obj, ?Pose, +Time).
%
comp_object_pose(Subject, Pose, Time) :-
  ground(Subject),
  mem_db_name(DB),
  current_object_pose_stamp(Subject,X),
  X > Time,
  object_frame_name(Subject,ObjFrame),
  mng_tf_interpolate(DB,ObjFrame,Pose,Time),
  !.
