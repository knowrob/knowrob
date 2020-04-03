/*
  Copyright (C) 2011-2014 Moritz Tenorth
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

:- module('knowrob/comp/object_pose',
    [
      object_pose/2,
      object_pose/3,
      current_object_pose/2,
      current_object_pose_stamp/2,
      object_pose_update/2,
      object_pose_update/3,
      object_pose_data/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/lang/ask'), [
    kb_triple/3
]).
:- use_module(library('knowrob/lang/holds'), [
    holds/4
]).
:- use_module(library('knowrob/model/Object'),[
    object_frame_name/2,
    object_localization/2
]).
:- use_module(library('knowrob/comp/tf'), [
    transform_between/3,
    transform_multiply/3
]).
% FIXME where is tf_lookup_transform defined?

:-  rdf_meta
    current_object_pose(r,-),
    object_pose(r,+,r),
    object_pose(r,+),
    object_pose_update(r,+),
    object_pose_update(r,+,+),
    object_trajectory(r,t,+,-).

:- dynamic object_pose_data/3,
           current_object_pose_stamp/2.

%%
%% TODO: also assert Localization region to trigger
%%        mem to store it?
object_pose_update(Obj,Pose) :-
  get_time(Now),
  object_pose_update(Obj,Pose,Now).

object_pose_update(Obj,_,Stamp) :-
  object_pose_data(Obj,_,LatestStamp),
  LatestStamp >= Stamp, !.

object_pose_update(Obj,Pose,Stamp) :-
  ground(Pose),
  Pose = [_,_,_,_],
  %%
  (( current_object_pose_stamp(Obj,Latest), Latest >= Stamp ) -> true ; (
     retractall(current_object_pose_stamp(Obj,_)),
     asserta(current_object_pose_stamp(Obj,Stamp))
  )),
  %%
  retractall(object_pose_data(Obj,_,_)),
  asserta(object_pose_data(Obj,Pose,Stamp)), !.

object_pose_update(Obj,Pose,_) :-
  print_message(warning, pose_update_failed(Obj,Pose)),
  fail.

%% current_object_pose(+Obj:iri, -Pose:list) is semidet
%
% Get the current pose of an object.
%
% @param Obj   Instance of SpatialThing
% @param Pose  The pose term [atom Reference, atom Target, [float x,y,z], [float qx,qy,qz,qw]]
% 
current_object_pose(Obj,Pose) :- 
  object_pose_data(Obj,Pose,_),!.

current_object_pose(Obj, [RefFrame,ObjFrame,T,Q]) :- 
  object_frame_name(Obj,ObjFrame),
  object_localization(Obj,Loc),
  kb_triple(Loc, ease_obj:hasSpaceRegion, [ParentFrame,ObjFrame,T0,Q0]),
  ( RefFrame=ParentFrame ->
  ( T=T0, Q=Q0 ) ;
  ( current_map_pose_([ParentFrame,ObjFrame,T0,Q0], MapPose0),
    current_map_pose(RefFrame,MapPose1),
    transform_between(MapPose1,MapPose0,[RefFrame,ObjFrame,T,Q])
  )),!.

current_object_pose(Obj,[ParentFrame,ObjFrame,T,Q]) :-
  % try TF lookup in case above clauses failed.
  (ground(ParentFrame) ; map_frame_name(ParentFrame)),
  object_frame_name(Obj,ObjFrame),
  tf_lookup_transform(ParentFrame,ObjFrame,pose(T,Q)),!.

%%
current_map_pose(ObjFrame,MapPose) :-
  object_frame_name(Obj,ObjFrame),
  current_object_pose(Obj,CurrentPose),
  current_map_pose_(CurrentPose,MapPose).

current_map_pose_([ParentFrame,ObjFrame,T,Q],MapPose) :-
  map_frame_name(MapFrame),
  ( MapFrame=ParentFrame ->
  ( MapPose=[MapFrame,ObjFrame,T,Q] ) ;
  ( current_map_pose(ParentFrame,MapParent),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).
  
%% object_pose(+Obj:iri, ?Pose:term, +Instant:float) is semidet
%
% True if Pose is the pose of Obj at time Instant.
% Poses may be requested in particular reference frames
% by partially specifying Pose. For instance, to request the pose in camera frame:
%    Pose=['head_rgb_camera',ObjFrame,Position,Orientation]
%
% @param Obj     Instance of SpatialThing
% @param Instant The time instant (float or Timepoint:iri)
% @param Pose    The pose [atom Reference, atom Target, [float x,y,z], [float qx,qy,qz,qw]]
% 
object_pose(Obj, Pose) :-
  current_object_pose(Obj, Pose).

object_pose(Obj, Pose, Stamp0) :- 
  object_pose_data(Obj,Pose,Stamp1),
  Stamp1 < Stamp0, !.

object_pose(Obj, [RefFrame,ObjFrame,T,Q], Time) :- 
  object_frame_name(Obj,ObjFrame),
  object_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [ParentFrame,ObjFrame,T0,Q0], Time),
  ( RefFrame=ParentFrame ->
  ( T=T0, Q=Q0 ) ;
  ( object_map_pose_([ParentFrame,ObjFrame,T0,Q0], MapPose0, Time),
    object_map_pose(RefFrame,MapPose1,Time),
    transform_between(MapPose1,MapPose0,[RefFrame,ObjFrame,T,Q])
  )).

%%
object_map_pose(ObjFrame,MapPose,Time) :-
  object_frame_name(Obj,ObjFrame),
  object_pose(Obj,CurrentPose,Time),
  object_map_pose_(CurrentPose,MapPose,Time).

object_map_pose_([ParentFrame,ObjFrame,T,Q],MapPose,Time) :-
  map_frame_name(MapFrame),
  ( MapFrame=ParentFrame ->
  ( MapPose=[MapFrame,ObjFrame,T,Q] ) ;
  ( object_map_pose(ParentFrame,MapParent,Time),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).
