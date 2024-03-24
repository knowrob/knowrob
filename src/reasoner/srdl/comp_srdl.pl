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

:- module(comp_srdl, []).

%:- module(comp_srdl,
%    [
%        comp_restricted_action/3,
%        comp_installable_for_action/4,
%        comp_installable_on_robot/3,
%        comp_baselink_pose/2,
%        comp_baselink_pose/3,
%        srdl_inFieldOfView/2,
%        srdl_inFieldOfView/3
%  ]).
%/** <module> Computables in the SRDL domain
%
%  @author Daniel Beßler
%  @license BSD
%*/
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/owl_parser')).
%:- use_module(library('semweb/owl')).
%:- use_module(library('knowrob/knowrob')).
%:- use_module(library('knowrob/computable')).
%:- use_module(library('knowrob/action_planning')).
%
%:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(srdl2act, 'http://knowrob.org/kb/srdl2-action.owl#', [keep(true)]).
%
%:- rdf_meta
%        comp_restricted_action(r,r,r),
%        comp_installable_for_action(r,r,r,-),
%        comp_installable_on_robot(r,r,r),
%        comp_baselink_pose(r,-),
%        comp_baselink_pose_at_time(r,-,+),
%        srdl_inFieldOfView(r,-),
%        srdl_inFieldOfView_at_time(r,-,+).
%
%%% comp_restricted_action(+Comp, ?ActionC, ?RestrictedC) is nondet.
%%
%% Yields restricted action classes of components that are subclass
%% of the action class.
%%
%% @param Comp     Component required to perform the action
%% @param ActionC   Action class to be checked
%% @param RestrictedC   Restricted action class possible with component
%%
%comp_restricted_action(Comp, ActionC, RestrictedC) :-
%  findall(R, (
%    owl_individual_of(Comp, CompC),
%    owl_class_properties(CompC, 'http://knowrob.org/kb/knowrob.owl#actionable', R),
%    owl_subclass_of(R, ActionC)
%  ), Rs),
%  list_to_set(Rs, RestrictedSet),
%  member(RestrictedC, RestrictedSet).
%
%
%%% comp_installable_for_action(+ActionC, +ActionD, +Robot, -Components).
%%
%% For each insufficient component for the action, obtain a set of
%% sufficient component individuals that could be installed on the robot.
%%
%% @param ActionC   Action class to be checked
%% @param ActionD   Action individual or description to be checked
%% @param Robot   Robot instance to be checked
%% @param Components    Sequence of installation candidates for each insufficient component
%%
%comp_installable_for_action(ActionC, ActionD, Robot, Components) :-
%  with_action_description(ActionD, ActionI, Robot, (
%    findall((CompC,Xs), (
%      insufficient_comp_for_action(ActionC, ActionI, Robot, CompC),
%      findall(X, comp_installable_on_robot(X, CompC, Robot), Xs)
%    ), Components)
%  )).
%
%
%%% comp_installable_on_robot(+Comp, +CompC, +Robot).
%%
%% True for components Comp of type CompC that can be installed on the robot.
%%
%% @param CompC   Component class to be checked
%% @param Comp   Component individual to be checked
%% @param Robot   Robot instance to be checked
%%
%comp_installable_on_robot(Comp, CompC, Robot) :-
%  once(owl_individual_of(Comp, CompC)),
%  ActionD = [an, action,
%      [type, srdl2act:installing_hardware_component],
%      [object_acted_on, [an, object, [name, Comp] ]]
%  ],
%  action_feasible_on_robot(srdl2act:'InstallingHardwareComponent', ActionD, Robot).
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Pose of semantic components
%
%%% comp_baselink_pose(+Obj,-Pose) is nondet.
%%% comp_baselink_pose(+Obj,-Pose,+Interval) is nondet.
%%
%% Computes the pose of a component composition as the 
%% pose of one of its base links.
%%
%comp_baselink_pose(Obj, Pose) :-
%  get_time(Instant),
%  comp_baselink_pose_at_time(Obj, Pose, [Instant,Instant]).
%
%comp_baselink_pose(Obj, Pose, Interval) :-
%  nonvar(Obj),
%  rdf_has(Obj, srdl2comp:baseLinkOfComposition, BaseLink),
%  holds(BaseLink, knowrob:pose, Pose, Interval), !.
%
%% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Higher-level reasoning methods
%%
%
%%% srdl_inFieldOfView(+Agent, ?Object) is nondet.
%%% srdl_inFieldOfView(+Agent, ?Object, +Instant) is nondet.
%%
%% Check if Obj is visible by Agent at time Instant by reading the camera
%% properties from the robot's SRDL description and computing whether the
%% object center is inside the view frustrum.
%%
%% @param Agent      Some agent with a srdl:Camera component
%% @param Obj        Instance of an object in the scene
%% @param Instant    The time instant
%% 
%srdl_inFieldOfView(Agent, Object) :-
%  get_time(Instant),
%  srdl_inFieldOfView_at_time(Agent, Object, Instant).
%
%srdl_inFieldOfView(Agent, Object, [Instant,Instant]) :-
%  srdl_inFieldOfView_at_time(Agent, Object, Instant).
%
%srdl_inFieldOfView_at_time(Agent, Object, Instant) :-
%  nonvar(Agent),
%  map_frame_name(MapFrame),
%  % read camera properties
%  agent_camera(Agent, Camera),
%  camera_image_size(Camera, [ImgX,ImgY]),
%  camera_hfov(Camera, HFOV),
%  VFOV is ImgY / ImgX * HFOV,
%  % find object pose in camera frame
%  object_frame_name(Camera, CameraFrame),
%  object_pose_at_time(Camera, Instant, [MapFrame, _, CamPos_world, CamRot_world]),
%  object_frame_name(Object, ObjectFrame),
%  object_pose_at_time(Object, Instant, [MapFrame, _, ObjPos_world, ObjRot_world]),
%  transform_between(
%      [x,ObjectFrame, ObjPos_world, ObjRot_world],
%      [x,CameraFrame, CamPos_world, CamRot_world],
%      [CameraFrame, ObjectFrame, [ObjX,ObjY,ObjZ], _]),
%  % make the visibility tests
%  BearingX is atan2(ObjY, ObjX),
%  BearingY is atan2(ObjZ, ObjX),
%  abs(BearingX) < HFOV/2,
%  abs(BearingY) < VFOV/2.
%
%agent_camera(Agent, Camera) :-
%  sub_component(Agent, Camera),
%  owl_individual_of(Camera, srdl2comp:'Camera'), !.
%
%camera_image_size(Camera, [ImgX,ImgY,ImgZ]) :-
%  owl_has(Camera, srdl2comp:imageSizeX, literal(type(_, ImgXa))), atom_number(ImgXa, ImgX),
%  owl_has(Camera, srdl2comp:imageSizeY, literal(type(_, ImgYa))), atom_number(ImgYa, ImgY),
%  owl_has(Camera, srdl2comp:imageSizeZ, literal(type(_, ImgZa))), atom_number(ImgZa, ImgZ), !.
%camera_hfov(Camera, HFOV) :-
%  owl_has(Camera, srdl2comp:hfov, literal(type(_, HFOVa))), atom_number(HFOVa, HFOV), !.
    
