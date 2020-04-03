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

:- module(rdf_urdf,
    [
    rdf_urdf_load/2,
    rdf_urdf_name/2,
    rdf_urdf_robot_joint/2,
    rdf_urdf_robot_link/2,
    rdf_urdf_joint_origin/2,
    rdf_urdf_joint_limits/4,
    rdf_urdf_joint_soft_limits/4,
    rdf_urdf_joint_calibration/3,
    rdf_urdf_joint_axis/2,
    rdf_urdf_joint_friction/2,
    rdf_urdf_joint_damping/2,
    rdf_urdf_has_child/2,
    rdf_urdf_has_parent/2,
    rdf_urdf_link_mass/2,
    rdf_urdf_link_visual/3,
    rdf_urdf_link_collision/3,
    rdf_urdf_link_inertia/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/comp/tf')).

:- use_module('urdf_parser').

:-  rdf_meta
    rdf_urdf_load(r,+),
    rdf_urdf_name(r,?),
    rdf_urdf_robot_joint(r,t),
    rdf_urdf_robot_link(r,t),
    rdf_urdf_joint_origin(r,?),
    rdf_urdf_joint_limits(r,?,?,?),
    rdf_urdf_joint_soft_limits(r,?,?,?),
    rdf_urdf_joint_axis(r,?),
    rdf_urdf_joint_friction(r,?),
    rdf_urdf_joint_damping(r,?),
    rdf_urdf_has_child(r,r),
    rdf_urdf_has_parent(r,r),
    rdf_urdf_calibration(r,?,?),
    rdf_urdf_link_visual(r,?,?),
    rdf_urdf_link_collision(r,?,?),
    rdf_urdf_link_inertia(r,?,?).

%% rdf_urdf_name(?Entity,?Name) is semidet.
%
rdf_urdf_name(Entity,Name) :-
  rdf_has(Entity,urdf:hasURDFName,literal(type(_,Name))).

%% rdf_urdf_joint_origin(?Robot,?Joint) is semidet.
%
rdf_urdf_robot_joint(Robot,Joint) :-
  rdf_has(Robot,urdf:hasJoint,Joint).

%% rdf_urdf_joint_origin(?Robot,?Link) is semidet.
%
rdf_urdf_robot_link(Robot,Link) :-
  rdf_has(Robot,urdf:hasLink,Link).

%% rdf_urdf_joint_origin(?Joint,?Origin) is semidet.
%
rdf_urdf_joint_origin(Joint,[_,_,Pos,Rot]) :- % TODO: what is the reference frame?
  rdf_has(Joint,urdf:hasOrigin,Origin),!,
  transform_data(Origin,(Pos,Rot)).

%% rdf_urdf_joint_limits(?Joint,?Pos,?Vel,?Eff) is semidet.
%
rdf_urdf_joint_limits(J, [LL,UL], VelMax, EffMax) :-
  rdf_has(J,urdf:hasJointLimits,Lim),
  kb_triple(Lim,urdf:hasLowerLimit,LL),
  kb_triple(Lim,urdf:hasUpperLimit,UL),
  kb_triple(Lim,urdf:hasMaxJointVelocity,VelMax),
  kb_triple(Lim,urdf:hasMaxJointEffort,EffMax).

%% rdf_urdf_joint_soft_limits(?Joint,?Pos,?KP,?KV) is semidet.
%
rdf_urdf_joint_soft_limits(J, [LL,UL], KP, KV) :-
  rdf_has(J,urdf:hasJointSoftLimits,Saf),
  kb_triple(Saf,urdf:hasLowerLimit,LL),
  kb_triple(Saf,urdf:hasUpperLimit,UL),
  kb_triple(Saf,urdf:hasKPosition,KP),
  kb_triple(Saf,urdf:hasKVelocity,KV).
  
%% rdf_urdf_joint_calibration(?Joint,?FallingEdge,?RisingEdge) is semidet.
%
rdf_urdf_joint_calibration(J,Falling,Rising) :-
  rdf_has(J,urdf:hasJointReferencePositions,Calib),
  kb_triple(Calib,urdf:hasRisingEdge,Rising),
  kb_triple(Calib,urdf:hasFallingEdge,Falling).

%% rdf_urdf_joint_axis(?Joint,?Axis) is semidet.
%
rdf_urdf_joint_axis(J,[X,Y,Z]) :-
  rdf_has(J,urdf:hasJointAxis,Axis),
  kb_triple(Axis,urdf:hasAxisVector,[X,Y,Z]).

%% rdf_urdf_has_child(?Joint,?Friction) is semidet.
%
rdf_urdf_joint_friction(J, Friction) :-
  rdf_has(J,ease_obj:hasFrictionAttribute,X),
  kb_triple(X,ease_obj:hasFrictionValue,Friction).
  
%% rdf_urdf_has_child(?Joint,?Damping) is semidet.
%
rdf_urdf_joint_damping(J, Damping) :-
  rdf_has(J,urdf:hasDampingAttribute,X),
  kb_triple(X,urdf:hasDampingValue,Damping).

%% rdf_urdf_has_child(?Joint,?Link) is semidet.
%
rdf_urdf_has_child(Joint,Link) :-
  rdf_has(Joint,urdf:hasChildLink,Link).

%% rdf_urdf_has_parent(?Joint,?Link) is semidet.
%
rdf_urdf_has_parent(Joint,Link) :-
  rdf_has(Joint,urdf:hasParentLink,Link).

rdf_urdf_origin(Entity,RefFrame,[_,RefFrame,Pos,Rot]) :-
  rdf_has(Entity,urdf:hasOrigin,Origin),!,
  transform_data(Origin,(Pos,Rot)).

rdf_urdf_origin(_Entity,RefFrame,[_,RefFrame,
  [0.0,0.0,0.0],[1.0,0.0,0.0,0.0]]).
  
%%
rdf_urdf_shape(Shape,box(X, Y, Z)) :-
  rdfs_individual_of(Shape,ease_obj:'BoxShape'),
  kb_triple(Shape, ease_obj:hasWidth, X),
  kb_triple(Shape, ease_obj:hasHeight, Y),
  kb_triple(Shape, ease_obj:hasDepth, Z),!.

rdf_urdf_shape(Shape,cylinder(Radius, Length)) :-
  rdfs_individual_of(Shape,ease_obj:'CylinderShape'),
  kb_triple(Shape, ease_obj:hasRadius, Radius),
  kb_triple(Shape, ease_obj:hasLength, Length),!.

rdf_urdf_shape(Shape,sphere(Radius)) :-
  rdfs_individual_of(Shape,ease_obj:'SphereShape'),
  kb_triple(Shape, ease_obj:hasRadius, Radius),!.

rdf_urdf_shape(Shape,mesh(Filename,Scale)) :-
  rdfs_individual_of(Shape,ease_obj:'MeshShape'),
  kb_triple(Shape, ease_obj:hasFilePath, Filename),
  mesh_scale_(Shape, Scale),!.

mesh_scale_(Shape, [X,Y,Z]) :-
  kb_triple(Shape, knowrob:hasXScale, X),
  kb_triple(Shape, knowrob:hasYScale, Y),
  kb_triple(Shape, knowrob:hasZScale, Z),!.
mesh_scale_(_Shape, [1.0,1.0,1.0]).

%% rdf_urdf_link_mass(+Link, ?MassValue) is semidet.
%
rdf_urdf_link_mass(Link,MassValue) :-
  rdf_has(Link,ease_obj:hasMassAttribute,Mass),
  kb_triple(Mass,ease_obj:hasMassValue,MassValue).

%% rdf_urdf_link_visual(+Link, ?ShapeTerm, ?Origin) is semidet.
%
rdf_urdf_link_visual(Link,ShapeTerm,Origin) :-
  rdf_urdf_name(Link,RefFrame),
  rdf_has(Link,ease_obj:hasShape,Shape),
  once(kb_triple(Shape,dul:hasRegion,ShapeRegion)),
  rdf_urdf_shape(ShapeRegion,ShapeTerm),
  rdf_urdf_origin(ShapeRegion,RefFrame,Origin).

%% rdf_urdf_link_collision(+Link, ?ShapeTerm, ?Origin) is semidet.
%
rdf_urdf_link_collision(Link,ShapeTerm,Origin) :-
  rdf_urdf_name(Link,RefFrame),
  rdf_has(Link,urdf:hasCollisionShape,Shape),
  once(kb_triple(Shape,dul:hasRegion,ShapeRegion)),
  rdf_urdf_shape(ShapeRegion,ShapeTerm),
  rdf_urdf_origin(ShapeRegion,RefFrame,Origin).

%% rdf_urdf_link_inertia(+Link, ?Matrix, ?Origin) is semidet.
%
rdf_urdf_link_inertia(Link,[XX,XY,XZ,YY,YZ,ZZ],[_,RefFrame,Pos,Rot]) :-
  rdf_has(Link,urdf:hasInertia,Inertia),
  %
  kb_triple(Inertia,urdf:hasInertia_ixx,XX),
  kb_triple(Inertia,urdf:hasInertia_ixy,XY),
  kb_triple(Inertia,urdf:hasInertia_ixz,XZ),
  kb_triple(Inertia,urdf:hasInertia_iyy,YY),
  kb_triple(Inertia,urdf:hasInertia_iyz,YZ),
  kb_triple(Inertia,urdf:hasInertia_izz,ZZ),
  %%
  rdf_urdf_name(Link,RefFrame),
  rdf_has(Inertia,urdf:hasOrigin,Origin),
  transform_data(Origin,(Pos,Rot)).

		 /************************************
		  *            Loading URDF Files    *
		  ************************************/

%% rdf_urdf_load(+Robot, +URDF_File) is det.
%
% Load a URDF file and map it into RDF triple store
% using the model defined in 'urdf.owl'.
%
rdf_urdf_load(Robot, URDF_File) :-
  rdf_split_url(_, Robot_Id, Robot),
  setup_call_cleanup(
    load_urdf_file(Robot_Id, URDF_File),
    rdf_urdf_load_(Robot, Robot_Id),
    unload_urdf_file(Robot_Id)
  ).
  
rdf_urdf_load_(Robot, Robot_Id) :-
  %%
  once(rdf(Robot,rdf:type,_,Graph)),
  %%
  robot_name(Robot_Id,RobotName),
  root_link_name(Robot_Id,RootLinkName),
  urdf_name_to_owl(Robot,RobotName),
  % read links
  link_names(Robot_Id,Links),
  forall(
    member(LinkName,Links), (
    urdf_link_to_owl(Robot_Id,LinkName,Link,Graph),
    ( RootLinkName = LinkName ->
      kb_assert(Robot,urdf:hasRootLink,Link);
      kb_assert(Robot,urdf:hasLink,Link) )
  )),
  % read joints
  joint_names(Robot_Id,Joints),
  forall(
    member(JointName,Joints), (
    urdf_joint_to_owl(Robot_Id,JointName,Joint,Graph),
    kb_assert(Robot,urdf:hasJoint,Joint)
  )).

%%
urdf_name_to_owl(Entity,Name) :-
  kb_assert(Entity,urdf:hasURDFName,Name).

%%
urdf_pose_to_owl(ParentFrame,pose([X,Y,Z],[QX,QY,QZ,QW]),Pose,Graph) :-
  kb_create(ease_obj:'6DPose', Pose, _{graph:Graph}),
  kb_assert(Pose,ease_obj:hasPositionVector,[X,Y,Z]),
  kb_assert(Pose,ease_obj:hasOrientationVector,[QX,QY,QZ,QW]),
  kb_assert(Pose,ease_obj:hasReferenceFrame,ParentFrame).
  
		 /*******************************
		 *		  Links		*
		 *******************************/

%%
urdf_link_to_owl(Robot_Id,Name,Link,Graph) :-
  kb_create(dul:'PhysicalObject', Link, _{graph:Graph}),
  urdf_name_to_owl(Link,Name),
  % TODO: handle material?
  %link_material_name/3,
  %link_material_color/3,
  %link_material_texture/3
  %%
  ( link_inertial_mass(Robot_Id,Name,MassValue) -> (
    kb_create(ease_obj:'MassAttribute', Mass, _{graph:Graph}),
    kb_assert(Link,ease_obj:hasMassAttribute,Mass),
    kb_assert(Mass,ease_obj:hasMassValue,MassValue)
  ) ; true ),
  %%
  ( link_inertial_inertia(Robot_Id,Name,_) ->
    urdf_link_inertia_to_owl(Robot_Id,Name,Link,Graph) ; true ),
  urdf_link_visuals_to_owl(Robot_Id,Name,Link,Graph),
  urdf_link_collisions_to_owl(Robot_Id,Name,Link,Graph).

%%
urdf_link_inertia_to_owl(Robot_Id,Name,Link,Graph) :-
  kb_create(urdf:'Inertia', Inertia, _{graph:Graph}),
  kb_assert(Link,urdf:hasInertia,Inertia),
  %%
  ( link_inertial_origin(Robot_Id,Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    kb_assert(Inertia,urdf:hasOrigin,Pose)
  ) ; true ),
  %%
  link_inertial_inertia(Robot_Id,Name,[XX,XY,XZ,YY,YZ,ZZ]),
  kb_assert(Inertia,urdf:hasInertia_ixx,XX),
  kb_assert(Inertia,urdf:hasInertia_ixy,XY),
  kb_assert(Inertia,urdf:hasInertia_ixz,XZ),
  kb_assert(Inertia,urdf:hasInertia_iyy,YY),
  kb_assert(Inertia,urdf:hasInertia_iyz,YZ),
  kb_assert(Inertia,urdf:hasInertia_izz,ZZ).

%%
urdf_link_visuals_to_owl(Robot_Id,Name,Link,Graph) :-
  link_num_visuals(Robot_Id,Name,Count),
  N is Count - 1,
  forall(
    between(0,N,Index),
    urdf_link_visual_to_owl(Robot_Id,Name,Index,Link,Graph)
  ).

%%
urdf_link_visual_to_owl(Robot_Id,Name,Index,Link,Graph) :-
  %%
  link_visual_geometry(Robot_Id,Name,Index,Geom),
  urdf_shape_to_owl(Geom,Shape,Graph),
  urdf_assert_shape(Link,Shape,Graph),
  %%
  ( link_visual_origin(Robot_Id,Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    kb_assert(Shape,urdf:hasOrigin,Pose)
  ) ; true ).

%%
urdf_link_collisions_to_owl(Robot_Id,Name,Link,Graph) :-
  link_num_collisions(Robot_Id,Name,Count),
  N is Count - 1,
  forall(
    between(0,N,Index),
    urdf_link_collision_to_owl(Robot_Id,Name,Index,Link,Graph)
  ).

%%
urdf_link_collision_to_owl(Robot_Id,Name,Index,Link,Graph) :-
  %%
  link_collision_geometry(Robot_Id,Name,Index,Geom),
  urdf_shape_to_owl(Geom,Shape,Graph),
  urdf_assert_collision_shape(Link,Shape,Graph),
  %%
  ( link_collision_origin(Robot_Id,Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    kb_assert(Shape,urdf:hasOrigin,Pose)
  ) ; true ).

urdf_assert_shape(Resource,ShapeRegion,G) :-
  kb_create(ease_obj:'Shape',Shape,_{graph: G}),
  kb_assert(Resource,ease_obj:hasShape,Shape),
  kb_assert(Shape,dul:hasRegion,ShapeRegion).

urdf_assert_collision_shape(Resource,ShapeRegion,G) :-
  kb_create(ease_obj:'Shape',Shape,_{graph: G}),
  kb_assert(Resource,urdf:hasCollisionShape,Shape),
  kb_assert(Shape,dul:hasRegion,ShapeRegion).

		 /*******************************
		 *		  Joints	*
		 *******************************/

%%
urdf_joint_has_calibration(Robot_Id,Name) :-
  ( joint_calibration_rising(Robot_Id,Name,_) ;
    joint_calibration_falling(Robot_Id,Name,_) ), !.

%%
urdf_joint_has_dynamics(Robot_Id,Name) :-
  ( joint_dynamics_damping(Robot_Id,Name,_) ;
    joint_dynamics_friction(Robot_Id,Name,_) ), !.

%%
urdf_joint_has_safety(Robot_Id,Name) :-
  joint_safety_kv(Robot_Id,Name,_).

%%
urdf_joint_to_owl(Robot_Id,Name,Joint,Graph) :-
  joint_type(Robot_Id,Name,Type),
  owl_joint_create(Type,Joint,Graph),
  %%
  urdf_name_to_owl(Joint,Name),
  %% kinematic chain
  joint_child_link(Robot_Id,Name,ChildName),
  joint_parent_link(Robot_Id,Name,ParentName),
  kb_triple(ChildLink, urdf:hasURDFName, ChildName),
  kb_triple(ParentLink, urdf:hasURDFName, ParentName),
  kb_assert(Joint, urdf:hasChildLink, ChildLink),
  kb_assert(Joint, urdf:hasParentLink, ParentLink),
  %%
  ( joint_origin(Robot_Id,Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    kb_assert(Joint,urdf:hasOrigin,Pose)
  ) ; true ),
  %% joint dynamics (optional)
  ( urdf_joint_has_dynamics(Robot_Id,Name) ->
    urdf_joint_dynamics_to_owl(Robot_Id,Name,Joint,Graph) ; true ),
  %% joint kinematics
  ( Type = fixed -> true ;
    urdf_joint_kinematics_to_owl(Robot_Id,Name,Type,Joint,Graph) ).

%%
owl_joint_create(revolute,Joint,Graph) :-
  kb_create(urdf:'RevoluteJoint',Joint,_{graph:Graph}).
owl_joint_create(continuous,Joint,Graph) :-
  kb_create(urdf:'ContinuousJoint',Joint,_{graph:Graph}).
owl_joint_create(prismatic,Joint,Graph) :-
  kb_create(urdf:'PrismaticJoint',Joint,_{graph:Graph}).
owl_joint_create(fixed,Joint,Graph) :-
  kb_create(urdf:'FixedJoint',Joint,_{graph:Graph}).
owl_joint_create(floating,Joint,Graph) :-
  kb_create(urdf:'FloatingJoint',Joint,_{graph:Graph}).
owl_joint_create(planar,Joint,Graph) :-
  kb_create(urdf:'PlanarJoint',Joint,_{graph:Graph}).

%%
urdf_joint_dynamics_to_owl(Robot_Id,Name,Joint,Graph) :-
  %%
  ( joint_dynamics_damping(Robot_Id,Name,DampingValue) -> (
    kb_create(urdf:'DampingAttribute', Damping, _{graph:Graph}),
    kb_assert(Joint, urdf:hasDampingAttribute, Damping),
    kb_assert(Damping,urdf:hasDampingValue,DampingValue)
  ) ; true ),
  %%
  ( joint_dynamics_friction(Robot_Id,Name,FrictionValue) ->(
    kb_create(ease_obj:'StaticFrictionAttribute', Friction, _{graph:Graph}),
    kb_assert(Joint, ease_obj:hasFrictionAttribute, Friction),
    kb_assert(Friction,ease_obj:hasFrictionValue,FrictionValue)
  ) ; true ).

%%
urdf_joint_kinematics_to_owl(Robot_Id,Name,Type,Joint,Graph) :-
  urdf_joint_axis_to_owl(Robot_Id,Name,Joint,Graph),
  %% joint limits (only revolute or prismatic)
  ( member(Type,[revolute,prismatic]) ->
    urdf_joint_limits_to_owl(Robot_Id,Name,Joint,Graph) ; true ),
  %% joint calibration (optional)
  ( urdf_joint_has_calibration(Robot_Id,Name) ->
    urdf_joint_calibration_to_owl(Robot_Id,Name,Joint,Graph) ; true ),
  %% joint safety (optional)
  ( urdf_joint_has_safety(Robot_Id,Name) ->
    urdf_joint_safety_to_owl(Robot_Id,Name,Joint,Graph) ; true ).

%%
urdf_joint_axis_to_owl(Robot_Id,Name,Pos,Graph) :-
  (joint_axis(Robot_Id,Name,[X,Y,Z]) ; [X,Y,Z] = [1,0,0]), !,
  kb_create(urdf:'JointAxis', Axis, _{graph:Graph}),
  kb_assert(Pos, urdf:hasJointAxis, Axis),
  kb_assert(Axis, urdf:hasAxisVector, [X,Y,Z]).

%%
urdf_joint_calibration_to_owl(Robot_Id,Name,Pos,Graph) :-  
  ( joint_calibration_rising(Robot_Id,Name,Rising) ; Rising is 0.0 ),
  ( joint_calibration_falling(Robot_Id,Name,Falling) ; Falling is 0.0 ),!,
  kb_create(urdf:'JointReferencePositions', RP, _{graph:Graph}),
  kb_assert(Pos, urdf:hasJointReferencePositions, RP),
  kb_assert(RP, urdf:hasRisingEdge, Rising),
  kb_assert(RP, urdf:hasFallingEdge, Falling).

%%
urdf_joint_limits_to_owl(Robot_Id,Name,Pos,Graph) :-
  joint_velocity_limit(Robot_Id,Name,Vel),
  joint_effort_limit(Robot_Id,Name,Eff),
  ( joint_lower_pos_limit(Robot_Id,Name, LL) ; LL is 0 ),
  ( joint_upper_pos_limit(Robot_Id,Name, UL) ; UL is 0 ),!,
  %%
  kb_create(urdf:'JointLimits', Lim, _{graph:Graph}),
  kb_assert(Pos, urdf:hasJointLimits, Lim),
  kb_assert(Lim, urdf:hasMaxJointEffort, Eff),
  kb_assert(Lim, urdf:hasMaxJointVelocity, Vel),
  kb_assert(Lim, urdf:hasLowerLimit, LL),
  kb_assert(Lim, urdf:hasUpperLimit, UL).

%%
urdf_joint_safety_to_owl(Robot_Id,Name,Pos,Graph) :-
  joint_safety_kv(Robot_Id,Name, KV),
  ( joint_safety_kp(Robot_Id,Name, KP) ; KP is 0.0 ),
  ( joint_safety_lower_limit(Robot_Id,Name, LL) ; LL is 0.0 ),
  ( joint_safety_upper_limit(Robot_Id,Name, UL) ; UL is 0.0 ),!,
  %%
  kb_create(urdf:'JointSoftLimits', Saf, _{graph:Graph}),
  kb_assert(Pos, urdf:hasJointSoftLimits, Saf),
  kb_assert(Saf, urdf:hasKVelocity, KV),
  kb_assert(Saf, urdf:hasKPosition, KP),
  kb_assert(Saf, urdf:hasLowerLimit, LL),
  kb_assert(Saf, urdf:hasUpperLimit, UL).
  
		 /*******************************
		 *		  Shapes	*
		 *******************************/

%% create shape region symbol. URDF only supports the following:
%%    Box, Sphere, Cylinder, and Mesh
urdf_shape_to_owl(box(X, Y, Z),Shape,Graph) :-
  kb_create(ease_obj:'BoxShape', Shape, _{graph:Graph}),
  kb_assert(Shape, ease_obj:hasWidth, X),
  kb_assert(Shape, ease_obj:hasHeight, Y),
  kb_assert(Shape, ease_obj:hasDepth, Z).

urdf_shape_to_owl(cylinder(Radius, Length),Shape,Graph) :-
  kb_create(ease_obj:'CircularCylinder', Shape, _{graph:Graph}),
  kb_assert(Shape, ease_obj:hasRadius, Radius),
  kb_assert(Shape, ease_obj:hasLength, Length).

urdf_shape_to_owl(sphere(Radius),Shape,Graph) :-
  kb_create(ease_obj:'SphereShape', Shape, _{graph:Graph}),
  kb_assert(Shape, ease_obj:hasRadius, Radius).

urdf_shape_to_owl(mesh(Filename,Scale),Shape,Graph) :-
  kb_create(ease_obj:'MeshShape', Shape, _{graph:Graph}),
  kb_assert(Shape, ease_obj:hasFilePath, Filename),
  ( Scale=[1,1,1] -> true ; (
    Scale=[Scale_X,Scale_Y,Scale_Z],
    kb_assert(Shape, knowrob:hasXScale, Scale_X),
    kb_assert(Shape, knowrob:hasYScale, Scale_Y),
    kb_assert(Shape, knowrob:hasZScale, Scale_Z)
  )).
