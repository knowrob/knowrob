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
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/transforms')).

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
  rdf_has_prolog(Lim,urdf:hasLowerLimit,LL),
  rdf_has_prolog(Lim,urdf:hasUpperLimit,UL),
  rdf_has_prolog(Lim,urdf:hasMaxJointVelocity,VelMax),
  rdf_has_prolog(Lim,urdf:hasMaxJointEffort,EffMax).

%% rdf_urdf_joint_soft_limits(?Joint,?Pos,?KP,?KV) is semidet.
%
rdf_urdf_joint_soft_limits(J, [LL,UL], KP, KV) :-
  rdf_has(J,urdf:hasJointSoftLimits,Saf),
  rdf_has_prolog(Saf,urdf:hasLowerLimit,LL),
  rdf_has_prolog(Saf,urdf:hasUpperLimit,UL),
  rdf_has_prolog(Saf,urdf:hasKPosition,KP),
  rdf_has_prolog(Saf,urdf:hasKVelocity,KV).
  
%% rdf_urdf_joint_calibration(?Joint,?FallingEdge,?RisingEdge) is semidet.
%
rdf_urdf_joint_calibration(J,Falling,Rising) :-
  rdf_has(J,urdf:hasJointReferencePositions,Calib),
  rdf_has_prolog(Calib,urdf:hasRisingEdge,Rising),
  rdf_has_prolog(Calib,urdf:hasFallingEdge,Falling).

%% rdf_urdf_joint_axis(?Joint,?Axis) is semidet.
%
rdf_urdf_joint_axis(J,[X,Y,Z]) :-
  rdf_has(J,urdf:hasJointAxis,Axis),
  rdf_has_prolog(Axis,ease:hasXComponent,X),
  rdf_has_prolog(Axis,ease:hasYComponent,Y),
  rdf_has_prolog(Axis,ease:hasZComponent,Z).

%% rdf_urdf_has_child(?Joint,?Friction) is semidet.
%
rdf_urdf_joint_friction(J, Friction) :-
  rdf_has(J,ease_obj:hasFrictionAttribute,X),
  rdf_has_prolog(X,ease_obj:hasFrictionValue,Friction).
  
%% rdf_urdf_has_child(?Joint,?Damping) is semidet.
%
rdf_urdf_joint_damping(J, Damping) :-
  rdf_has(J,urdf:hasDampingAttribute,X),
  rdf_has_prolog(X,urdf:hasDampingValue,Damping).

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
  rdfs_individual_of(Shape,ease_obj:'BoxShapeAttribute'),
  rdf_has_prolog(Shape, ease_obj:hasWidth, X),
  rdf_has_prolog(Shape, ease_obj:hasHeight, Y),
  rdf_has_prolog(Shape, ease_obj:hasDepth, Z).

rdf_urdf_shape(Shape,cylinder(Radius, Length)) :-
  rdfs_individual_of(Shape,ease_obj:'CylinderShapeAttribute'),
  rdf_has_prolog(Shape, ease_obj:hasRadius, Radius),
  rdf_has_prolog(Shape, ease_obj:hasLength, Length).

rdf_urdf_shape(Shape,sphere(Radius)) :-
  rdfs_individual_of(Shape,ease_obj:'SphereShapeAttribute'),
  rdf_has_prolog(Shape, ease_obj:hasRadius, Radius).

rdf_urdf_shape(Shape,mesh(Filename,Scale)) :-
  rdfs_individual_of(Shape,ease_obj:'MeshAttribute'),
  rdf_has_prolog(Shape, ease_obj:hasFilePath, Filename),
  mesh_scale_(Shape, Scale).

mesh_scale_(Shape, [X,Y,Z]) :-
  rdf_has_prolog(Shape, knowrob:hasXScale, X),
  rdf_has_prolog(Shape, knowrob:hasYScale, Y),
  rdf_has_prolog(Shape, knowrob:hasZScale, Z),!.
mesh_scale_(_Shape, [1.0,1.0,1.0]).

%% rdf_urdf_link_mass(+Link, ?MassValue) is semidet.
%
rdf_urdf_link_mass(Link,MassValue) :-
  rdf_has(Link,ease_obj:hasMassAttribute,Mass),
  rdf_has_prolog(Mass,ease_obj:hasMassValue,MassValue).

%% rdf_urdf_link_visual(+Link, ?ShapeTerm, ?Origin) is semidet.
%
rdf_urdf_link_visual(Link,ShapeTerm,Origin) :-
  rdf_urdf_name(Link,RefFrame),
  rdf_has(Link,ease_obj:hasShapeAttribute,Shape),
  rdf_urdf_shape(Shape,ShapeTerm),
  rdf_urdf_origin(Shape,RefFrame,Origin).

%% rdf_urdf_link_collision(+Link, ?ShapeTerm, ?Origin) is semidet.
%
rdf_urdf_link_collision(Link,ShapeTerm,Origin) :-
  rdf_urdf_name(Link,RefFrame),
  rdf_has(Link,urdf:hasCollisionShape,Shape),
  rdf_urdf_shape(Shape,ShapeTerm),
  rdf_urdf_origin(Shape,RefFrame,Origin).

%% rdf_urdf_link_inertia(+Link, ?Matrix, ?Origin) is semidet.
%
rdf_urdf_link_inertia(Link,[XX,XY,XZ,YY,YZ,ZZ],[_,RefFrame,Pos,Rot]) :-
  rdf_has(Link,urdf:hasInertia,Inertia),
  %
  rdf_has_prolog(Inertia,urdf:hasInertia_ixx,XX),
  rdf_has_prolog(Inertia,urdf:hasInertia_ixy,XY),
  rdf_has_prolog(Inertia,urdf:hasInertia_ixz,XZ),
  rdf_has_prolog(Inertia,urdf:hasInertia_iyy,YY),
  rdf_has_prolog(Inertia,urdf:hasInertia_iyz,YZ),
  rdf_has_prolog(Inertia,urdf:hasInertia_izz,ZZ),
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
  urdf_name_to_owl(Robot,RobotName,Graph),
  % read links
  link_names(Robot_Id,Links),
  forall(
    member(LinkName,Links), (
    urdf_link_to_owl(Robot_Id,LinkName,Link,Graph),
    ( RootLinkName = LinkName ->
      rdf_assert(Robot,urdf:hasRootLink,Link,Graph);
      rdf_assert(Robot,urdf:hasLink,Link,Graph) )
  )),
  % read joints
  joint_names(Robot_Id,Joints),
  forall(
    member(JointName,Joints), (
    urdf_joint_to_owl(Robot_Id,JointName,Joint,Graph),
    rdf_assert(Robot,urdf:hasJoint,Joint,Graph)
  )).

%%
urdf_name_to_owl(Entity,Name,Graph) :-
  rdf_assert_prolog(Entity,urdf:hasURDFName,Name,Graph).

%%
urdf_pose_to_owl(ParentFrame,pose([X,Y,Z],[QX,QY,QZ,QW]),Pose,Graph) :-
  rdf_instance_from_class(ease:'6DPose', Graph, Pose),
  urdf_position_to_owl([X,Y,Z],P,Graph),
  urdf_quaternion_to_owl([QX,QY,QZ,QW],Q,Graph),
  rdf_assert(Pose,dul:hasPart,P,Graph),
  rdf_assert(Pose,dul:hasPart,Q,Graph),
  rdf_assert_prolog(Pose,ease:hasReferenceFrame,ParentFrame,Graph).

urdf_position_to_owl([X,Y,Z],P,Graph) :-
  rdf_instance_from_class(ease:'3DPosition', Graph, P),
  rdf_assert_prolog(P,ease:hasXComponent,X,Graph),
  rdf_assert_prolog(P,ease:hasYComponent,Y,Graph),
  rdf_assert_prolog(P,ease:hasZComponent,Z,Graph).

urdf_quaternion_to_owl([QX,QY,QZ,QW],Q,Graph) :-
  rdf_instance_from_class(ease:'Quaternion', Graph, Q),
  rdf_assert_prolog(Q,ease:hasXComponent,QX,Graph),
  rdf_assert_prolog(Q,ease:hasYComponent,QY,Graph),
  rdf_assert_prolog(Q,ease:hasZComponent,QZ,Graph),
  rdf_assert_prolog(Q,ease:hasWComponent,QW,Graph).
  
		 /*******************************
		 *		  Links		*
		 *******************************/

%%
urdf_link_to_owl(Robot_Id,Name,Link,Graph) :-
  rdf_instance_from_class(urdf:'Link', Graph, Link),
  urdf_name_to_owl(Link,Name,Graph),
  % TODO: handle material?
  %link_material_name/3,
  %link_material_color/3,
  %link_material_texture/3
  %%
  ( link_inertial_mass(Robot_Id,Name,MassValue) -> (
    rdf_instance_from_class(ease_obj:'MassAttribute', Graph, Mass),
    rdf_assert(Link,ease_obj:hasMassAttribute,Mass,Graph),
    rdf_assert_prolog(Mass,ease_obj:hasMassValue,MassValue,Graph)
  ) ; true ),
  %%
  ( link_inertial_inertia(Robot_Id,Name,_) ->
    urdf_link_inertia_to_owl(Robot_Id,Name,Link,Graph) ; true ),
  urdf_link_visuals_to_owl(Robot_Id,Name,Link,Graph),
  urdf_link_collisions_to_owl(Robot_Id,Name,Link,Graph).

%%
urdf_link_inertia_to_owl(Robot_Id,Name,Link,Graph) :-
  rdf_instance_from_class(urdf:'Inertia', Graph, Inertia),
  rdf_assert(Link,urdf:hasInertia,Inertia,Graph),
  %%
  ( link_inertial_origin(Robot_Id,Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Inertia,urdf:hasOrigin,Pose,Graph)
  ) ; true ),
  %%
  link_inertial_inertia(Robot_Id,Name,[XX,XY,XZ,YY,YZ,ZZ]),
  rdf_assert_prolog(Inertia,urdf:hasInertia_ixx,XX,Graph),
  rdf_assert_prolog(Inertia,urdf:hasInertia_ixy,XY,Graph),
  rdf_assert_prolog(Inertia,urdf:hasInertia_ixz,XZ,Graph),
  rdf_assert_prolog(Inertia,urdf:hasInertia_iyy,YY,Graph),
  rdf_assert_prolog(Inertia,urdf:hasInertia_iyz,YZ,Graph),
  rdf_assert_prolog(Inertia,urdf:hasInertia_izz,ZZ,Graph).

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
  rdf_assert(Link,ease_obj:hasShapeAttribute,Shape,Graph),
  %%
  ( link_visual_origin(Robot_Id,Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Shape,urdf:hasOrigin,Pose,Graph)
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
  rdf_assert(Link,urdf:hasCollisionShape,Shape,Graph),
  %%
  ( link_collision_origin(Robot_Id,Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Shape,urdf:hasOrigin,Pose,Graph)
  ) ; true ).

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
  urdf_name_to_owl(Joint,Name,Graph),
  %% kinematic chain
  joint_child_link(Robot_Id,Name,ChildName),
  joint_parent_link(Robot_Id,Name,ParentName),
  rdf_has_prolog(ChildLink, urdf:hasURDFName, ChildName),
  rdf_has_prolog(ParentLink, urdf:hasURDFName, ParentName),
  rdf_assert(Joint, urdf:hasChildLink, ChildLink, Graph),
  rdf_assert(Joint, urdf:hasParentLink, ParentLink, Graph),
  %%
  ( joint_origin(Robot_Id,Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Joint,urdf:hasOrigin,Pose,Graph)
  ) ; true ),
  %% joint dynamics (optional)
  ( urdf_joint_has_dynamics(Robot_Id,Name) ->
    urdf_joint_dynamics_to_owl(Robot_Id,Name,Joint,Graph) ; true ),
  %% joint kinematics
  ( Type = fixed -> true ;
    urdf_joint_kinematics_to_owl(Robot_Id,Name,Type,Joint,Graph) ).

%%
owl_joint_create(revolute,Joint,Graph) :-
  rdf_instance_from_class(urdf:'RevoluteJoint',Graph,Joint).
owl_joint_create(continuous,Joint,Graph) :-
  rdf_instance_from_class(urdf:'ContinuousJoint',Graph,Joint).
owl_joint_create(prismatic,Joint,Graph) :-
  rdf_instance_from_class(urdf:'PrismaticJoint',Graph,Joint).
owl_joint_create(fixed,Joint,Graph) :-
  rdf_instance_from_class(urdf:'FixedJoint',Graph,Joint).
owl_joint_create(floating,Joint,Graph) :-
  rdf_instance_from_class(urdf:'FloatingJoint',Graph,Joint).
owl_joint_create(planar,Joint,Graph) :-
  rdf_instance_from_class(urdf:'PlanarJoint',Graph,Joint).

%%
urdf_joint_dynamics_to_owl(Robot_Id,Name,Joint,Graph) :-
  %%
  ( joint_dynamics_damping(Robot_Id,Name,DampingValue) -> (
    rdf_instance_from_class(urdf:'DampingAttribute', Graph, Damping),
    rdf_assert(Joint, urdf:hasDampingAttribute, Damping, Graph),
    rdf_assert_prolog(Damping,urdf:hasDampingValue,DampingValue,Graph)
  ) ; true ),
  %%
  ( joint_dynamics_friction(Robot_Id,Name,FrictionValue) ->(
    rdf_instance_from_class(ease_obj:'StaticFrictionAttribute', Graph, Friction),
    rdf_assert(Joint, ease_obj:hasFrictionAttribute, Friction, Graph),
    rdf_assert_prolog(Friction,ease_obj:hasFrictionValue,FrictionValue,Graph)
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
  rdf_instance_from_class(urdf:'JointAxis', Graph, Axis),
  rdf_assert(Pos, urdf:hasJointAxis, Axis, Graph),
  rdf_assert_prolog(Axis, ease:hasXComponent, X, Graph),
  rdf_assert_prolog(Axis, ease:hasYComponent, Y, Graph),
  rdf_assert_prolog(Axis, ease:hasZComponent, Z, Graph).

%%
urdf_joint_calibration_to_owl(Robot_Id,Name,Pos,Graph) :-  
  ( joint_calibration_rising(Robot_Id,Name,Rising) ; Rising is 0.0 ),
  ( joint_calibration_falling(Robot_Id,Name,Falling) ; Falling is 0.0 ),!,
  rdf_instance_from_class(urdf:'JointReferencePoints', Graph, RP),
  rdf_assert(Pos, urdf:hasJointReferencePositions, RP, Graph),
  rdf_assert_prolog(RP, urdf:hasRisingEdge, Rising, Graph),
  rdf_assert_prolog(RP, urdf:hasFallingEdge, Falling, Graph).

%%
urdf_joint_limits_to_owl(Robot_Id,Name,Pos,Graph) :-
  joint_velocity_limit(Robot_Id,Name,Vel),
  joint_effort_limit(Robot_Id,Name,Eff),
  ( joint_lower_pos_limit(Robot_Id,Name, LL) ; LL is 0 ),
  ( joint_upper_pos_limit(Robot_Id,Name, UL) ; UL is 0 ),!,
  %%
  rdf_instance_from_class(urdf:'JointLimit', Graph, Lim),
  rdf_assert(Pos, urdf:hasJointLimits, Lim, Graph),
  rdf_assert_prolog(Lim, urdf:hasMaxJointEffort, Eff, Graph),
  rdf_assert_prolog(Lim, urdf:hasMaxJointVelocity, Vel, Graph),
  rdf_assert_prolog(Lim, urdf:hasLowerLimit, LL, Graph),
  rdf_assert_prolog(Lim, urdf:hasUpperLimit, UL, Graph).

%%
urdf_joint_safety_to_owl(Robot_Id,Name,Pos,Graph) :-
  joint_safety_kv(Robot_Id,Name, KV),
  ( joint_safety_kp(Robot_Id,Name, KP) ; KP is 0.0 ),
  ( joint_safety_lower_limit(Robot_Id,Name, LL) ; LL is 0.0 ),
  ( joint_safety_upper_limit(Robot_Id,Name, UL) ; UL is 0.0 ),!,
  %%
  rdf_instance_from_class(urdf:'JointSafetyAttribute', Graph, Saf),
  rdf_assert(Pos, urdf:hasJointSoftLimits, Saf, Graph),
  rdf_assert_prolog(Saf, urdf:hasKVelocity, KV, Graph),
  rdf_assert_prolog(Saf, urdf:hasKPosition, KP, Graph),
  rdf_assert_prolog(Saf, urdf:hasLowerLimit, LL, Graph),
  rdf_assert_prolog(Saf, urdf:hasUpperLimit, UL, Graph).
  
		 /*******************************
		 *		  Shapes	*
		 *******************************/

%% create shape region symbol. URDF only supports the following:
%%    Box, Sphere, Cylinder, and Mesh
urdf_shape_to_owl(box(X, Y, Z),Shape,Graph) :-
  rdf_instance_from_class(ease_obj:'BoxShapeAttribute', Graph, Shape),
  rdf_assert_prolog(Shape, ease_obj:hasWidth, X, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasHeight, Y, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasDepth, Z, Graph).

urdf_shape_to_owl(cylinder(Radius, Length),Shape,Graph) :-
  rdf_instance_from_class(ease_obj:'CircularCylinderShapeAttribute', Graph, Shape),
  rdf_assert_prolog(Shape, ease_obj:hasRadius, Radius, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasLength, Length, Graph).

urdf_shape_to_owl(sphere(Radius),Shape,Graph) :-
  rdf_instance_from_class(ease_obj:'SphereShapeAttribute', Graph, Shape),
  rdf_assert_prolog(Shape, ease_obj:hasRadius, Radius, Graph).

urdf_shape_to_owl(mesh(Filename,Scale),Shape,Graph) :-
  rdf_instance_from_class(ease_obj:'MeshAttribute', Graph, Shape),
  rdf_assert_prolog(Shape, ease_obj:hasFilePath, Filename, Graph),
  ( Scale=[1,1,1] -> true ; (
    Scale=[Scale_X,Scale_Y,Scale_Z],
    rdf_assert_prolog(Shape, knowrob:hasXScale, Scale_X, Graph),
    rdf_assert_prolog(Shape, knowrob:hasYScale, Scale_Y, Graph),
    rdf_assert_prolog(Shape, knowrob:hasZScale, Scale_Z, Graph)
  )).
