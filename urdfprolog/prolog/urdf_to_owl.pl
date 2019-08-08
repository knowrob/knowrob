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

:- module(urdf_to_owl,
    [
      urdf_to_owl/3,
      urdf_to_owl/4
    ]).

/** <module> Translation of URDF to OWL-based representation.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).

:- use_module('urdf_parser').

:-  rdf_meta
    urdf_to_owl(r,+,-),
    urdf_to_owl(r,+,-,+).

%% urdf_to_owl(RobotType, FileURL, Robot, Graph)
%
urdf_to_owl(RobotType, FileURL, Robot) :-
  urdf_to_owl(RobotType, FileURL, Robot, user).

urdf_to_owl(RobotType, FileURL, Robot, Graph) :-
  load_urdf_file(FileURL),
  % create robot symbol
  rdf_instance_from_class(RobotType, Graph, Robot),
  robot_name(RobotName),
  root_link_name(RootLinkName),
  urdf_name_to_owl(Robot,RobotName,Graph),
  % read links
  link_names(Links),
  forall(
    member(LinkName,Links), (
    urdf_link_to_owl(LinkName,Link,Graph),
    ( RootLinkName = LinkName ->
      rdf_assert(Robot,urdf:hasRootLink,Link,Graph);
      rdf_assert(Robot,urdf:hasLink,Link,Graph) )
  )),
  % read joints
  joint_names(Joints),
  forall(
    member(JointName,Joints), (
    urdf_joint_to_owl(JointName,Joint,Graph),
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
urdf_link_has_dynamics(Name) :-
  ( link_inertial_inertia(Name,_) ;
    link_inertial_mass(Name,_) ), !.

%%
urdf_link_to_owl(Name,Link,Graph) :-
  rdf_instance_from_class(urdf:'Link', Graph, Link),
  urdf_name_to_owl(Link,Name,Graph),
  % TODO: handle material?
  %link_material_name/3,
  %link_material_color/3,
  %link_material_texture/3
  %%
  ( urdf_link_has_dynamics(Name) ->
    urdf_link_dynamics_to_owl(Name,Link,Graph) ; true ),
  urdf_link_visuals_to_owl(Name,Link,Graph),
  urdf_link_collisions_to_owl(Name,Link,Graph).

%%
urdf_link_dynamics_to_owl(Name,Link,Graph) :-
  rdf_instance_from_class(urdf:'Dynamics', Graph, Dyn),
  rdf_assert(Link,urdf:hasDynamics,Dyn,Graph),
  %%
  ( link_inertial_origin(Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Dyn,urdf:hasInertiaOrigin,Pose,Graph)
  ) ; true ),
  %%
  link_inertial_mass(Name,MassValue),
  rdf_instance_from_class(ease_obj:'Mass', Graph, Mass),
  rdf_assert(Dyn,urdf:hasMass,Mass,Graph),
  rdf_assert(Mass,dul:hasDataValue,literal(type(xsd:double,MassValue)),Graph),
  %%
  link_inertial_inertia(Name,[XX,XY,XZ,YY,YZ,ZZ]),
  rdf_instance_from_class(urdf:'InertiaMatrix', Graph, Mat),
  rdf_assert(Dyn,urdf:hasInertiaMatrix,Mat,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_ixx,XX,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_ixy,XY,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_ixz,XZ,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_iyy,YY,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_iyz,YZ,Graph),
  rdf_assert_prolog(Mat,urdf:hasInertia_izz,ZZ,Graph).

%%
urdf_link_visuals_to_owl(Name,Link,Graph) :-
  link_num_visuals(Name,Count),
  N is Count - 1,
  forall(
    between(0,N,Index),
    urdf_link_visual_to_owl(Name,Index,Link,Graph)
  ).

%%
urdf_link_visual_to_owl(Name,Index,Link,Graph) :-
  rdf_instance_from_class(urdf:'Visual', Graph, Vis),
  rdf_assert(Link,urdf:hasVisual,Vis,Graph),
  %%
  ( link_visual_origin(Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Vis,urdf:hasOrigin,Pose,Graph)
  ) ; true ),
  %%
  link_visual_geometry(Name,Index,Geom),
  urdf_shape_to_owl(Vis,Graph,Geom).

%%
urdf_link_collisions_to_owl(Name,Link,Graph) :-
  link_num_collisions(Name,Count),
  N is Count - 1,
  forall(
    between(0,N,Index),
    urdf_link_collision_to_owl(Name,Index,Link,Graph)
  ).

%%
urdf_link_collision_to_owl(Name,Index,Link,Graph) :-
  rdf_instance_from_class(urdf:'CollisionModel', Graph, Coll),
  rdf_assert(Link,urdf:hasCollisionModel,Coll,Graph),
  %%
  ( link_collision_origin(Name,Index,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Coll,urdf:hasOrigin,Pose,Graph)
  ) ; true ),
  %%
  link_collision_geometry(Name,Index,Geom),
  urdf_shape_to_owl(Coll,Graph,Geom).

		 /*******************************
		 *		  Joints	*
		 *******************************/

%%
urdf_joint_has_calibration(Name) :-
  ( joint_calibration_rising(Name,_) ;
    joint_calibration_falling(Name,_) ), !.

%%
urdf_joint_has_dynamics(Name) :-
  ( joint_dynamics_damping(Name,_) ;
    joint_dynamics_friction(Name,_) ), !.

%%
urdf_joint_has_safety(Name) :-
  joint_safety_kv(Name,_).

%%
urdf_joint_to_owl(Name,Joint,Graph) :-
  joint_type(Name,Type),
  owl_joint_create(Type,Joint,Graph),
  %%
  urdf_name_to_owl(Joint,Name,Graph),
  %% kinematic chain
  joint_child_link(Name,ChildName),
  joint_parent_link(Name,ParentName),
  rdf_has_prolog(ChildLink, urdf:hasURDFName, ChildName),
  rdf_has_prolog(ParentLink, urdf:hasURDFName, ParentName),
  rdf_assert(Joint, urdf:hasChildLink, ChildLink, Graph),
  rdf_assert(Joint, urdf:hasParentLink, ParentLink, Graph),
  %%
  ( joint_origin(Name,Pose_data) -> (
    urdf_pose_to_owl(Name,Pose_data,Pose,Graph),
    rdf_assert(Joint,urdf:hasJointOrigin,Pose,Graph)
  ) ; true ),
  %% joint dynamics (optional)
  ( urdf_joint_has_dynamics(Name) ->
    urdf_joint_dynamics_to_owl(Name,Joint,Graph) ; true ),
  %% joint kinematics
  ( Type = fixed -> true ;
    urdf_joint_kinematics_to_owl(Name,Type,Joint,Graph) ).

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
urdf_joint_dynamics_to_owl(Name,Joint,Graph) :-
  rdf_instance_from_class(urdf:'Dynamics', Graph, Dyn),
  rdf_assert(Joint, urdf:hasDynamics, Dyn, Graph),
  %%
  ( joint_dynamics_damping(Name,DampingValue) -> (
    rdf_instance_from_class(urdf:'Damping', Graph, Damping),
    rdf_assert(Dyn, urdf:hasDamping, Damping, Graph),
    rdf_assert(Damping,dul:hasDataValue,
      literal(type(xsd:double,DampingValue)),Graph)
  ) ; true ),
  %%
  ( joint_dynamics_friction(Name,FrictionValue) ->(
    rdf_instance_from_class(urdf:'Friction', Graph, Friction),
    rdf_assert(Dyn, urdf:hasFriction, Friction, Graph),
    rdf_assert(Friction,dul:hasDataValue,
      literal(type(xsd:double,FrictionValue)),Graph)
  ) ; true ).

%%
urdf_joint_kinematics_to_owl(Name,Type,Joint,Graph) :-
  rdf_instance_from_class(urdf:'Kinematics', Graph, Kin),
  rdf_assert(Joint, urdf:hasKinematics, Kin, Graph),
  urdf_joint_axis_to_owl(Name,Kin,Graph),
  %% joint limits (only revolute or prismatic)
  ( member(Type,[revolute,prismatic]) ->
    urdf_joint_limits_to_owl(Name,Kin,Graph) ; true ),
  %% joint calibration (optional)
  ( urdf_joint_has_calibration(Name) ->
    urdf_joint_calibration_to_owl(Name,Kin,Graph) ; true ),
  %% joint safety (optional)
  ( urdf_joint_has_safety(Name) ->
    urdf_joint_safety_to_owl(Name,Kin,Graph) ; true ).

%%
urdf_joint_axis_to_owl(Name,Kin,Graph) :-
  (joint_axis(Name,[X,Y,Z]) ; [X,Y,Z] = [1,0,0]), !,
  rdf_instance_from_class(urdf:'JointAxis', Graph, Axis),
  rdf_assert(Kin, urdf:hasJointAxis, Axis, Graph),
  rdf_assert_prolog(Axis, ease:hasXComponent, X, Graph),
  rdf_assert_prolog(Axis, ease:hasYComponent, Y, Graph),
  rdf_assert_prolog(Axis, ease:hasZComponent, Z, Graph).

%%
urdf_joint_calibration_to_owl(Name,Kin,Graph) :-  
  ( joint_calibration_rising(Name,Rising) ; Rising is 0.0 ),
  ( joint_calibration_falling(Name,Falling) ; Falling is 0.0 ),!,
  rdf_instance_from_class(urdf:'JointCalibrationAttribute', Graph, Call),
  rdf_assert(Kin, urdf:hasJointCalibration, Call, Graph),
  rdf_assert_prolog(Call, urdf:hasRisingEdge, Rising, Graph),
  rdf_assert_prolog(Call, urdf:hasFallingEdge, Falling, Graph).

%%
urdf_joint_limits_to_owl(Name,Kin,Graph) :-
  joint_velocity_limit(Name,Vel),
  joint_effort_limit(Name,Eff),
  ( joint_lower_pos_limit(Name, LL) ; LL is 0 ),
  ( joint_upper_pos_limit(Name, UL) ; UL is 0 ),!,
  %%
  rdf_instance_from_class(urdf:'JointLimit', Graph, Lim),
  rdf_assert(Kin, urdf:hasJointLimit, Lim, Graph),
  rdf_assert_prolog(Lim, urdf:hasMaxJointEffort, Eff, Graph),
  rdf_assert_prolog(Lim, urdf:hasMaxJointVelocity, Vel, Graph),
  rdf_assert_prolog(Lim, urdf:hasLowerLimit, LL, Graph),
  rdf_assert_prolog(Lim, urdf:hasUpperLimit, UL, Graph).

%%
urdf_joint_safety_to_owl(Name,Kin,Graph) :-
  joint_safety_kv(Name, KV),
  ( joint_safety_kp(Name, KP) ; KP is 0.0 ),
  ( joint_safety_lower_limit(Name, LL) ; LL is 0.0 ),
  ( joint_safety_upper_limit(Name, UL) ; UL is 0.0 ),!,
  %%
  rdf_instance_from_class(urdf:'JointSafetyAttribute', Graph, Saf),
  rdf_assert(Kin, urdf:hasJointSafety, Saf, Graph),
  rdf_assert_prolog(Saf, urdf:hasKVelocity, KV, Graph),
  rdf_assert_prolog(Saf, urdf:hasKPosition, KP, Graph),
  rdf_assert_prolog(Saf, urdf:hasLowerLimit, LL, Graph),
  rdf_assert_prolog(Saf, urdf:hasUpperLimit, UL, Graph).
  
		 /*******************************
		 *		  Shapes	*
		 *******************************/

%% create shape region symbol. URDF only supports the following:
%%    Box, Sphere, Cylinder, and Mesh
urdf_shape_to_owl(Entity,Graph,box(X, Y, Z)) :-
  rdf_instance_from_class(ease_obj:'Box', Graph, Shape),
  rdf_assert(Entity, ease_obj:hasShape, Shape, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasWidth, X, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasHeight, Y, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasDepth, Z, Graph).

urdf_shape_to_owl(Entity,Graph,cylinder(Radius, Length)) :-
  rdf_instance_from_class(ease_obj:'Cylinder', Graph, Shape),
  rdf_assert(Entity, ease_obj:hasShape, Shape, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasRadius, Radius, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasLength, Length, Graph).

urdf_shape_to_owl(Entity,Graph,sphere(Radius)) :-
  rdf_instance_from_class(ease_obj:'Sphere', Graph, Shape),
  rdf_assert(Entity, ease_obj:hasShape, Shape, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasRadius, Radius, Graph).

urdf_shape_to_owl(Entity,Graph,mesh(Filename,Scale)) :-
  rdf_instance_from_class(ease_obj:'Mesh', Graph, Shape),
  rdf_assert(Entity, ease_obj:hasShape, Shape, Graph),
  rdf_assert_prolog(Shape, ease_obj:hasFilePath, Filename, Graph),
  ( Scale=[1,1,1] -> true ; (
    Scale=[Scale_X,Scale_Y,Scale_Z],
    rdf_assert_prolog(Shape, ease_obj:hasXScale, Scale_X, Graph),
    rdf_assert_prolog(Shape, ease_obj:hasYScale, Scale_Y, Graph),
    rdf_assert_prolog(Shape, ease_obj:hasZScale, Scale_Z, Graph)
  )).
