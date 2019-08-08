:- begin_tests(urdf_to_owl).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/rdfs')).

:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).
:- owl_parser:owl_parse('package://urdfprolog/owl/urdf.owl').

:- use_module('urdf_parser').
:- use_module('urdf_to_owl').
:- use_module('rdf_urdf').

:- dynamic testbot/1.

test(urdf_to_owl) :-
  ros_package_path('urdfprolog', X),
  atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
  %%
  urdf_to_owl(urdf:'Robot', FileURL, Robot),
  assertz(testbot(Robot)).

test(robot_name) :-
  testbot(Robot),
  rdf_urdf_name(Robot,pr2).

test(root_link) :-
  testbot(Robot),
  rdf_has(Robot,urdf:hasRootLink,L),
  rdf_urdf_name(L,base_footprint).

test(joint_names) :-
  testbot(Robot),
  forall( member(N, [
      l_forearm_roll_joint,
      base_bellow_joint,
      projector_wg6802418_frame_joint]),
  (
    rdf_urdf_joint(Robot,J),
    rdf_urdf_name(J,N)
  )).

test(link_names) :-
  testbot(Robot),
  forall( member(N, [
      head_tilt_link,
      projector_wg6802418_child_frame,
      l_gripper_r_finger_tip_link]),
  (
    rdf_urdf_link(Robot,L),
    rdf_urdf_name(L,N)
  )).

test(joint_has_child) :-
  rdf_urdf_name(J,torso_lift_joint),
  rdf_urdf_name(L,torso_lift_link),
  rdf_urdf_has_child(J,L).

test(joint_has_parent) :-
  rdf_urdf_name(J,r_elbow_flex_joint),
  rdf_urdf_name(L,r_upper_arm_link),
  rdf_urdf_has_parent(J,L).

test(joint_type_prismatic) :-
  rdf_urdf_name(J,torso_lift_joint),
  rdfs_individual_of(J,urdf:'PrismaticJoint').

test(joint_type_continuous) :-
  rdf_urdf_name(J,l_wrist_roll_joint),
  rdfs_individual_of(J,urdf:'ContinuousJoint').

test(joint_type_revolute) :-
  rdf_urdf_name(J,r_shoulder_lift_joint),
  rdfs_individual_of(J,urdf:'RevoluteJoint').

test(joint_type_fixed) :-
  rdf_urdf_name(J,head_plate_frame_joint),
  rdfs_individual_of(J,urdf:'FixedJoint').

test(joint_axis) :-
  rdf_urdf_name(J,l_shoulder_pan_joint),
  rdf_urdf_axis(J,[0.0,0.0,1.0]).

test(joint_origin) :-
  rdf_urdf_name(J,r_gripper_led_joint),
  rdf_urdf_joint_origin(J,[_,_,[0.0513,0.0,0.0244],[0.0,0.0,0.0,1.0]]).

test(joint_limits) :-
  rdf_urdf_name(J,l_elbow_flex_joint),
  rdf_urdf_kinematics(J,Kin),
  rdf_has(Kin,urdf:hasJointLimit,Lim),
  rdf_has_prolog(Lim,urdf:hasLowerLimit,-2.3213),
  rdf_has_prolog(Lim,urdf:hasUpperLimit,0.0),
  rdf_has_prolog(Lim,urdf:hasMaxJointVelocity,3.3),
  rdf_has_prolog(Lim,urdf:hasMaxJointEffort,30.0).

test(joint_calib) :-
  rdf_urdf_name(J,fl_caster_rotation_joint),
  rdf_urdf_calibration(J,0.0,-0.785398163397).

test(link_inertial) :-
  rdf_urdf_name(L,l_elbow_flex_link),
  rdf_urdf_inertial(L, 1.90327,
     [0.00346541989,
      0.00004066825,
      0.00043171614,
      0.00441606455,
      -0.00003968914,
      0.00359156824]).
  
test(link_inertial_origin) :-
  rdf_urdf_name(L,l_elbow_flex_link),
  rdf_urdf_inertial_origin(L,
    [_, _, [0.01014, 0.00032, -0.01211], [0.0, 0.0, 0.0, 1.0]]).

test(link_no_vis) :-
  rdf_urdf_name(L,r_gripper_led_frame),
  \+ rdf_urdf_visual(L,_).

test(link_vis) :-
  rdf_urdf_name(L,r_gripper_motor_accelerometer_link),
  once(rdf_urdf_visual(L,_)).

test(link_visual_box, [nondet]) :-
  rdf_urdf_name(L,r_gripper_motor_accelerometer_link),
  rdf_urdf_visual(L,box(0.001, 0.001, 0.001)).

test(link_visual_sphere, [nondet]) :-
  rdf_urdf_name(L,head_mount_kinect_ir_link),
  rdf_urdf_visual(L,sphere(0.0005)).

test(link_visual_mesh_scaled) :-
  rdf_urdf_name(L,head_mount_link),
  rdf_urdf_visual(L,mesh('package://pr2_description/meshes/sensors/kinect_prosilica_v0/115x100_swept_back--coarse.STL', [0.001, 0.001, 0.001])).

test(link_visual_mesh) :-
  rdf_urdf_name(L,laser_tilt_mount_link),
  rdf_urdf_visual(L, mesh('package://pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.dae', [1.0, 1.0, 1.0])).

test(link_visual_cylinder, [nondet]) :-
  rdf_urdf_name(L,r_gripper_motor_slider_link),
  rdf_urdf_visual(L,cylinder(0.025, 0.002)).

test(link_visual_origin, [nondet]) :-
  rdf_urdf_name(L,r_gripper_motor_slider_link),
  rdf_urdf_visual(L,cylinder(0.025, 0.002),
    [_,_,[0.0,0.0,0.0],[0.7071080798594737, 0.0, 0.0, 0.7071054825112364]]).

test(link_with_collision) :-
  rdf_urdf_name(L,base_link),
  once(rdf_urdf_collision(L,_)).

test(link_no_collision) :-
  rdf_urdf_name(L,base_laser_link),
  \+ rdf_urdf_collision(L,_).

test(link_collision_mesh) :-
  rdf_urdf_name(L,base_link),
  rdf_urdf_collision(L, mesh(_,_)).

test(link_collision_box, [nondet]) :-
  rdf_urdf_name(L,torso_lift_motor_screw_link),
  rdf_urdf_collision(L, box(0.5, 0.7, 0.01)).

test(link_collision_sphere, [nondet]) :-
  rdf_urdf_name(L,head_mount_prosilica_link),
  rdf_urdf_collision(L, sphere(0.0005)).

test(link_collision_cylinder, [nondet]) :-
  rdf_urdf_name(L,fr_caster_r_wheel_link),
  rdf_urdf_collision(L, cylinder(0.074792, 0.034)).

test(link_collision_mesh) :-
  rdf_urdf_name(L,r_gripper_r_finger_link),
  rdf_urdf_collision(L, mesh('package://pr2_description/meshes/gripper_v0/l_finger.stl', [1.0,1.0,1.0])).

test(link_collision_origin) :-
  rdf_urdf_name(L,r_gripper_r_finger_link),
  writeln(link_collision_origin),
  rdf_urdf_collision(L, mesh(_,_), XX),
  writeln(XX),
  rdf_urdf_collision(L, mesh(_,_),
    [_,_,[0.0,0.0,0.0],[1.0,0.0,0.0,0.0]]).

test(joint_dynamics) :-
  rdf_urdf_name(J,l_elbow_flex_joint),
  rdf_urdf_damping(J,1.0),
  rdf_urdf_friction(J,0.0).

test(joint_safety) :-
  rdf_urdf_name(J,l_elbow_flex_joint),
  rdf_urdf_kinematics(J,Kin),
  rdf_has(Kin,urdf:hasJointSafety,Saf),
  rdf_has_prolog(Saf,urdf:hasLowerLimit,-2.1213),
  rdf_has_prolog(Saf,urdf:hasUpperLimit,-0.15),
  rdf_has_prolog(Saf,urdf:hasKPosition,100.0),
  rdf_has_prolog(Saf,urdf:hasKVelocity,3.0).

:- end_tests(urdf_to_owl).
