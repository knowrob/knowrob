:- begin_tripledb_tests(
    'model_URDF',
    'package://knowrob/owl/test/swrl.owl',
    [ namespace('http://knowrob.org/kb/swrl_test#')
    ]).

:- use_module(library('lang/query')).
:- use_module(library('lang/terms/holds')).
:- use_module(library('model/RDFS')).
:- use_module('parser').
:- use_module('URDF').

:- dynamic testbot/1.

%%
get_urdf_entity(Name,Entity) :-
	has_urdf_name(Entity,Name),
	!.

test(rdf_urdf_load) :-
	ros_package_path('knowrob', X),
	atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
	%%
	tell(has_type(Robot,urdf:'Robot')),
%	profile(urdf_load(Robot, FileURL)),
%	sleep(999999),
	urdf_load(Robot, FileURL),
	assertz(testbot(Robot)).

test(robot_name) :-
	testbot(Robot),
	has_urdf_name(Robot,pr2).

test(root_link) :-
	testbot(Robot),
	get_urdf_entity(base_footprint,L),
	has_root_link(Robot,L).

test(joint_names) :-
	testbot(Robot),
	forall(
		member(N, [
			l_forearm_roll_joint,
			base_bellow_joint,
			projector_wg6802418_frame_joint
		]),
		(	has_urdf_name(J,N),
			has_joint(Robot,J)
		)
	).

test(link_names) :-
	testbot(Robot),
	forall(
		member(N, [
			head_tilt_link,
			projector_wg6802418_child_frame,
			l_gripper_r_finger_tip_link
		]),
		(	has_urdf_name(L,N),
			has_link(Robot,L)
		)
	).

test(joint_has_child) :-
	get_urdf_entity(torso_lift_joint,J),
	get_urdf_entity(torso_lift_link,L),
	has_child_link(J,L).

test(joint_has_parent) :-
	get_urdf_entity(r_elbow_flex_joint,J),
	get_urdf_entity(r_upper_arm_link,L),
	has_parent_link(J,L).

test(joint_type_prismatic) :-
	get_urdf_entity(torso_lift_joint,J),
	has_type(J,urdf:'PrismaticJoint').

test(joint_type_continuous) :-
	get_urdf_entity(l_wrist_roll_joint,J),
	has_type(J,urdf:'ContinuousJoint').

test(joint_type_revolute) :-
	get_urdf_entity(r_shoulder_lift_joint,J),
	has_type(J,urdf:'RevoluteJoint').

test(joint_type_fixed) :-
	get_urdf_entity(head_plate_frame_joint,J),
	has_type(J,urdf:'FixedJoint').

test(joint_axis) :-
	get_urdf_entity(l_shoulder_pan_joint,J),
	has_joint_axis(J,[0.0,0.0,1.0]).

test(joint_limits) :-
	get_urdf_entity(l_elbow_flex_joint,J),
	has_joint_hard_limits(J, [-2.3213,0.0], 3.3, 30.0).

test(joint_soft_limits) :-
	get_urdf_entity(l_elbow_flex_joint,J),
	has_joint_soft_limits(J, [-2.1213,-0.15], 100.0, 3.0).

test(joint_calib) :-
	get_urdf_entity(fl_caster_rotation_joint,J),
	has_joint_calibration(J,0.0,-0.785398163397).

test(joint_dynamics) :-
	get_urdf_entity(l_elbow_flex_joint,J),
	has_joint_damping(J,1.0),
	has_joint_friction(J,0.0).

test(link_mass) :-
	get_urdf_entity(l_elbow_flex_link,L),
	has_link_mass(L, 1.90327).

test(link_inertial) :-
	get_urdf_entity(l_elbow_flex_link,L),
	once(has_link_inertia(L, _, _)).

test(link_no_vis) :-
	get_urdf_entity(r_gripper_led_frame,L),
	\+ has_link_visual(L,_,_).

test(link_vis) :-
	get_urdf_entity(r_gripper_motor_accelerometer_link,L),
	once(has_link_visual(L,_,_)).

test(link_visual_box, [nondet]) :-
	get_urdf_entity(r_gripper_motor_accelerometer_link,L),
	has_link_visual(L,box(0.001, 0.001, 0.001),_).

test(link_visual_sphere, [nondet]) :-
	get_urdf_entity(head_mount_kinect_ir_link,L),
	has_link_visual(L,sphere(0.0005),_).

test(link_visual_mesh_scaled) :-
	get_urdf_entity(head_mount_link,L),
	once(has_link_visual(L,mesh(_, [0.001, 0.001, 0.001]),_)).

test(link_visual_mesh) :-
	get_urdf_entity(laser_tilt_mount_link,L),
	once(has_link_visual(L, mesh(_, [1.0, 1.0, 1.0]),_)).

test(link_visual_cylinder, [nondet]) :-
	get_urdf_entity(r_gripper_motor_slider_link,L),
	has_link_visual(L,cylinder(0.025, 0.002),_).

test(link_with_collision) :-
	get_urdf_entity(base_link,L),
	once(has_link_collision(L,_,_)).

test(link_no_collision) :-
	get_urdf_entity(base_laser_link,L),
	\+ has_link_collision(L,_,_).

test(link_collision_mesh) :-
	get_urdf_entity(base_link,L),
	once(has_link_collision(L, mesh(_,_),_)).

test(link_collision_box, [nondet]) :-
	get_urdf_entity(torso_lift_motor_screw_link,L),
	has_link_collision(L, box(0.5, 0.7, 0.01),_).

test(link_collision_sphere, [nondet]) :-
	get_urdf_entity(head_mount_prosilica_link,L),
	has_link_collision(L, sphere(0.0005),_).

test(link_collision_cylinder, [nondet]) :-
	get_urdf_entity(fr_caster_r_wheel_link,L),
	has_link_collision(L, cylinder(0.074792, 0.034),_).

test(link_collision_mesh2) :-
	get_urdf_entity(r_gripper_r_finger_link,L),
	once(has_link_collision(L, mesh(_, [1.0,1.0,1.0]),_)).

test(link_visual_origin, [nondet]) :-
	get_urdf_entity(r_gripper_motor_slider_link,L),
	has_link_visual(L,cylinder(0.025, 0.002), pose(_,_)).

test(link_collision_origin) :-
	get_urdf_entity(r_gripper_r_finger_link,L),
	once(has_link_collision(L, mesh(_,_), pose(_,_))).

:- end_tripledb_tests('model_URDF').
