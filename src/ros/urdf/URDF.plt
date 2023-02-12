:- begin_rdf_tests('ros_urdf', 'package://knowrob/owl/test/swrl.owl').

:- use_module(library('model/RDFS')).
:- use_module('URDF').

:- sw_register_prefix(test, 'http://knowrob.org/kb/swrl_test#', [force(true)]).

test(load_urdf_file_pr2) :-
  ros_package_path('knowrob', X),
  atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', Filename),
  urdf_load_file(pr2,Filename).

test(robot_name_pr2) :-
  urdf_robot_name(pr2,pr2).

test(root_link_name_pr2) :-
  urdf_root_link(pr2,base_footprint).

test(joint_child_link_pr2_torso_lift_joint) :-
  urdf_joint_child_link(pr2,torso_lift_joint, torso_lift_link).

test(joint_child_link_pr2_l_shoulder_pan_joint) :-
  urdf_joint_child_link(pr2,l_shoulder_pan_joint, l_shoulder_pan_link).

test(joint_child_link_pr2_nonexisting_joint) :-
  catch(urdf_joint_child_link(pr2, foo, l_shoulder_pan_link), urdf_error(Msg), true),
  ground(Msg).

test(joint_parent_link_pr2_r_elbow_flex_joint) :-
  urdf_joint_parent_link(pr2, r_elbow_flex_joint, r_upper_arm_roll_link).

test(joint_parent_link_pr2_head_tilt_joint) :-
  urdf_joint_parent_link(pr2, head_tilt_joint, head_pan_link).

test(joint_parent_link_nonexisting_joint) :-
  catch(urdf_joint_parent_link(pr2, bar, head_pan_link), urdf_error(Msg), true),
  ground(Msg).

test(joint_type_pr2_torso_lift_joint) :-
  urdf_joint_type(pr2, torso_lift_joint, prismatic).

test(joint_type_pr2_l_wrist_roll_joint) :-
  urdf_joint_type(pr2, l_wrist_roll_joint, continuous).

test(joint_type_pr2_r_shoulder_lift_joint) :-
  urdf_joint_type(pr2, r_shoulder_lift_joint, revolute).

test(joint_type_pr2_head_plate_frame_joint) :-
  urdf_joint_type(pr2, head_plate_frame_joint, fixed).

test(joint_child_link_and_link_parent_joint_pr2_left_arm, forall(
  member(J,[l_shoulder_pan_joint, l_shoulder_lift_joint, l_upper_arm_roll_joint,
        l_elbow_flex_joint, l_forearm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint]))) :-
  urdf_joint_child_link(pr2,J,L),
  urdf_link_parent_joint(pr2,L,J).

% Observation: The root link of a robot never has a parent joint.
test(link_parent_joint_pr2_root_link, fail) :-
  urdf_root_link(pr2,L),
  urdf_link_parent_joint(pr2, L, _).


test(link_child_joints_pr2_torso_lift_link) :-
  urdf_link_child_joints(pr2, torso_lift_link, Joints),
  Joints ==
    [head_pan_joint, imu_joint, l_shoulder_pan_joint, l_torso_lift_side_plate_joint,
     laser_tilt_mount_joint, r_shoulder_pan_joint, r_torso_lift_side_plate_joint].

test(joint_axis_pr2_l_shoulder_pan_joint) :-
  urdf_joint_axis(pr2, l_shoulder_pan_joint, [0,0,1]).

test(joint_axis_pr2_fixed_joint, fail) :-
  urdf_joint_axis(pr2, head_plate_frame_joint, _).

test(joint_origin_pr2_r_gripper_led_joint) :-
  urdf_joint_origin(pr2, r_gripper_led_joint, pose([0.0513, 0.0, 0.0244], [0, 0, 0, 1])).

test(joint_origin_pr2_l_foreaem_cam_optical_frame_joint) :-
  urdf_joint_origin(pr2, l_forearm_cam_optical_frame_joint,
               pose([0,0,0],[-0.5, 0.5, -0.5, 0.5])).

test(joint_lower_limit_pr2_l_elbow_flex_joint) :-
  urdf_joint_hard_limits(pr2, l_elbow_flex_joint, [-2.3213,_], _, _).

test(joint_lower_limit_pr2_l_wrist_roll_joint, fail) :-
  urdf_joint_hard_limits(pr2, l_wrist_roll_joint, _, _, _).

test(joint_upper_limit_pr2_torso_lift_joint) :-
  urdf_joint_hard_limits(pr2, torso_lift_joint, [_,0.33], _, _).

test(joint_upper_limit_pr2_r_forearm_roll_joint, fail) :-
  urdf_joint_hard_limits(pr2, r_forearm_roll_joint, _, _, _).

test(joint_vel_limit_pr2_r_gripper_joint) :-
  urdf_joint_hard_limits(pr2, r_gripper_joint, _, 0.2, _).

test(joint_effort_limit_pr2_head_pan_joint) :-
  urdf_joint_hard_limits(pr2, head_pan_joint, _, _, 6.0).

test(joint_calibration_rising_pr2_torso_lift_joint, fail) :-
  urdf_joint_calibration_rising(pr2, torso_lift_joint, _).

test(joint_calibration_falling_pr2_fl_caster_rotation_joint, fail) :-
  urdf_joint_calibration_falling(pr2, fl_caster_rotation_joint, _).

test(joint_calibration_rising_pr2_fl_caster_rotation_joint) :-
  urdf_joint_calibration_rising(pr2, fl_caster_rotation_joint, -0.785398163397).

test(joint_calibration_falling_pr2_torso_lift_joint) :-
	urdf_joint_calibration_falling(pr2, torso_lift_joint, Falling),
	assert_equals(Falling,0.00475).

test(joint_dynamics_damping_pr2_l_torso_lift_side_plate_joint, fail) :-
  urdf_joint_damping(pr2, l_torso_lift_side_plate_joint, _).

test(joint_dynamics_friction_pr2_l_torso_lift_side_plate_joint, fail) :-
  urdf_joint_friction(pr2, l_torso_lift_side_plate_joint, _).

test(joint_dynamics_damping_pr2_head_pan_joint) :-
  urdf_joint_damping(pr2, head_pan_joint, 0.5).

test(joint_dynamics_friction_pr2_head_pan_joint) :-
  urdf_joint_friction(pr2, head_pan_joint, 0.0).

test(joint_safety_lower_limit_pr2_l_upper_arm_joint, fail) :-
  urdf_joint_soft_limits(pr2, l_upper_arm_joint, _, _, _).

test(joint_safety_upper_limit_pr2_l_upper_arm_joint, fail) :-
  urdf_joint_soft_limits(pr2, l_upper_arm_joint, _, _, _).

test(joint_safety_kp_pr2_l_upper_arm_joint, fail) :-
  urdf_joint_soft_limits(pr2, l_upper_arm_joint, _, _, _).

test(joint_safety_kv_pr2_l_upper_arm_joint, fail) :-
  urdf_joint_soft_limits(pr2, l_upper_arm_joint, _, _, _).

test(joint_safety_lower_limit_pr2_l_elbow_flex_joint) :-
  urdf_joint_soft_limits(pr2, l_elbow_flex_joint, [-2.1213,_], _, _).

test(joint_safety_upper_limit_pr2_l_elbow_flex_joint) :-
  urdf_joint_soft_limits(pr2, l_elbow_flex_joint, [_,-0.15], _, _).

test(joint_safety_kp_pr2_l_elbow_flex_joint) :-
  urdf_joint_soft_limits(pr2, l_elbow_flex_joint, _, 100.0, _).

test(joint_safety_kv_pr2_l_elbow_flex_joint) :-
  urdf_joint_soft_limits(pr2, l_elbow_flex_joint, _, _, 3.0).

test(link_inertial_origin_pr2_l_gripper_led_frame, fail) :-
  urdf_link_inertial(pr2, l_gripper_led_frame, _, _, _).

test(link_inertial_mass_pr2_l_gripper_led_frame, fail) :-
  urdf_link_inertial(pr2, l_gripper_led_frame, _, _, _).

test(link_inertial_inertia_pr2_l_gripper_led_frame, fail) :-
  urdf_link_inertial(pr2, l_gripper_led_frame, _, _, _).

test(link_inertial_origin_pr2_l_elbow_flex_link) :-
  urdf_link_inertial(pr2, l_elbow_flex_link, _,
  	[l_elbow_flex_link, [0.01014, 0.00032, -0.01211], [0.0, 0.0, 0.0, 1.0]],
  	_).

test(link_inertial_mass_pr2_l_elbow_flex_link) :-
  urdf_link_inertial(pr2, l_elbow_flex_link, _, _, 1.90327).

test(link_inertial_mass_pr2_l_elbow_flex_link) :-
  urdf_link_inertial(pr2, l_elbow_flex_link,
  	[0.00346541989, 0.00004066825, 0.00043171614, 0.00441606455, 0.00003968914, 0.00359156824],
  	_, _).

test(link_num_visuals_pr2_r_gripper_led_frame, [fail]) :-
  urdf_link_visual_shape(pr2, r_gripper_led_frame, _, _, _, _).

test(link_visual_type_pr2_r_gripper_motor_accelerometer_link) :-
  urdf_link_visual_shape(pr2, r_gripper_motor_accelerometer_link, Shape, _, _, _),
  assert_equals(Shape,box(0.001,0.001,0.001)).

test(link_visual_type_pr2_r_gripper_motor_slider_link) :-
  urdf_link_visual_shape(pr2, r_gripper_motor_slider_link, cylinder(0.002, 0.025), _, _, _).

test(link_visual_geometry_pr2_head_mount_kinect_ir_link) :-
  urdf_link_visual_shape(pr2, head_mount_kinect_ir_link, sphere(0.0005), _, _, _).

test(link_visual_geometry_pr2_head_mount_link) :-
  urdf_link_visual_shape(pr2, head_mount_link, mesh(_, [0.001, 0.001, 0.001]), _, _, _).

test(link_origin_pr2_r_gripper_motor_slider_link) :-
  urdf_link_visual_shape(pr2, r_gripper_motor_slider_link, _,
	  [	r_gripper_motor_slider_link,
	  	[0.0,0.0,0.0],
	  	[0.7071080798594737, 0.0, 0.0, 0.7071054825112363]
	  ], _, _).

test(link_collision_geometry_pr2_r_gripper_r_finger_link) :-
	urdf_link_collision_shape(pr2, r_gripper_r_finger_link,
  		mesh("package://pr2_description/meshes/gripper_v0/l_finger.stl", [1.0, 1.0, 1.0]),
  		_).

test(link_num_collisions_pr2_base_laser_link, [fail]) :-
  urdf_link_collision_shape(pr2, base_laser_link, _, _).

test(link_num_collision_foo_link) :-
  catch(urdf_link_collision_shape(pr2, foo, _, _), urdf_error(Msg), true),
  ground(Msg).

test(link_collision_geometry_pr2_head_mount_prosilica_link) :-
  urdf_link_collision_shape(pr2, head_mount_prosilica_link, sphere(0.0005), _).

test(link_collision_geometry_pr2_fr_caster_r_wheel_link) :-
  urdf_link_collision_shape(pr2, fr_caster_r_wheel_link, cylinder(0.074792, 0.034), _).

test(link_collision_geometry_pr2_torso_lift_motor_screw_link) :-
  urdf_link_collision_shape(pr2, torso_lift_motor_screw_link, box(0.5, 0.7, 0.01), _).

test(link_collision_origin_pr2_r_gripper_r_finger_link) :-
	urdf_link_collision_shape(pr2, r_gripper_r_finger_link, _,
		[r_gripper_r_finger_link, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]]).

test(urdf_unload) :-
  urdf_unload_file(pr2).

:- end_rdf_tests('ros_urdf').
