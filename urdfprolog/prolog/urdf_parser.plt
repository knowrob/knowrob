/*
  Copyright (C) 2018 Georg Bartels
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

:- begin_tests(urdf_parser).

:- use_module('urdf_parser').
:- use_module(library('roscpp')).

test(load_urdf_file_pr2) :-
  ros_package_path('urdfprolog', X),
  atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', Filename),
  load_urdf_file(pr2,Filename).

test(load_non_existent_urdf, fail) :-
  load_urdf_file(foo,'foo.urdf').

test(robot_name_pr2) :-
  robot_name(pr2,pr2).

test(root_link_name_pr2) :-
  root_link_name(pr2,base_footprint).

test(joint_names_pr2) :-
  joint_names(pr2,Names),
  Names == [base_bellow_joint, base_footprint_joint, base_laser_joint,
            bl_caster_l_wheel_joint, bl_caster_r_wheel_joint, bl_caster_rotation_joint,
            br_caster_l_wheel_joint, br_caster_r_wheel_joint, br_caster_rotation_joint,
            double_stereo_frame_joint, fl_caster_l_wheel_joint, fl_caster_r_wheel_joint,
            fl_caster_rotation_joint, fr_caster_l_wheel_joint, fr_caster_r_wheel_joint,
            fr_caster_rotation_joint, head_mount_joint, head_mount_kinect_ir_joint,
            head_mount_kinect_ir_optical_frame_joint, head_mount_kinect_rgb_joint,
            head_mount_kinect_rgb_optical_frame_joint, head_mount_prosilica_joint,
            head_mount_prosilica_optical_frame_joint, head_pan_joint,
            head_plate_frame_joint, head_tilt_joint, high_def_frame_joint,
            high_def_optical_frame_joint, imu_joint, l_elbow_flex_joint,
            l_forearm_cam_frame_joint, l_forearm_cam_optical_frame_joint,
            l_forearm_joint, l_forearm_roll_joint, l_gripper_joint,
            l_gripper_l_finger_joint, l_gripper_l_finger_tip_joint,
            l_gripper_led_joint, l_gripper_motor_accelerometer_joint,
            l_gripper_motor_screw_joint, l_gripper_motor_slider_joint,
            l_gripper_palm_joint, l_gripper_r_finger_joint,
            l_gripper_r_finger_tip_joint, l_gripper_tool_joint, l_shoulder_lift_joint,
            l_shoulder_pan_joint, l_torso_lift_side_plate_joint, l_upper_arm_joint,
            l_upper_arm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint,
            laser_tilt_joint, laser_tilt_mount_joint, narrow_stereo_frame_joint,
            narrow_stereo_l_stereo_camera_frame_joint, narrow_stereo_l_stereo_camera_optical_frame_joint,
            narrow_stereo_optical_frame_joint, narrow_stereo_r_stereo_camera_frame_joint,
            narrow_stereo_r_stereo_camera_optical_frame_joint,
            projector_wg6802418_child_frame_joint, projector_wg6802418_frame_joint,
            r_elbow_flex_joint, r_forearm_cam_frame_joint, r_forearm_cam_optical_frame_joint,
            r_forearm_joint, r_forearm_roll_joint, r_gripper_joint, r_gripper_l_finger_joint,
            r_gripper_l_finger_tip_joint, r_gripper_led_joint, r_gripper_motor_accelerometer_joint,
            r_gripper_motor_screw_joint, r_gripper_motor_slider_joint, r_gripper_palm_joint,
            r_gripper_r_finger_joint, r_gripper_r_finger_tip_joint, r_gripper_tool_joint,
            r_shoulder_lift_joint, r_shoulder_pan_joint, r_torso_lift_side_plate_joint,
            r_upper_arm_joint, r_upper_arm_roll_joint, r_wrist_flex_joint,
            r_wrist_roll_joint, sensor_mount_frame_joint, torso_lift_joint,
            torso_lift_motor_screw_joint, wide_stereo_frame_joint,
            wide_stereo_l_stereo_camera_frame_joint, wide_stereo_l_stereo_camera_optical_frame_joint,
            wide_stereo_optical_frame_joint, wide_stereo_r_stereo_camera_frame_joint,
            wide_stereo_r_stereo_camera_optical_frame_joint].

test(link_names_pr2) :-
  link_names(pr2,Names),
  Names ==
  [base_bellow_link, base_footprint, base_laser_link, base_link, bl_caster_l_wheel_link,
   bl_caster_r_wheel_link, bl_caster_rotation_link, br_caster_l_wheel_link,
   br_caster_r_wheel_link, br_caster_rotation_link, double_stereo_link, fl_caster_l_wheel_link,
   fl_caster_r_wheel_link, fl_caster_rotation_link, fr_caster_l_wheel_link, fr_caster_r_wheel_link,
   fr_caster_rotation_link, head_mount_kinect_ir_link, head_mount_kinect_ir_optical_frame,
   head_mount_kinect_rgb_link, head_mount_kinect_rgb_optical_frame, head_mount_link,
   head_mount_prosilica_link, head_mount_prosilica_optical_frame, head_pan_link,
   head_plate_frame, head_tilt_link, high_def_frame, high_def_optical_frame,
   imu_link, l_elbow_flex_link, l_forearm_cam_frame, l_forearm_cam_optical_frame,
   l_forearm_link, l_forearm_roll_link, l_gripper_l_finger_link, l_gripper_l_finger_tip_frame,
   l_gripper_l_finger_tip_link, l_gripper_led_frame, l_gripper_motor_accelerometer_link,
   l_gripper_motor_screw_link, l_gripper_motor_slider_link, l_gripper_palm_link,
   l_gripper_r_finger_link, l_gripper_r_finger_tip_link, l_gripper_tool_frame,
   l_shoulder_lift_link, l_shoulder_pan_link, l_torso_lift_side_plate_link,
   l_upper_arm_link, l_upper_arm_roll_link, l_wrist_flex_link, l_wrist_roll_link,
   laser_tilt_link, laser_tilt_mount_link, narrow_stereo_l_stereo_camera_frame,
   narrow_stereo_l_stereo_camera_optical_frame, narrow_stereo_link, narrow_stereo_optical_frame,
   narrow_stereo_r_stereo_camera_frame, narrow_stereo_r_stereo_camera_optical_frame,
   projector_wg6802418_child_frame, projector_wg6802418_frame, r_elbow_flex_link,
   r_forearm_cam_frame, r_forearm_cam_optical_frame, r_forearm_link, r_forearm_roll_link,
   r_gripper_l_finger_link, r_gripper_l_finger_tip_frame, r_gripper_l_finger_tip_link,
   r_gripper_led_frame, r_gripper_motor_accelerometer_link, r_gripper_motor_screw_link,
   r_gripper_motor_slider_link, r_gripper_palm_link, r_gripper_r_finger_link,
   r_gripper_r_finger_tip_link, r_gripper_tool_frame, r_shoulder_lift_link, r_shoulder_pan_link,
   r_torso_lift_side_plate_link, r_upper_arm_link, r_upper_arm_roll_link, r_wrist_flex_link,
   r_wrist_roll_link, sensor_mount_link, torso_lift_link, torso_lift_motor_screw_link,
   wide_stereo_l_stereo_camera_frame, wide_stereo_l_stereo_camera_optical_frame, wide_stereo_link,
   wide_stereo_optical_frame, wide_stereo_r_stereo_camera_frame, wide_stereo_r_stereo_camera_optical_frame].

test(joint_child_link_pr2_torso_lift_joint) :-
  joint_child_link(pr2,torso_lift_joint, torso_lift_link).

test(joint_child_link_pr2_l_shoulder_pan_joint) :-
  joint_child_link(pr2,l_shoulder_pan_joint, l_shoulder_pan_link).

test(joint_child_link_pr2_nonexisting_joint, fail) :-
  joint_child_link(pr2, foo, l_shoulder_pan_link).

test(joint_parent_link_pr2_r_elbow_flex_joint) :-
  joint_parent_link(pr2, r_elbow_flex_joint, r_upper_arm_roll_link).

test(joint_parent_link_pr2_head_tilt_joint) :-
  joint_parent_link(pr2, head_tilt_joint, head_pan_link).

test(joint_parent_link_nonexisting_joint, fail) :-
  joint_parent_link(pr2, bar, head_pan_link).

test(joint_type_pr2_torso_lift_joint) :-
  joint_type(pr2, torso_lift_joint, prismatic).

test(joint_type_pr2_l_wrist_roll_joint) :-
  joint_type(pr2, l_wrist_roll_joint, continuous).

test(joint_type_pr2_r_shoulder_lift_joint) :-
  joint_type(pr2, r_shoulder_lift_joint, revolute).

test(joint_type_pr2_head_plate_frame_joint) :-
  joint_type(pr2, head_plate_frame_joint, fixed).

test(joint_child_link_and_link_parent_joint_pr2_left_arm, forall(
  member(J,[l_shoulder_pan_joint, l_shoulder_lift_joint, l_upper_arm_roll_joint,
        l_elbow_flex_joint, l_forearm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint]))) :-
  joint_child_link(pr2,J,L),
  link_parent_joint(pr2,L,J).

% Observation: The root link of a robot never has a parent joint.
test(link_parent_joint_pr2_root_link, fail) :-
  root_link_name(pr2,L),
  link_parent_joint(pr2, L, _).


test(link_child_joints_pr2_torso_lift_link) :-
  link_child_joints(pr2, torso_lift_link, Joints),
  Joints ==
    [head_pan_joint, imu_joint, l_shoulder_pan_joint, l_torso_lift_side_plate_joint,
     laser_tilt_mount_joint, r_shoulder_pan_joint, r_torso_lift_side_plate_joint].

test(joint_axis_pr2_l_shoulder_pan_joint) :-
  joint_axis(pr2, l_shoulder_pan_joint, [0,0,1]).

test(joint_axis_pr2_fixed_joint, fail) :-
  joint_axis(pr2, head_plate_frame_joint, _).

test(joint_origin_pr2_r_gripper_led_joint) :-
  joint_origin(pr2, r_gripper_led_joint, pose([0.0513, 0.0, 0.0244], [0, 0, 0, 1])).

test(joint_origin_pr2_l_foreaem_cam_optical_frame_joint) :-
  joint_origin(pr2, l_forearm_cam_optical_frame_joint,
               pose([0,0,0],[-0.5, 0.5, -0.5, 0.5])).

test(joint_lower_limit_pr2_l_elbow_flex_joint) :-
  joint_lower_pos_limit(pr2, l_elbow_flex_joint, -2.3213).

test(joint_lower_limit_pr2_l_wrist_roll_joint, fail) :-
  joint_lower_pos_limit(pr2, l_wrist_roll_joint, _).

test(joint_upper_limit_pr2_torso_lift_joint) :-
  joint_upper_pos_limit(pr2, torso_lift_joint, 0.33).

test(joint_upper_limit_pr2_r_forearm_roll_joint, fail) :-
  joint_lower_pos_limit(pr2, r_forearm_roll_joint, _).

test(joint_vel_limit_pr2_r_gripper_joint) :-
  joint_velocity_limit(pr2, r_gripper_joint, 0.2).

test(joint_effort_limit_pr2_head_pan_joint) :-
  joint_effort_limit(pr2, head_pan_joint, 6.0).

test(joint_calibration_rising_pr2_fl_caster_rotation_joint) :-
  joint_calibration_rising(pr2, fl_caster_rotation_joint, -0.785398163397).

test(joint_calibration_rising_pr2_torso_lift_joint, fail) :-
  joint_calibration_rising(pr2, torso_lift_joint, _).

test(joint_calibration_falling_pr2_fl_caster_rotation_joint, fail) :-
  joint_calibration_falling(pr2, fl_caster_rotation_joint, _).

test(joint_calibration_falling_pr2_torso_lift_joint) :-
  joint_calibration_falling(pr2, torso_lift_joint, 0.00475).

test(joint_dynamics_damping_pr2_l_torso_lift_side_plate_joint, fail) :-
  joint_dynamics_damping(pr2, l_torso_lift_side_plate_joint, _).

test(joint_dynamics_friction_pr2_l_torso_lift_side_plate_joint, fail) :-
  joint_dynamics_friction(pr2, l_torso_lift_side_plate_joint, _).

test(joint_dynamics_damping_pr2_head_pan_joint) :-
  joint_dynamics_damping(pr2, head_pan_joint, 0.5).

test(joint_dynamics_friction_pr2_head_pan_joint) :-
  joint_dynamics_friction(pr2, head_pan_joint, 0.0).

test(joint_mimic_joint_name_pr2_torso_lift_joint, fail) :-
  joint_mimic_joint_name(pr2, torso_lift_joint, _).

test(joint_mimic_multiplier_pr2_torso_lift_joint, fail) :-
  joint_mimic_multiplier(pr2, torso_lift_joint, _).

test(joint_mimic_offset_pr2_torso_lift_joint, fail) :-
  joint_mimic_offset(pr2, torso_lift_joint, _).

test(joint_mimic_joint_name_pr2_r_gripper_r_finger_joint) :-
  joint_mimic_joint_name(pr2, r_gripper_r_finger_joint, r_gripper_l_finger_joint).

test(joint_mimic_multiplier_pr2_r_gripper_r_finger_joint) :-
  joint_mimic_multiplier(pr2, r_gripper_r_finger_joint, 1.0).

test(joint_mimic_offset_pr2_r_gripper_r_finger_joint) :-
  joint_mimic_offset(pr2, r_gripper_r_finger_joint, 0.0).

test(joint_safety_lower_limit_pr2_l_upper_arm_joint, fail) :-
  joint_safety_lower_limit(pr2, l_upper_arm_joint, _).

test(joint_safety_upper_limit_pr2_l_upper_arm_joint, fail) :-
  joint_safety_upper_limit(pr2, l_upper_arm_joint, _).

test(joint_safety_kp_pr2_l_upper_arm_joint, fail) :-
  joint_safety_kp(pr2, l_upper_arm_joint, _).

test(joint_safety_kv_pr2_l_upper_arm_joint, fail) :-
  joint_safety_kv(pr2, l_upper_arm_joint, _).

test(joint_safety_lower_limit_pr2_l_elbow_flex_joint) :-
  joint_safety_lower_limit(pr2, l_elbow_flex_joint, -2.1213).

test(joint_safety_upper_limit_pr2_l_elbow_flex_joint) :-
  joint_safety_upper_limit(pr2, l_elbow_flex_joint, -0.15).

test(joint_safety_kp_pr2_l_elbow_flex_joint) :-
  joint_safety_kp(pr2, l_elbow_flex_joint, 100.0).

test(joint_safety_kv_pr2_l_elbow_flex_joint) :-
  joint_safety_kv(pr2, l_elbow_flex_joint, 3.0).

test(link_inertial_origin_pr2_l_gripper_led_frame, fail) :-
  link_inertial_origin(pr2, l_gripper_led_frame, _).

test(link_inertial_origin_pr2_l_elbow_flex_link) :-
  link_inertial_origin(pr2, l_elbow_flex_link, pose([0.01014, 0.00032, -0.01211], [0.0, 0.0, 0.0, 1.0])).

test(link_inertial_mass_pr2_l_gripper_led_frame, fail) :-
  link_inertial_mass(pr2, l_gripper_led_frame, _).

test(link_inertial_mass_pr2_l_elbow_flex_link) :-
  link_inertial_mass(pr2, l_elbow_flex_link, 1.90327).

test(link_inertial_inertia_pr2_l_gripper_led_frame, fail) :-
  link_inertial_inertia(pr2, l_gripper_led_frame, _).

test(link_inertial_mass_pr2_l_elbow_flex_link) :-
  link_inertial_inertia(pr2, l_elbow_flex_link, [0.00346541989, 0.00004066825, 0.00043171614, 0.00441606455, 0.00003968914, 0.00359156824]).

test(link_num_visuals_pr2_r_gripper_led_frame) :-
  link_num_visuals(pr2, r_gripper_led_frame, N),
  N=0.

test(link_num_visuals_pr2_r_gripper_motor_accelerometer_link) :-
  link_num_visuals(pr2, r_gripper_motor_accelerometer_link, N),
  N=1.

test(link_visual_type_pr2_r_gripper_motor_accelerometer_link) :-
  link_visual_type(pr2, r_gripper_motor_accelerometer_link, 0, box).

test(link_visual_type_pr2_r_gripper_motor_slider_link) :-
  link_visual_type(pr2, r_gripper_motor_slider_link, 0, cylinder).

test(link_visual_type_pr2_head_mount_kinect_ir_link) :-
  link_visual_type(pr2, head_mount_kinect_ir_link, 0, sphere).

test(link_visual_type_negative_out_of_bounds, fail) :-
  link_visual_type(pr2, head_mount_kinect_ir_link, -1, _).

test(link_visual_type_positive_out_of_bounds, fail) :-
  link_visual_type(pr2, head_mount_kinect_ir_link, 1, _).

test(link_visual_name_pr2_head_mount_kinect_ir_link, fail) :-
  link_visual_name(pr2, head_mount_kinect_ir_link, 0, _).

test(link_origin_pr2_r_gripper_motor_slider_link) :-
  link_visual_origin(pr2, r_gripper_motor_slider_link, 0,
  pose([0.0,0.0,0.0],[0.7071080798594737, 0.0, 0.0, 0.7071054825112363])).

test(link_visual_geometry_pr2_r_gripper_motor_accelerometer_link) :-
  link_visual_geometry(pr2, r_gripper_motor_accelerometer_link, 0, box([0.001, 0.001, 0.001])).

test(link_visual_geometry_pr2_head_mount_kinect_ir_link) :-
  link_visual_geometry(pr2, head_mount_kinect_ir_link, 0, sphere([0.0005])).

test(link_visual_geometry_pr2_r_gripper_motor_slider_link) :-
  link_visual_geometry(pr2, r_gripper_motor_slider_link, 0, cylinder([0.002, 0.025])).

test(link_visual_geometry_pr2_head_mount_link) :-
  link_visual_geometry(pr2, head_mount_link, 0, mesh('package://pr2_description/meshes/sensors/kinect_prosilica_v0/115x100_swept_back--coarse.STL', [0.001, 0.001, 0.001])).

test(link_visual_geometry_pr2_r_gripper_led_frame, fail) :-
  link_visual_geometry(pr2, r_gripper_led_frame, 0, _).

test(link_visual_geometry_pr2_laser_tilt_mount_link) :-
  link_visual_geometry(pr2, laser_tilt_mount_link, 0, mesh('package://pr2_description/meshes/tilting_laser_v0/tilting_hokuyo.dae', [1.0, 1.0, 1.0])).

test(link_material_name_pr2_laser_tilt_mount_link) :-
  link_material_name(pr2, laser_tilt_mount_link, 0, 'Red').

test(link_material_color_pr2_laser_tilt_mount_link) :-
  link_material_name(pr2, laser_tilt_mount_link, 0, rgba([0.8, 0.0, 0.0, 1.0])).

test(link_material_texture_pr2_laser_tilt_mount_link, fail) :-
  link_material_texture(pr2, laser_tilt_mount_link, 0, _).

test(link_material_name_pr2_fl_caster_rotation_link) :-
  link_material_name(pr2, fl_caster_rotation_link, 0, 'Caster').

test(link_material_color_pr2_fl_caster_rotation_link) :-
  link_material_name(pr2, fl_caster_rotation_link, 0, rgba([0.0, 0.0, 0.0, 1.0])).

test(link_material_texture_pr2_fl_caster_rotation_link) :-
  link_material_texture(pr2, fl_caster_rotation_link, 0, 'package://pr2_description/materials/textures/pr2_caster_texture.png').

test(link_num_collisions_pr2_base_link) :-
  link_num_collisions(pr2, base_link, N),
  N=1.

test(link_num_collisions_pr2_base_laser_link) :-
  link_num_collisions(pr2, base_laser_link, N),
  N=0.

test(link_num_collision_foo_link, fail) :-
  link_num_collisions(pr2, foo, _).

test(link_collision_type_pr2_base_link) :-
  link_collision_type(pr2, base_link, 0, mesh).

test(link_collision_type_pr2_base_laser_link, fail) :-
  link_collision_type(pr2, base_laser_link, 0, _).

test(link_collision_type_pr2_torso_lift_motor_screw_link) :-
  link_collision_type(pr2, torso_lift_motor_screw_link, 0, box).

test(link_collision_type_pr2_head_mount_prosilica_link) :-
  link_collision_type(pr2, head_mount_prosilica_link, 0, sphere).

test(link_collision_type_pr2_fr_caster_r_wheel_link) :-
  link_collision_type(pr2, fr_caster_r_wheel_link, 0, cylinder).

test(link_collision_name_pr2_torso_lift_link) :-
  link_collision_name(pr2, torso_lift_link, 0, torso_lift_collision).

test(link_collision_name_pr2_torso_lift_motor_screw_link, fail) :-
  link_collision_name(pr2, torso_lift_motor_screw_link, 0, _).

test(link_collision_origin_pr2_r_gripper_r_finger_link) :-
  link_collision_origin(pr2, r_gripper_r_finger_link, 0,
  pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])).

test(link_collision_geometry_pr2_r_gripper_r_finger_link) :-
  link_collision_geometry(pr2, r_gripper_r_finger_link, 0,
  mesh("package://pr2_description/meshes/gripper_v0/l_finger.stl", [1.0, 1.0, 1.0])).

test(link_collision_geometry_pr2_head_mount_prosilica_link) :-
  link_collision_geometry(pr2, head_mount_prosilica_link, 0, sphere([0.0005])).

test(link_collision_geometry_pr2_fr_caster_r_wheel_link) :-
  link_collision_geometry(pr2, fr_caster_r_wheel_link, 0, cylinder([0.074792, 0.034])).

test(link_collision_geometry_pr2_torso_lift_motor_screw_link) :-
  link_collision_geometry(pr2, torso_lift_motor_screw_link, 0, box([0.5, 0.7, 0.01])).

test(link_collision_geometry_pr2_base_laser_link, fail) :-
  link_collision_geometry(pr2, base_laser_link, 0, _).

test(urdf_unload) :-
  unload_urdf_file(pr2).

:- end_tests(urdf_parser).
