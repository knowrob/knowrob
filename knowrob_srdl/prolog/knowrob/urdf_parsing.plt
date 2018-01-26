%%
%% Copyright (C) 2018 by Georg Bartels
%%
%% This file contains tests for the URDF parsing tools in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- begin_tests(urdf_parsing).

:- use_module('urdf_parsing').
:- use_module(library('roscpp')).


test(foo) :-
  findall(X, foo(X), Xs),
  Xs == [foo].

test(bar) :-
  findall(X, bar(X), Xs),
  Xs == [bar].

%% test(foo_bar) :-
%%   findall(X, foo_bar(X), Xs),
%%   Xs == [foo, bar].

test(load_urdf_pr2) :-
  ros_package_path('knowrob_srdl', X),
  atom_concat(X, '/urdf/pr2.urdf', Filename),
  load_urdf(Filename).

test(load_non_existent_urdf, fail) :-
  load_urdf('foo.urdf').

%% TODO: setUp and cleanUP functions
test(roo_link_name_pr2) :-
  root_link_name(base_footprint).

test(joint_names_pr2) :-
  joint_names(Names),
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
  link_names(Names),
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
  joint_child_link(torso_lift_joint, torso_lift_link).

test(joint_child_link_pr2_l_shoulder_pan_joint) :-
  joint_child_link(l_shoulder_pan_joint, l_shoulder_pan_link).

test(joint_child_link_pr2_nonexisting_joint, fail) :-
  joint_child_link(foo, l_shoulder_pan_link).

test(joint_parent_link_pr2_r_elbow_flex_joint) :-
  joint_parent_link(r_elbow_flex_joint, r_upper_arm_roll_link).

test(joint_parent_link_pr2_head_tilt_joint) :-
  joint_parent_link(head_tilt_joint, head_pan_link).

test(joint_parent_link_nonexisting_joint, fail) :-
  joint_parent_link(bar, head_pan_link).

test(joint_type_pr2_torso_lift_joint) :-
  joint_type(torso_lift_joint, prismatic).

test(joint_type_pr2_l_wrist_roll_joint) :-
  joint_type(l_wrist_roll_joint, continuous).

test(joint_type_pr2_r_shoulder_lift_joint) :-
  joint_type(r_shoulder_lift_joint, revolute).

test(joint_type_pr2_head_plate_frame_joint) :-
  joint_type(head_plate_frame_joint, fixed).

test(joint_child_link_and_link_parent_joint_pr2_left_arm, forall(
  member(J,[l_shoulder_pan_joint, l_shoulder_lift_joint, l_upper_arm_roll_joint,
        l_elbow_flex_joint, l_forearm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint]))) :-
  joint_child_link(J,L),
  link_parent_joint(L,J).

% Observation: The root link of a robot never has a parent joint.
test(link_parent_joint_pr2_root_link, fail) :-
  root_link_name(L),
  link_parent_joint(L, _).


test(link_child_joints_pr2_torso_lift_link) :-
  link_child_joints(torso_lift_link, Joints),
  Joints ==
    [head_pan_joint, imu_joint, l_shoulder_pan_joint, l_torso_lift_side_plate_joint,
     laser_tilt_mount_joint, r_shoulder_pan_joint, r_torso_lift_side_plate_joint].

test(joint_axis_pr2_l_shoulder_pan_joint) :-
  joint_axis(l_shoulder_pan_joint, [0,0,1]).

test(joint_axis_pr2_fail_fixed_joint, fail) :-
  joint_axis(head_plate_frame_joint, _).

:- end_tests(urdf_parsing).
