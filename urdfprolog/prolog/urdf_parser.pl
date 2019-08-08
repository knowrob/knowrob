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

:- module(urdf_parser,
    [
      load_urdf_file/1,
      load_urdf_param/1,
      load_urdf_string/1,
      robot_name/1,
      root_link_name/1,
      link_names/1,
      link_parent_joint/2,
      link_child_joints/2,
      link_inertial_origin/2,
      link_inertial_mass/2,
      link_inertial_inertia/2,
      link_num_visuals/2,
      link_visual_type/3,
      link_visual_origin/3,
      link_visual_geometry/3,
      link_material_name/3,
      link_material_color/3,
      link_material_texture/3,
      link_num_collisions/2,
      link_collision_type/3,
      link_collision_name/3,
      link_collision_origin/3,
      link_collision_geometry/3,
      joint_names/1,
      joint_type/2,
      joint_child_link/2,
      joint_parent_link/2,
      joint_axis/2,
      joint_origin/2,
      joint_lower_pos_limit/2,
      joint_upper_pos_limit/2,
      joint_velocity_limit/2,
      joint_effort_limit/2,
      joint_calibration_rising/2,
      joint_calibration_falling/2,
      joint_dynamics_damping/2,
      joint_dynamics_friction/2,
      joint_mimic_joint_name/2,
      joint_mimic_multiplier/2,
      joint_mimic_offset/2,
      joint_safety_lower_limit/2,
      joint_safety_upper_limit/2,
      joint_safety_kp/2,
      joint_safety_kv/2
    ]).

/** <module> Prolog-wrapping for the C++ URDF Parser.

@author Georg Bartels
@license BSD
*/

:- use_foreign_library('liburdf_parser.so').

%% ros_param_get_string(+Key,-Value) is semidet.
%
% Read parameter from ROS parameter server.

%% ros_info(+Msg) is det.
%
% Debug via ROS master.

%% load_urdf_file(+Filename) is semidet.
%
% Load URDF from disc using global filename.

%% load_urdf_param(+Param) is semidet.
%
% Load URDF from ROS parameter server using public parameter name.

%% load_urdf_string(+String) is semidet.
%
% Load URDF from given XML string.

%% robot_name(-Name) is semidet.
%
% Get the name of the currently loaded robot.

%% root_link_name(-Name) is semidet.
%
% Get the name of the root link of the robot.

%% link_names(-Names) is semidet.
%
% Get the list of all link names of this robot.

%% link_parent_joint(+LinkName, -JointName) is semidet.
%
% Get the name of the parent joint of a link.

%% link_child_joints(+LinkName, -JointNames) is semidet.
%
% Get the list of joint names of all child joints of a link.

%% link_inertial_origin(+LinkName, -Origin) is semidet.
%
% Get the inertial origin of a link as a pose w.r.t.
% the origin frame of a link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% link_inertial_mass(+LinkName, -Mass) is semidet.
%
% Get the inertial mass of a link in kg.

%% link_inertial_inertia(+LinkName, -InertiaMat) is semidet.
%
% Get the relevant parts of a links inertia matrix w.r.t.
% the origin frame of a link.
%
% Inertia matrices are coded as a list:
% [XX, XY, XZ, YY, YZ, ZZ].
% For an explanation, visit: http://wiki.ros.org/urdf/XML/link

%% link_num_visuals(+LinkName, -Num) is semidet.
%
% Get the number of visual elements of a link.

%% link_visual_type(+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular visual element of a link.
% Possible types: box, sphere, cylinder, mesh
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_origin(+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular visual element of a link
% as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_geometry(+LinkName, +Index, -Geometry) is semidet.
%
% Get the geometry of a particular visual element of a link.
%
% Depending on the type of visual element, the geometry is
% coded differently:
% - SPHERE: sphere([Radius])
% - CYLINDER: cylinder([Radius, Length])
% - BOX: box([X, Y, Z])
% - MESH: mesh(Filename, [ScaleX, ScaleY, ScaleZ])
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_name(+LinkName, +Index, -Name) is semidet.
%
% Get the name of a material of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_color(+LinkName, +Index, -Color) is semidet.
%
% Get the color of a material of a particular visual element
% of a link.
%
% Colors are coded as compound terms: rgba([R, G, B, A]).
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_texture(+LinkName, +Index, -FileName) is semidet.
%
% Get the filename of a texture of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_num_collisions(+LinkName, -Num) is semidet.
%
% Get the number of collision elements of a link.

%% link_collision_type(+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular collision element of a link.
% Possible types: box, sphere, cylinder, and mesh.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_name(+LinkName, +Index, -Name) is semidet.
%
% Get the name of a particular collision element of a link.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_origin(+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular collision element of a link,
% expressed as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_geometry(+LinkName, +Index, -Geometry) is semidet.
%
% Get the geometry of a particular collision element of a link.
%
% Depending on the type of collision element, the geometry is
% coded differently:
% - SPHERE: sphere([Radius])
% - CYLINDER: cylinder([Radius, Length])
% - BOX: box([X, Y, Z])
% - MESH: mesh(Filename, [ScaleX, ScaleY, ScaleZ])
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% joint_names(-Names) is semidet.
%
% Get the list of joint names of the currently loaded robot.

%% joint_type(+JointName, Type) is semidet.
%
% Get the type of a joint.
% Possible types: revolute, prismatic, continuous, fixed,
% floating, planar, and unknown.

%% joint_child_link(+JointName, -LinkName) is semidet.
%
% Get the name of the link of a joint.

%% joint_parent_link(+JointName, -LinkName) is semidet.
%
% Get the name the parent link of a joint.

%% joint_axis(+JointName, -Axis) is semidet.
%
% Get the axis of a joint, expressed as a list [X, Y, Z].

%% joint_origin(+JointName, -Origin) is semidet.
%
% Get the origin of a joint, expressed as a pose
% w.r.t. the link frame of its parent link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% joint_lower_pos_limit(+JointName, -Lower) is semidet.
%
% Read the lower position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.


%% joint_upper_pos_limit(+JointName, -Upper) is semidet.
%
% Read the upper position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.

%% joint_velocity_limit(+JointName, -VelLimit) is semidet.
%
% Read the velocity limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_effort_limit(+JointName, -EffLimit) is semidet.
%
% Read the effort limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_calibration_rising(+JointName, -Rising) is semidet.
%
% Read the rising reference position of a joint.

%% joint_calibration_falling(+JointName, -Falling) is semidet.
%
% Read the falling reference position of a joint.

%% joint_dynamics_damping(+JointName, -Damping) is semidet.
%
% Read the damping value of a joint.

%% joint_dynamics_friction(+JointName, -Friction) is semidet.
%
% Get the static friction value of a joint.

%% joint_mimic_joint_name(+JointName, -MimickedJointName) ist semidet.
%
% Get the name of a joint that a mimic joint mimicks.

%% joint_mimic_multiplier(+JointName, -Multiplier) is semidet.
%
% Get the multiplication factor of a mimic joint.

%% joint_mimic_offset(+JointName, -Offset) is semidet.
%
% Get the offset value of a mimic joint.

%% joint_safety_lower_limit(+JointName, -Lower) is semidet.
%
% Get the lower position limit of the safety controller
% of a joint.

%% joint_safety_upper_limit(+JointName, -Upper) is semidet.
%
% Get the upper position limit of the safety controller
% of a joint.

%% joint_safety_kp(+JointName, -Kp) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits

%% joint_safety_kv(+JointName, -Kv) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits
