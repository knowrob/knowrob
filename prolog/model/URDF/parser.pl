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
      load_urdf_file/2,
      unload_urdf_file/1,
      load_urdf_param/2,
      load_urdf_string/2,
      robot_name/2,
      root_link_name/2,
      link_names/2,
      link_parent_joint/3,
      link_child_joints/3,
      link_inertial_origin/3,
      link_inertial_mass/3,
      link_inertial_inertia/3,
      link_num_visuals/3,
      link_visual_type/4,
      link_visual_origin/4,
      link_visual_geometry/4,
      link_material_name/4,
      link_material_color/4,
      link_material_texture/4,
      link_num_collisions/3,
      link_collision_type/4,
      link_collision_name/4,
      link_collision_origin/4,
      link_collision_geometry/4,
      joint_names/2,
      joint_type/3,
      joint_child_link/3,
      joint_parent_link/3,
      joint_axis/3,
      joint_origin/3,
      joint_lower_pos_limit/3,
      joint_upper_pos_limit/3,
      joint_velocity_limit/3,
      joint_effort_limit/3,
      joint_calibration_rising/3,
      joint_calibration_falling/3,
      joint_dynamics_damping/3,
      joint_dynamics_friction/3,
      joint_mimic_joint_name/3,
      joint_mimic_multiplier/3,
      joint_mimic_offset/3,
      joint_safety_lower_limit/3,
      joint_safety_upper_limit/3,
      joint_safety_kp/3,
      joint_safety_kv/3
    ]).

/** <module> Prolog-wrapping for the C++ URDF Parser.

@author Georg Bartels
@license BSD
*/

:- use_foreign_library('liburdf_parser.so').

%% load_urdf_file(+Id,+Filename) is semidet.
%
% Load URDF from disc using global filename.

%% unload_urdf_file(+Id) is semidet.
%
% Unloads a previously loaded URDF.

%% load_urdf_param(+Id,+Param) is semidet.
%
% Load URDF from ROS parameter server using public parameter name.

%% load_urdf_string(+Id,+String) is semidet.
%
% Load URDF from given XML string.

%% robot_name(+Id,-Name) is semidet.
%
% Get the name of the currently loaded robot.

%% root_link_name(+Id,-Name) is semidet.
%
% Get the name of the root link of the robot.

%% link_names(+Id,-Names) is semidet.
%
% Get the list of all link names of this robot.

%% link_parent_joint(+Id,+LinkName, -JointName) is semidet.
%
% Get the name of the parent joint of a link.

%% link_child_joints(+Id,+LinkName, -JointNames) is semidet.
%
% Get the list of joint names of all child joints of a link.

%% link_inertial_origin(+Id,+LinkName, -Origin) is semidet.
%
% Get the inertial origin of a link as a pose w.r.t.
% the origin frame of a link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% link_inertial_mass(+Id,+LinkName, -Mass) is semidet.
%
% Get the inertial mass of a link in kg.

%% link_inertial_inertia(+Id,+LinkName, -InertiaMat) is semidet.
%
% Get the relevant parts of a links inertia matrix w.r.t.
% the origin frame of a link.
%
% Inertia matrices are coded as a list:
% [XX, XY, XZ, YY, YZ, ZZ].
% For an explanation, visit: http://wiki.ros.org/urdf/XML/link

%% link_num_visuals(+Id,+LinkName, -Num) is semidet.
%
% Get the number of visual elements of a link.

%% link_visual_type(+Id,+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular visual element of a link.
% Possible types: box, sphere, cylinder, mesh
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_origin(+Id,+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular visual element of a link
% as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_visual_geometry(+Id,+LinkName, +Index, -Geometry) is semidet.
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

%% link_material_name(+Id,+LinkName, +Index, -Name) is semidet.
%
% Get the name of a material of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_color(+Id,+LinkName, +Index, -Color) is semidet.
%
% Get the color of a material of a particular visual element
% of a link.
%
% Colors are coded as compound terms: rgba([R, G, B, A]).
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_material_texture(+Id,+LinkName, +Index, -FileName) is semidet.
%
% Get the filename of a texture of a particular visual element
% of a link.
%
% Note: Links can have several visuals elements, the first
% one has index 0.

%% link_num_collisions(+Id,+LinkName, -Num) is semidet.
%
% Get the number of collision elements of a link.

%% link_collision_type(+Id,+LinkName, +Index, -Type) is semidet.
%
% Get the type of a particular collision element of a link.
% Possible types: box, sphere, cylinder, and mesh.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_name(+Id,+LinkName, +Index, -Name) is semidet.
%
% Get the name of a particular collision element of a link.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_origin(+Id,+LinkName, +Index, -Origin) is semidet.
%
% Get the origin of a particular collision element of a link,
% expressed as a pose w.r.t. to the link frame.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.
%
% Note: Links can have several collision elements, the first
% one has index 0.

%% link_collision_geometry(+Id,+LinkName, +Index, -Geometry) is semidet.
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

%% joint_names(+Id,-Names) is semidet.
%
% Get the list of joint names of the currently loaded robot.

%% joint_type(+Id,+JointName, Type) is semidet.
%
% Get the type of a joint.
% Possible types: revolute, prismatic, continuous, fixed,
% floating, planar, and unknown.

%% joint_child_link(+Id,+JointName, -LinkName) is semidet.
%
% Get the name of the link of a joint.

%% joint_parent_link(+Id,+JointName, -LinkName) is semidet.
%
% Get the name the parent link of a joint.

%% joint_axis(+Id,+JointName, -Axis) is semidet.
%
% Get the axis of a joint, expressed as a list [X, Y, Z].

%% joint_origin(+Id,+JointName, -Origin) is semidet.
%
% Get the origin of a joint, expressed as a pose
% w.r.t. the link frame of its parent link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% joint_lower_pos_limit(+Id,+JointName, -Lower) is semidet.
%
% Read the lower position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.


%% joint_upper_pos_limit(+Id,+JointName, -Upper) is semidet.
%
% Read the upper position limit of a joint.
%
% Note: Only valid for prismatic and revolute joints.

%% joint_velocity_limit(+Id,+JointName, -VelLimit) is semidet.
%
% Read the velocity limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_effort_limit(+Id,+JointName, -EffLimit) is semidet.
%
% Read the effort limit of a joint.
%
% Note: Only valid for prismatic, revolute, and continuous joints.

%% joint_calibration_rising(+Id,+JointName, -Rising) is semidet.
%
% Read the rising reference position of a joint.

%% joint_calibration_falling(+Id,+JointName, -Falling) is semidet.
%
% Read the falling reference position of a joint.

%% joint_dynamics_damping(+Id,+JointName, -Damping) is semidet.
%
% Read the damping value of a joint.

%% joint_dynamics_friction(+Id,+JointName, -Friction) is semidet.
%
% Get the static friction value of a joint.

%% joint_mimic_joint_name(+Id,+JointName, -MimickedJointName) ist semidet.
%
% Get the name of a joint that a mimic joint mimicks.

%% joint_mimic_multiplier(+Id,+JointName, -Multiplier) is semidet.
%
% Get the multiplication factor of a mimic joint.

%% joint_mimic_offset(+Id,+JointName, -Offset) is semidet.
%
% Get the offset value of a mimic joint.

%% joint_safety_lower_limit(+Id,+JointName, -Lower) is semidet.
%
% Get the lower position limit of the safety controller
% of a joint.

%% joint_safety_upper_limit(+Id,+JointName, -Upper) is semidet.
%
% Get the upper position limit of the safety controller
% of a joint.

%% joint_safety_kp(+Id,+JointName, -Kp) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits

%% joint_safety_kv(+Id,+JointName, -Kv) is semidet.
%
% Get the relation between position and velocity
% limits of the safety controller of a joint. For
% more details, visit:
% http://wiki.ros.org/pr2_controller_manager/safety_limits
