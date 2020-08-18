:- module(model_URDF,
	[ urdf_load_file/2,
	  urdf_is_loaded/1,
	  urdf_unload_file/1,
	  urdf_robot_name/2,
	  urdf_link_names/2,
	  urdf_joint_names/2,
	  urdf_root_link/2,
	  urdf_link_parent_joint/3,
	  urdf_link_child_joints/3,
	  urdf_link_inertial/5,
	  urdf_link_visual_shape/4,
	  urdf_link_collision_shape/4,
	  urdf_joint_type/3,
	  urdf_joint_child_link/3,
	  urdf_joint_parent_link/3,
	  urdf_joint_axis/3,
	  urdf_joint_origin/3,
	  urdf_joint_calibration_falling/3,
	  urdf_joint_calibration_rising/3,
	  urdf_joint_hard_limits/5,
	  urdf_joint_soft_limits/5,
	  urdf_joint_damping/3,
	  urdf_joint_friction/3,
	  urdf_chain/4
    ]).

%:- use_module(library('semweb/rdf_db'),
%	[ rdf_split_url/3, rdf_meta/1 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).
%:- use_module(library('lang/query')).
%:- use_module(library('lang/terms/holds')).

:- use_foreign_library('liburdf_parser.so').

:- tripledb_load('http://knowrob.org/kb/URDF.owl',
    [ namespace(urdf,'http://knowrob.org/kb/urdf.owl#')
    ]).

%%
%
urdf_link_visual_shape(Object,Link,ShapeTerm,Origin) :-
	urdf_link_num_visuals(Object,Link,Count),
	N is Count - 1,
	between(0,N,Index),
	urdf_link_nth_visual_shape(Object,Link,Index,ShapeTerm,Origin).

%%
%
urdf_link_collision_shape(Object,Link,ShapeTerm,Origin) :-
	urdf_link_num_collisions(Object,Link,Count),
	N is Count - 1,
	between(0,N,Index),
	urdf_link_nth_collision_shape(Object,Link,Index,ShapeTerm,Origin).

%%
%
urdf_chain(_,X,X,X) :- !.
urdf_chain(_,_,X,X).
urdf_chain(Object,FromLink,ToLink,Node) :-
	urdf_link_parent_joint(Object,ToLink,Joint),
	urdf_joint_parent_link(Object,Joint,ParentLink),
	urdf_chain(Object,FromLink,ParentLink,Node).

/**************************************/
/********** FOREIGN LIBRARY ***********/
/**************************************/

%% urdf_load_file(+Object,+Filename) is semidet.
%
% Load URDF from disc using global filename.

%% urdf_unload_file(+Object) is semidet.
%
% Unloads a previously loaded URDF.

%% urdf_robot_name(+Object,-Name) is semidet.
%
% Get the name of the currently loaded robot.

%% urdf_root_link(+Object,-Name) is semidet.
%
% Get the name of the root link of the robot.

%% urdf_link_parent_joint(+Object,+LinkName, -JointName) is semidet.
%
% Get the name of the parent joint of a link.

%% urdf_link_child_joints(+Object,+LinkName, -JointNames) is semidet.
%
% Get the list of joint names of all child joints of a link.

%% urdf_link_inertial(+Object, +LinkName, -Inertia, -Origin, -Mass) is semidet.
%
% Get the inertial origin of a link as a pose w.r.t.
% the origin frame of a link.
%
% Inertia matrices are coded as a list:
% [XX, XY, XZ, YY, YZ, ZZ].
% For an explanation, visit: http://wiki.ros.org/urdf/XML/link

%% urdf_joint_type(+Object,+JointName, Type) is semidet.
%
% Get the type of a joint.
% Possible types: revolute, prismatic, continuous, fixed,
% floating, planar, and unknown.

%% urdf_joint_child_link(+Object,+JointName, -LinkName) is semidet.
%
% Get the name of the link of a joint.

%% urdf_joint_parent_link(+Object,+JointName, -LinkName) is semidet.
%
% Get the name the parent link of a joint.

%% urdf_joint_axis(+Object,+JointName, -Axis) is semidet.
%
% Get the axis of a joint, expressed as a list [X, Y, Z].

%% urdf_joint_origin(+Object,+JointName, -Origin) is semidet.
%
% Get the origin of a joint, expressed as a pose
% w.r.t. the link frame of its parent link.
%
% Poses are coded as a compound term: pose([X,Y,Z],[QX,QY,QZ,QW]),
% with the orientation represented as Quaternion.

%% urdf_joint_calibration(+Object, +JointName, -Falling, -Rising) is semidet.
%
% Read the falling and rising reference positions of a joint.

%% urdf_joint_damping(+Object,+JointName, -Damping) is semidet.
%
% Read the damping value of a joint.

%% urdf_joint_friction(+Object,+JointName, -Friction) is semidet.
%
% Get the static friction value of a joint.

%% urdf_joint_hard_limits(+Object, +JointName, -PosLimits, -VelMax, -EffMax) is semidet.
%
% Read the hard limits of a joint.

%% urdf_joint_soft_limits(+Object, +JointName, -PosLimits, -KP, -KV) is semidet.
%
% Read the soft limits of a joint.
