:- module(model_URDF,
	[ urdf_load(r,+),
	  urdf_load(r,+,+),
	  has_urdf(r,r),
	  has_base_link_name(r,?),
	  has_end_link_name(r,?),
	  has_base_link(r,r),
	  has_end_link(r,r),
	  has_child_link(r,r),
	  has_parent_link(r,r),
	  urdf_set_pose(r,+),
	  urdf_set_pose_to_origin(r,+),
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
	  urdf_joint_friction/3
    ]).

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3, rdf_meta/1 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).
:- use_module(library('lang/query')).

:- use_foreign_library('liburdf_parser.so').

:- tripledb_load('http://knowrob.org/kb/URDF.owl',
    [ namespace(urdf,'http://knowrob.org/kb/urdf.owl#')
    ]).

%% urdf_load(+Object,+File) is semidet.
%
% Same as urdf_load/3 with empty options list.
%
% @Object IRI atom
% @File Path to URDF file
%
urdf_load(Object,File) :-
	urdf_load(Object,File,[]).

%% urdf_load(+Object,+File,+Options) is semidet.
%
% Assert a URDF file as description of an object.
% This is primarly done to associate parts of the object
% to links described in the URDF file.
% Links and joints in the URDF file may further be mapped
% to a RDF model and asserted to the triple store.
%
% @Object IRI atom
% @File Path to URDF file
% @Options List of options
%
urdf_load(Object,File,Options) :-
	urdf_load_file(Object,File),
	% assign urdf name to object
	urdf_root_link(Object,RootLinkName),
	tell(has_base_link_name(Object,RootLinkName)),
	% assign prefix to object
	option(prefix(OptPrefix),Options,''),
	(	OptPrefix=''
	->	true
	;	tell(has_urdf_prefix(Object,OptPrefix))
	),
	% get all the object parts
	findall(X, transitive(triple(Object,dul:hasPart,X)), Parts),
	% set component poses relative to links
	forall(
		(	member(Y,[Object|Parts]),
			has_base_link_name(Y,YName)
		),
		(	atom_concat(OptPrefix,YName,YFrame),
			tell(is_at(Y,[YFrame,[0,0,0],[0,0,0,1]]))
		)
	),
	% optional: load links and joints as rdf objects
	(	option(load_rdf,Options)
	->	load_rdf_(Object,Parts,OptPrefix)
	;	true
	).

%% has_urdf(+Object,+RootObject) is semidet.
%
% True if Object is a part of RootObject, and RootObject is assigned
% to some URDF description.
%
% @Part IRI atom
% @Root IRI atom
%
has_urdf(Root,Root) :-
	urdf_is_loaded(Root),
	!.
has_urdf(Part,Root) :-
	has_part(X,Part),
	has_urdf(X,Root).

%% urdf_set_pose_to_origin(+Object,+Frame) is semidet.
%
% Same as urdf_set_pose/2 but assigns the base of the
% URDF to be located exactly at the frame given in the second argument.
%
% @Object IRI atom
% @Frame URDF frame name atom
%
urdf_set_pose_to_origin(Object,Frame) :-
	urdf_set_pose(Object,[Frame,[0,0,0],[0,0,0,1]]).

%% urdf_set_pose(+Object,+Pose) is semidet.
%
% Assign the initial pose of links described
% in URDF as the current pose of link entities.
% This requires that the *load_rdf* option was used in urdf_load/3.
% The base link of the URDF is assigned to the transform given
% in the second argument.
%
% @Object IRI atom
% @Frame A pose list of frame-position-quaternion
%
urdf_set_pose(Object,Pose) :-
	(	has_urdf_prefix(Object,Prefix)
	;	Prefix=''
	),!,
	% set root link pose
	urdf_root_link(Object,RootLinkName),
	urdf_iri(Object,Prefix,RootLinkName,RootLink),
	tell(is_at(RootLink,Pose)),
	% set pose of other links
	urdf_link_names(Object,Links),
	forall(
		member(LinkName,Links), 
		(	LinkName=RootLinkName
		;	set_link_pose_(Object,Prefix,LinkName)
		)
	).

set_link_pose_(Object,Prefix,LinkName) :-
	urdf_link_parent_joint(Object,LinkName,JointName),
	urdf_joint_origin(Object,JointName,[_,Pos,Rot]),
	urdf_joint_parent_link(Object,JointName,ParentName),
	urdf_iri(Object,Prefix,LinkName,Link),
	atom_concat(Prefix,ParentName,ParentFrame),
	tell(is_at(Link,[ParentFrame,Pos,Rot])).

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
urdf_chain(_,X,X,X) :- !.
urdf_chain(_,_,X,X).
urdf_chain(Object,FromLink,ToLink,Node) :-
	urdf_link_parent_joint(Object,ToLink,Joint),
	urdf_joint_parent_link(Object,Joint,ParentLink),
	urdf_chain(Object,FromLink,ParentLink,Node).

/**************************************/
/********* RDF REPRESENTATION *********/
/**************************************/

%%
urdf_iri(Object,URDFPrefix,Name,IRI) :-
	rdf_split_url(IRIPrefix,_,Object),
	atomic_list_concat([IRIPrefix,URDFPrefix,Name],'',IRI).

%%
:- rdf_meta(joint_type_iri(?,r)).
joint_type_iri(revolute,   urdf:'RevoluteJoint').
joint_type_iri(continuous, urdf:'ContinuousJoint').
joint_type_iri(prismatic,  urdf:'PrismaticJoint').
joint_type_iri(fixed,      urdf:'FixedJoint').
joint_type_iri(floating,   urdf:'FloatingJoint').
joint_type_iri(planar,     urdf:'PlanarJoint').

%%
load_rdf_(Object,Parts,Prefix) :-
	% create link entities
	urdf_link_names(Object,Links),
	forall(member(LinkName,Links), create_link_(Object,Prefix,LinkName,_)),
	% create joint entities
	urdf_joint_names(Object,Joints),
	forall(member(JointName,Joints), create_joint_(Object,Prefix,JointName,_)),
	% associate links to components
	forall(member(Part,Parts), set_links_(Part,Prefix)).

%%
create_link_(Object,Prefix,Name,Link) :-
	urdf_iri(Object,Prefix,Name,Link),
	tell(has_type(Link,urdf:'Link')).

%%
create_joint_(Object,Prefix,Name,Joint) :-
	urdf_iri(Object,Prefix,Name,Joint),
	%%
	urdf_joint_type(Object,Name,Type),
	joint_type_iri(Type,JointType),
	%%
	urdf_joint_child_link(Object,Name,ChildName),
	urdf_joint_parent_link(Object,Name,ParentName),
	urdf_iri(Object,Prefix,ChildName,Child),
	urdf_iri(Object,Prefix,ParentName,Parent),
	%%
	tell([
		has_type(Joint,JointType),
		has_child_link(Joint,Child),
		has_parent_link(Joint,Parent)
	]).

%%
set_links_(Part,Prefix) :-
	(	has_base_link_name(Part,BaseLinkName)
	->	(	urdf_iri(Part,Prefix,BaseLinkName,BaseLink),
			tell(has_base_link(Part,BaseLink))
		)
	;	true
	),!,
	forall(
		has_end_link_name(Part,EndLinkName),
		(	urdf_iri(Part,Prefix,EndLinkName,EndLink),
			tell(has_end_link(Part,EndLink))
		)
	).

/**************************************/
/*********** LANG EXTENSIONS **********/
/**************************************/

%% has_urdf_prefix(?Obj,?Prefix) is semidet.
%
has_urdf_prefix(Obj,Prefix) ?+>
	triple(Obj,urdf:hasNamePrefix,Prefix).

%% has_urdf_name(?Obj,?Name) is semidet.
%
%has_urdf_name(Obj,Name) ?+>
%	triple(Obj,urdf:hasURDFName,Name).

%% has_base_link_name(?Obj,?Name) is semidet.
%
has_base_link_name(Obj,Name) ?+>
	triple(Obj,urdf:hasBaseLinkName,Name).

%% has_end_link_name(?Obj,?Name) is semidet.
%
has_end_link_name(Obj,Name) ?+>
	triple(Obj,urdf:hasEndLinkName,Name).

%% has_base_link(?Obj,?Link) is semidet.
%
has_base_link(Obj,Link) ?+>
	triple(Obj,urdf:hasBaseLink,Link).

%% has_end_link(?Obj,?Link) is semidet.
%
has_end_link(Obj,Link) ?+>
	triple(Obj,urdf:hasEndLink,Link).

%% has_child_link(?Joint,?Link) is semidet.
%
has_child_link(Joint,Link) ?+>
	triple(Joint,urdf:hasChildLink,Link).

%% has_parent_link(?Joint,?Link) is semidet.
%
has_parent_link(Joint,Link) ?+>
	triple(Joint,urdf:hasParentLink,Link).

%%
% TODO: reconsider this
% 
object_shape(Obj,ShapeTerm,Origin) ?>
	has_base_link_name(Obj,BaseName),
	{ get_object_shape_(Obj,BaseName,ShapeTerm,Origin) }.

%%
get_object_shape_(Obj,BaseName,ShapeTerm,[Frame,Pos,Rot]) :-
	has_urdf(Obj,Root),
	(	has_urdf_prefix(Root,Prefix)
	;	Prefix=''
	),!,
	setof(L,
		(	has_end_link_name(Obj,EndName),
			urdf_chain(Root,BaseName,EndName,L)
		),
		LinkNames
	),
	member(LinkName,LinkNames),
	urdf_link_visual_shape(Root,LinkName,
		ShapeTerm,[Name,Pos,Rot]),
	atom_concat(Prefix,Name,Frame).

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
