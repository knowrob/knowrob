:- module(ros_urdf,
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
	  urdf_link_visual_shape/6,
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
	  is_urdf_link(r),
	  is_urdf_joint(r),
	  urdf_init/0
    ]).

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3, rdf_meta/1 ]).
:- use_module(library('lang/db'),
    [ load_owl/2 ]).
:- use_module(library('lang/query')).
:- use_module(library('utility/url'), [ url_resolve/2 ]).
:- use_module(library('utility/filesystem'), [ path_concat/3 ]).
:- use_module(library(http/http_client)).
:- use_module(library('ros/tf/tf'), [ tf_mem_set_pose/3 ]).

:- use_foreign_library('liburdf_parser.so').

:- load_owl('http://knowrob.org/kb/URDF.owl',
    [ namespace(urdf,'http://knowrob.org/kb/urdf.owl#')
    ]).

%% has_urdf(+Object,+RootObject) is semidet.
%
% True if Object is a part of RootObject, and RootObject is assigned
% to some URDF description.
%
% @param Part IRI atom
% @param Root IRI atom
%
:- dynamic has_urdf/2.
:- dynamic urdf_server/1.
:- dynamic urdf_prefix/2.

%%
% Initialize prefix for downloading URDF via HTTP
%%
urdf_server_init :-
	once((
		getenv('KNOWROB_URDF_SERVER', URL)
	;	URL='http://neem-data.informatik.uni-bremen.de/data/kinematics/'
	)),
	retractall(urdf_server(_)),
	assertz(urdf_server(URL)).
:- urdf_server_init.

%%
%
%
urdf_init :-
	retractall(has_urdf(_,_)),
	retractall(urdf_prefix(_,_)),
	forall(
		has_kinematics_file(Object,Identifier,'URDF'),
		urdf_init(Object,Identifier)
	).

urdf_init(Object,_) :-
	has_urdf(Object,_),
	!.

urdf_init(Object,Identifier) :-
	urdf_server(DATA_URL),
	atomic_list_concat([Identifier,urdf],'.',Filename),
	path_concat(DATA_URL,Filename,URL),
	% get XML data
	(	http_get(URL,XML_data,[]) -> true
	;	(	log_warn(urdf(download_failed(Object,URL))),
			fail
		)
	),
	% parse data
	(	urdf_load_xml(Object,XML_data) -> true
	;	(	log_warn(urdf(parsing_failed(Object,URL))),
			fail
		)
	),
	% create has_urdf facts
	forall(
		(	Y=Object
		;	kb_call(triple(Object,transitive(dul:hasComponent),Y))
		),
		(	has_urdf(Y,Object) -> true
		;	assertz(has_urdf(Y,Object))
		)
	),
	log_info(urdf(initialized(Object,Identifier))),
	!.

%% urdf_load(+Object,+File) is semidet.
%
% Same as urdf_load/3 with empty options list.
%
% @param Object IRI atom
% @param File Path to URDF file
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
% @param Object IRI atom
% @param File Path to URDF file
% @param Options List of options
%
urdf_load(Object,URL,Options) :-
	(	url_resolve(URL,Resolved)
	->	true
	;	Resolved=URL 
	),
	urdf_load_file(Object,Resolved),
	% create IO and IR objects in triple store
	file_base_name(Resolved,FileName),
	file_name_extension(Identifier,_,FileName),
	kb_project(has_kinematics_file(Object,Identifier,'URDF')),
	% assign urdf name to object
	% TODO: only do this on first load
	urdf_root_link(Object,RootLinkName),
	kb_project(has_base_link_name(Object,RootLinkName)),
	% assign prefix to object
	% TODO: only do this on first load
	option(prefix(OptPrefix),Options,''),
	(	OptPrefix=''
	->	true
	;	kb_project(has_urdf_prefix(Object,OptPrefix))
	),
	% get all the object parts
	findall(X, kb_call(triple(Object,transitive(dul:hasComponent),X)), Parts),
	% set component poses relative to links
	forall(
		(	member(Y,[Object|Parts]),
			has_base_link_name(Y,YName)
		),
		(	atom_concat(OptPrefix,YName,YFrame),
			% update tf memory
			rdf_split_url(_,OName,Y),
			tf_mem_set_pose(OName, [YFrame,[0,0,0],[0,0,0,1]], 0),
			%
			assertz(has_urdf(Y,Object))
		)
	),
	% optional: load links and joints as rdf objects
	(	option(load_rdf,Options)
	->	load_rdf_(Object,Parts,OptPrefix)
	;	true
	).

%% urdf_set_pose_to_origin(+Object,+Frame) is semidet.
%
% Same as urdf_set_pose/2 but assigns the base of the
% URDF to be located exactly at the frame given in the second argument.
%
% @param Object IRI atom
% @param Frame URDF frame name atom
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
% @param Object IRI atom
% @param Frame A pose list of frame-position-quaternion
%
urdf_set_pose(Object,Pose) :-
	(	has_urdf_prefix(Object,Prefix)
	;	Prefix=''
	),!,
	% set root link pose
	urdf_root_link(Object,RootLinkName),
	urdf_iri(Object,Prefix,RootLinkName,RootLink),
	% update tf memory
	rdf_split_url(_,RootFrame,RootLink),
	tf_mem_set_pose(RootFrame,Pose,0),
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
	% update tf memory
	rdf_split_url(_,LinkFrame,Link),
	tf_mem_set_pose(LinkFrame, [ParentFrame,Pos,Rot], 0).

%%
%
urdf_link_visual_shape(Object,Link,ShapeTerm,Origin,MaterialTerm,ShapeID) :-
	urdf_link_num_visuals(Object,Link,Count),
	N is Count - 1,
	between(0,N,Index),
	atom_concat(Link,Index,ShapeID),
	urdf_link_nth_visual_shape(Object,Link,Index,ShapeTerm,Origin,MaterialTerm).

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
	%%
	% FIXME: BUG: providing a prefix with '/' breaks rdf_split_url
	%             for link individuals!!
	%%
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
	kb_project(has_type(Link,urdf:'Link')).

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
	kb_project([
		has_type(Joint,JointType),
		has_child_link(Joint,Child),
		has_parent_link(Joint,Parent)
	]).

%%
set_links_(Part,Prefix) :-
	(	has_base_link_name(Part,BaseLinkName)
	->	(	urdf_iri(Part,Prefix,BaseLinkName,BaseLink),
			kb_project(has_base_link(Part,BaseLink))
		)
	;	true
	),!,
	forall(
		has_end_link_name(Part,EndLinkName),
		(	urdf_iri(Part,Prefix,EndLinkName,EndLink),
			kb_project(has_end_link(Part,EndLink))
		)
	).

/**************************************/
/*********** LANG EXTENSIONS **********/
/**************************************/

%% is_urdf_link(?Entity) is semidet.
%
is_urdf_link(Entity) ?+>
	has_type(Entity, urdf:'Link').

%% is_urdf_joint(?Entity) is semidet.
%
is_urdf_joint(Entity) ?+>
	has_type(Entity, urdf:'Joint').

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
model_SOMA:object_shape(Obj,ShapeID,ShapeTerm,Origin,MaterialTerm) :-
	object_shape_urdf(Obj,ShapeID,ShapeTerm,Origin,MaterialTerm).

%%
object_shape_urdf(Obj,ShapeID,ShapeTerm,Origin,MaterialTerm) :-
	var(Obj),!,
	kb_call((
		has_base_link_name(Obj,BaseName),
		has_end_link_name(Obj,EndName)
	)),
	has_urdf(Obj,Root),
	get_object_shape_(
		Root, BaseName, EndName,
		ShapeID, ShapeTerm, Origin, MaterialTerm).

object_shape_urdf(Obj,ShapeID,ShapeTerm,Origin,MaterialTerm) :-
	%nonvar(Obj),
	has_urdf(Obj,Root),
	kb_call((
		has_base_link_name(Obj,BaseName),
		has_end_link_name(Obj,EndName)
	)),
	get_object_shape_(
		Root, BaseName, EndName,
		ShapeID, ShapeTerm, Origin, MaterialTerm).

%%
get_object_shape_(Root,BaseName,EndName,
		ShapeID,ShapeTerm,[Frame,Pos,Rot],MaterialTerm) :-
	% read prefix from root entity of URDF
	once((urdf_prefix(Root,Prefix);(
		(has_urdf_prefix(Root,Prefix);Prefix=''),
		assertz(urdf_prefix(Root,Prefix))
	))),
	urdf_catch(urdf_chain(Root,BaseName,EndName,LinkName)),
	urdf_catch(urdf_link_visual_shape(Root,LinkName,
		ShapeTerm,[Name,Pos,Rot],MaterialTerm,ShapeID)),
	atom_concat(Prefix,Name,Frame).

%%
urdf_catch(Goal) :-
	catch(
		call(Goal),
		urdf_error(Err),
		(	print_message(error,urdf_error(Err)),
			fail
		)
	).

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
