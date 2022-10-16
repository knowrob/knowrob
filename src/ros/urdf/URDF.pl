:- module(ros_urdf,
	[ urdf_load(r,+),
	  urdf_load(r,+,+),
	  has_urdf(r,r),
	  has_urdf_name(r,?),
	  has_base_link(r,r),
	  has_end_link(r,r),
	  has_child_link(r,r),
	  has_parent_link(r,r),
	  urdf_pose(r,-,-),
	  urdf_pose_rel(r,+,+,-,-),
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
assert_has_urdf(Object, URDFRoot) :-
	transaction((
		retractall(has_urdf(Object,_)),
		asserta(has_urdf(Object, URDFRoot)))).

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
	;	(	log_warning(urdf(download_failed(Object,URL))),
			fail
		)
	),
	% parse data
	(	urdf_load_xml(Object,XML_data) -> true
	;	(	log_warning(urdf(parsing_failed(Object,URL))),
			fail
		)
	),
	% lookup urdf prefix from triple store
	(	has_urdf_prefix(Object,Prefix)
	->	true
	;	Prefix=''
	),
	assertz(urdf_prefix(Object,Prefix)),
	% create has_urdf facts
	forall(
		(	Y=Object
		;	kb_call(triple(Object,transitive(dul:hasComponent),Y))
		),
		assert_has_urdf(Y, Object)
	),
	urdf_link_names(Object, Links),
	forall(member(LinkName, Links), (
		urdf_iri(Object, LinkName, Link),
		assert_has_urdf(Link, Object)
	)),
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
	%urdf_root_link(Object,RootLinkName),
	% remember prefix
	option(prefix(OptPrefix),Options,''),
	assertz(urdf_prefix(Object, OptPrefix)),
	% create some facts in the triple store
	file_base_name(Resolved,FileName),
	file_name_extension(Identifier,_,FileName),
	kb_project([
		has_kinematics_file(Object, Identifier, 'URDF'),
		has_urdf_prefix(Object, OptPrefix)
	]),
	% get all the object components
	findall(string(X), (
		(Object=X ; triple(Object,transitive(dul:hasComponent),X)),
		assertz(has_urdf(X,Object))
	), Parts),
	% NOTE: some composite components may not have their own frame in the URDF file.
	%       in this case, hasBaseLink can be used instead of hasComponent such that the
	%       base link frame is also used for the composite object (using identity transform between them).
	% 
	forall((	has_base_link(in(array(Parts)) -> Y, YBase),
			rdf_split_url(_,YName,YBase),
			rdf_split_url(_,OName,Y),
			OName \== YName,
			atom_concat(OptPrefix,YName,YFrame)
		),
		% update tf memory
		tf_mem_set_pose(OName, [YFrame,[0,0,0],[0,0,0,1]], 0)
	),
	% optional: load links and joints as rdf objects
	(	option(load_rdf,Options)
	->	load_rdf_(Object, Parts)
	;	true
	).

load_rdf_(Object, Parts) :-
	% compute list of URDF link IRIs
	urdf_link_names(Object,LinkNames),
	findall(Y,(member(X,LinkNames),urdf_iri(Object,X,Y)),Links),
	% assert has_urdf fact for each link
	forall(member(Link,Links), assert_has_urdf(Link,Object)),
	findall(Fact,
		(	load_rdf_(Object,Links,Fact)
		% add hasLink assertions if the enttity has
		% hasBaseLink and hasEndLink assertions.
		;	(kb_call((
				has_base_link(in(array(Parts)) -> Y, YBase),
				has_end_link(Y, YEnd)
			)),
			rdf_split_url(_,BaseName,YBase),
			rdf_split_url(_,EndName,YEnd),
			urdf_chain(Object,BaseName,EndName,X),
			X \== BaseName,
			X \== EndName,
			urdf_iri(Object, X, XLink),
			Fact = triple(Y, 'http://knowrob.org/kb/urdf.owl#hasLink', XLink))
		),
		Facts),
	list_to_set(Facts,Facts_unique),
	kb_project(Facts_unique).

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


%% urdf_pose(+Object, ?LinkFrame, ?Pose) is semidet.
%
% True if Pose is the pose of the link of Object
% relative to its parent link.
%
% @param Object IRI atom
% @param ObjectFrame The frame associated to the base link of Object
% @param Pose A pose list of frame-position-quaternion
%
urdf_pose(Object, ObjectFrame, Pose) :-
	has_urdf(Object, URDFObject),
	rdf_split_url(_, LinkName, Object),
	urdf_prefix(URDFObject, Prefix),
	urdf_pose1(URDFObject, Prefix, LinkName, _, Pose),
	atom_concat(Prefix, LinkName, ObjectFrame).

%% urdf_pose_rel(+Object, -ObjFrame, -AbsPose) is semidet.
%
% True if AbsPose is the pose of the link of Object
% relative to the root link of the associated URDF.
%
% @param Object IRI atom
% @param ObjectFrame The frame associated to the base link of Object
% @param Pose A pose list of frame-position-quaternion
%
urdf_pose_rel(Object, LinkName, LinkName, LinkFrame, [LinkFrame,[0,0,0],[0,0,0,1]]) :-
	!,
	has_urdf(Object, URDFRoot),
	urdf_prefix(URDFRoot, Prefix),
	atom_concat(Prefix, LinkName, LinkFrame).
	
urdf_pose_rel(Object, LinkName, RootName, LinkFrame, LinkPose) :-
	has_urdf(Object, URDFRoot),
	urdf_prefix(URDFRoot, Prefix),
	urdf_pose1(URDFRoot, Prefix, LinkName, ParentName, DirectPose),
	urdf_pose_rel1(URDFRoot, Prefix, RootName, ParentName, DirectPose, LinkPose),
	atom_concat(Prefix, LinkName, LinkFrame).

urdf_pose1(URDFObject, Prefix, LinkName, ParentName, [ParentFrame,Pos,Rot]) :-
	%urdf_iri(URDFObject, LinkName, Link),
	urdf_link_parent_joint(URDFObject, LinkName, JointName),
	urdf_joint_origin(URDFObject, JointName, [_,Pos,Rot]),
	urdf_joint_parent_link(URDFObject, JointName, ParentName),
	atom_concat(Prefix, ParentName, ParentFrame).

urdf_pose_rel1(_, _, RootName, RootName, RelPose, RelPose) :-
	% root link reached
	!.

urdf_pose_rel1(URDFObject, Prefix, RootName, ChildName,
		[ChildFrame,PosOX,RotOX], RelPose) :-
	urdf_pose1(URDFObject, Prefix, ChildName,
		ParentName, [ParentFrame,PosXP,RotXP]),
	!,
	transform_multiply(
		% in: object pose relative to some link x
		[obj_frame, ChildFrame, PosOX, RotOX],
		% in: link x pose relative to direct parent
		[ChildFrame, ParentFrame, PosXP, RotXP],
		% out: object pose relative to direct parent of x
		[obj_frame, ParentFrame, PosOP, RotOP]
	),
	urdf_pose_rel1(URDFObject, Prefix, RootName,
		ParentName, [ParentFrame, PosOP, RotOP],
		RelPose).

urdf_pose_rel1(_, _, _, _, RelPose, RelPose).

%% urdf_set_pose(+Object,+Pose) is semidet.
%
% Assign the initial pose of links described
% in URDF as the current pose of link entities.
% This requires that the *load_rdf* option was used in urdf_load/3.
% The base link of the URDF is assigned to the transform given
% in the second argument.
%
% @param Object IRI atom
% @param Pose A pose list of frame-position-quaternion
%
urdf_set_pose(Object,Pose) :-
	urdf_prefix(Object,Prefix),!,
	% set root link pose
	urdf_root_link(Object,RootLinkName),
	urdf_iri(Object,RootLinkName,RootLink),
	% update tf memory
	rdf_split_url(_,RootFrame,RootLink),
	tf_mem_set_pose(RootFrame,Pose,0),
	% set pose of other links
	urdf_link_names(Object,Links),
	forall(
		member(LinkName,Links), 
		(	LinkName=RootLinkName
		->	true
		;	set_link_pose_(Object,Prefix,LinkName)
		)
	).

set_link_pose_(Object, Prefix, LinkName) :-
	urdf_pose1(Object, Prefix, LinkName, _, Pose),
	% update tf memory
	atom_concat(Prefix, LinkName, LinkFrame),
	tf_mem_set_pose(LinkFrame, Pose, 0).

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
urdf_chain(Object,FromLink,ToLink,X) :-
	(	urdf_chain1(Object,FromLink,ToLink,Chain)
	->	member(X,Chain)
	;	(
		log_warning(urdf(not_connnected(FromLink,ToLink))),
		X=FromLink
	)).

urdf_chain1(_,X,X,[X]) :- !.
urdf_chain1(Object,FromLink,ToLink,[ToLink|Rest]) :-
	urdf_link_parent_joint(Object,ToLink,Joint),
	urdf_joint_parent_link(Object,Joint,ParentLink),
	urdf_chain1(Object,FromLink,ParentLink,Rest).

/**************************************/
/********* RDF REPRESENTATION *********/
/**************************************/

%%
urdf_iri(Object,Name,IRI) :-
	rdf_split_url(IRIPrefix,_,Object),
	atomic_list_concat([IRIPrefix,Name],'',IRI).

%%
:- rdf_meta(joint_type_iri(?,r)).
joint_type_iri(revolute,   urdf:'RevoluteJoint').
joint_type_iri(continuous, urdf:'ContinuousJoint').
joint_type_iri(prismatic,  urdf:'PrismaticJoint').
joint_type_iri(fixed,      urdf:'FixedJoint').
joint_type_iri(floating,   urdf:'FloatingJoint').
joint_type_iri(planar,     urdf:'PlanarJoint').

%%
load_rdf_(_, Links, has_type(Link,'http://knowrob.org/kb/urdf.owl#Link')) :-
	kb_call((member(Link,Links),\+ has_type(Link,urdf:'Link'))).

load_rdf_(Object, _, Fact) :-
	% create joint entities
	urdf_joint_names(Object,Joints),
	member(JointName,Joints),
	create_joint_(Object,JointName,_,Fact).

%%
create_joint_(Object,Name,Joint,Fact) :-
	urdf_iri(Object,Name,Joint),
	%%
	urdf_joint_type(Object,Name,Type),
	joint_type_iri(Type,JointType),
	%%
	urdf_joint_child_link(Object,Name,ChildName),
	urdf_joint_parent_link(Object,Name,ParentName),
	urdf_iri(Object,ChildName,Child),
	urdf_iri(Object,ParentName,Parent),
	%%
	(	Fact=has_type(Joint,JointType)
	;	Fact=has_child_link(Joint,Child)
	;	Fact=has_parent_link(Joint,Parent)
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
has_urdf_name(Obj,Name) ?+>
	triple(Obj,urdf:hasURDFName,Name).

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
object_shape_urdf(Link,ShapeID,ShapeTerm,Origin,MaterialTerm) :-
	var(Link),!,
	is_urdf_link(Link),
	has_urdf(Link, Root),
	link_shape_(Root, Link, ShapeID, ShapeTerm, Origin, MaterialTerm).

object_shape_urdf(Obj,ShapeID,ShapeTerm,Origin,MaterialTerm) :-
	%nonvar(Obj),
	has_urdf(Obj, Root),
	kb_call((
		triple(Obj, transitive(reflexive(dul:hasComponent)), Link),
		is_urdf_link(Link)
	)),
	link_shape_(Root, Link, ShapeID, ShapeTerm, Origin, MaterialTerm).

%%
link_shape_(Root, Link, ShapeID, ShapeTerm, [Frame,Pos,Rot], MaterialTerm) :-
	rdf_split_url(_, LinkName, Link),
	urdf_catch(urdf_link_visual_shape(Root, LinkName,
		ShapeTerm,[Name,Pos,Rot],MaterialTerm,ShapeID)),
	urdf_prefix(Root, Prefix),
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
