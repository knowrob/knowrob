:- module(srdl,
	[ has_base_link_name(r,?),
	  has_end_link_name(r,?),
	  has_base_link(r,r),
	  has_end_link(r,r),
	  has_child_link(r,r),
	  has_parent_link(r,r),
	  object_set_urdf(r,+),
	  object_set_urdf(r,+,+),
	  object_set_urdf_pose(r),
	  object_set_urdf_pose(r,+)
	]).

:- use_module(library('model/URDF/URDF')).
:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

:- tripledb_load('http://knowrob.org/kb/srdl2-comp.owl',
    [ namespace(srdlcomp,'http://knowrob.org/kb/srdl2-comp.owl#')
    ]).

/**************************************/
/******** Language Predicates *********/
/**************************************/

%% has_base_link_name(?Obj,?Name) is semidet.
%
has_base_link_name(Obj,Name) ?+>
	triple(Obj,srdlcomp:hasBaseLinkName,Name).

%% has_end_link_name(?Obj,?Name) is semidet.
%
has_end_link_name(Obj,Name) ?+>
	triple(Obj,srdlcomp:hasEndLinkName,Name).

%% has_base_link(?Obj,?Link) is semidet.
%
has_base_link(Obj,Link) ?+>
	triple(Obj,srdlcomp:hasBaseLink,Link).

%% has_urdf_prefix(?Obj,?Prefix) is semidet.
%
has_urdf_prefix(Obj,Prefix) ?+>
	triple(Obj,urdf:hasNamePrefix,Prefix).

%% has_end_link(?Obj,?Link) is semidet.
%
has_end_link(Obj,Link) ?+>
	triple(Obj,srdlcomp:hasEndLink,Link).

%% has_child_link(?Joint,?Link) is semidet.
%
has_child_link(Joint,Link) ?+>
	triple(Joint,urdf:hasChildLink,Link).

%% has_parent_link(?Joint,?Link) is semidet.
%
has_parent_link(Joint,Link) ?+>
	triple(Joint,urdf:hasParentLink,Link).

%%
%
object_shape(Obj,ShapeTerm,Origin) ?>
	has_base_link_name(Obj,BaseName),
	{ get_object_shape_(Obj,BaseName,ShapeTerm,Origin) }.

%%
get_object_shape_(Obj,BaseName,ShapeTerm,[Frame,Pos,Rot]) :-
	get_root_object_(Obj,Root),
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

%%
get_root_object_(Root,Root) :-
	urdf_is_loaded(Root),
	!.
get_root_object_(Part,Root) :-
	has_part(X,Part),
	get_root_object_(X,Root).

/**************************************/
/*********** URDF Data ****************/
/**************************************/

%%
:- rdf_meta(joint_type_(?,r)).
joint_type_(revolute,   urdf:'RevoluteJoint').
joint_type_(continuous, urdf:'ContinuousJoint').
joint_type_(prismatic,  urdf:'PrismaticJoint').
joint_type_(fixed,      urdf:'FixedJoint').
joint_type_(floating,   urdf:'FloatingJoint').
joint_type_(planar,     urdf:'PlanarJoint').

%%
%
object_set_urdf(Object,File) :-
	object_set_urdf(Object,File,[]).

object_set_urdf(Object,File,Options) :-
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
	% FIXME: lots of redundant results with transitive, and its super slow
	% setof(X, transitive(has_part(Object,X)), Parts),
	setof(X, has_part_(Object,X), Parts),
	% set component poses relative to links
	forall(member(Y,Parts),
		part_set_urdf_(Y,OptPrefix)),
	% optional: load links and joints as rdf objects
	(	option(load_rdf,Options)
	->	load_urdf_triples(Object,Parts,OptPrefix)
	;	true
	).

%%
has_part_(X,X).
has_part_(X,Y) :-
	triple(X,dul:hasPart,Z),
	has_part_(Z,Y).

%%
part_set_urdf_(Part,Prefix) :-
	% components are exactly located at the base link
	(	get_component_frame_(Part,Prefix,Frame)
	->	tell(is_at(Part,[Frame,[0,0,0],[0,0,0,1]]))
	;	log_warn(srdl(no_comp_pose(Part)))
	).

get_component_frame_(Part,Prefix,Frame) :-
	has_base_link_name(Part,LinkName),
	atom_concat(Prefix,LinkName,Frame).

%%
load_urdf_triples(Object,Parts,Prefix) :-
	urdf_link_names(Object,Links),
	urdf_joint_names(Object,Joints),
	% create link entities
	forall(member(LinkName,Links), 
		create_link_(Object,Prefix,LinkName,_)),
	% create joint entities
	forall(member(JointName,Joints), 
		create_joint_(Object,Prefix,JointName,_)),
	% associate links to components
	forall(member(Part,Parts),
		set_links_(Part,Prefix)).

%%
create_link_(Object,Prefix,Name,Link) :-
	urdf_iri_(Object,Prefix,Name,Link),
	tell(has_type(Link,urdf:'Link')).

%%
create_joint_(Object,Prefix,Name,Joint) :-
	urdf_iri_(Object,Prefix,Name,Joint),
	%%
	urdf_joint_type(Object,Name,Type),
	joint_type_(Type,JointType),
	%%
	urdf_joint_child_link(Object,Name,ChildName),
	urdf_joint_parent_link(Object,Name,ParentName),
	urdf_iri_(Object,Prefix,ChildName,Child),
	urdf_iri_(Object,Prefix,ParentName,Parent),
	%%
	tell([
		has_type(Joint,JointType),
		has_child_link(Joint,Child),
		has_parent_link(Joint,Parent)
	]).

%%
set_links_(Part,URDFPrefix) :-
	(	get_base_link_(Part,URDFPrefix,BaseLink)
	->	tell(has_base_link(Part,BaseLink))
	;	true
	),
	forall(
		get_end_link_(Part,URDFPrefix,EndLink),
		tell(has_end_link(Part,EndLink))
	).

get_base_link_(Part,Prefix,BaseLink) :-
	has_base_link_name(Part,LinkName),
	urdf_iri_(Part,Prefix,LinkName,BaseLink).

get_end_link_(Part,Prefix,BaseLink) :-
	has_end_link_name(Part,LinkName),
	urdf_iri_(Part,Prefix,LinkName,BaseLink).

%%
%
object_set_urdf_pose(Object) :-
	object_set_urdf_pose(Object,[map,[0,0,0],[0,0,0,1]]).

object_set_urdf_pose(Object,Pose) :-
	(	has_urdf_prefix(Object,Prefix)
	;	Prefix=''
	),!,
	% set root link pose
	urdf_root_link(Object,RootLinkName),
	urdf_iri_(Object,Prefix,RootLinkName,RootLink),
	tell(is_at(RootLink,Pose)),
	% set pose of other links
	urdf_link_names(Object,Links),
	forall(
		member(LinkName,Links), 
		(	LinkName=RootLinkName
		->	true
		;	set_link_pose_(Object,Prefix,LinkName)
		)
	).

set_link_pose_(Object,Prefix,LinkName) :-
	urdf_link_parent_joint(Object,LinkName,JointName),
	urdf_joint_origin(Object,JointName,[_,Pos,Rot]),
	urdf_joint_parent_link(Object,JointName,ParentName),
	urdf_iri_(Object,Prefix,LinkName,Link),
	atom_concat(Prefix,ParentName,ParentFrame),
	tell(is_at(Link,[ParentFrame,Pos,Rot])).

%%
urdf_iri_(Object,URDFPrefix,Name,IRI) :-
	rdf_split_url(IRIPrefix,_,Object),
	atomic_list_concat([IRIPrefix,URDFPrefix,Name],'',IRI).
