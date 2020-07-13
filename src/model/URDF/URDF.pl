:- module(model_URDF,
	[ urdf_load(r,+),
	  has_urdf_name(r,?),
	  has_joint(r,r),
	  has_link(r,r),
	  has_root_link(r,r),
	  has_child_link(r,r),
	  has_parent_link(r,r),
	  has_joint_hard_limits(r,?,?,?),
	  has_joint_soft_limits(r,?,?,?),
	  has_joint_calibration(r,?,?),
	  has_joint_axis(r,?),
	  has_joint_friction(r,?),
	  has_joint_damping(r,?),
	  has_link_mass(r,?),
	  has_link_inertia(r,?,r),
	  has_link_visual(r,?,?),
	  has_link_collision(r,?,?)
    ]).

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3, rdf_meta/1 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).
:- use_module(library('lang/query')).
:- use_module(library('lang/terms/holds')).
:- use_module('./parser.pl').

:- tripledb_load('package://knowrob/owl/urdf.owl',
    [ graph(tbox),
      namespace(urdf,'http://knowrob.org/kb/urdf.owl#')
    ]).

%% urdf_name(?Entity,?Name) is semidet.
%
has_urdf_name(Entity,Name) ?+>
	triple(Entity,urdf:hasURDFName,Name).

%% has_joint(?Robot,?Joint) is semidet.
%
has_joint(Robot,Joint) ?+>
	triple(Robot,urdf:hasJoint,Joint).

%% has_link(?Robot,?Link) is semidet.
%
has_link(Robot,Link) ?+>
	triple(Robot,urdf:hasLink,Link).

%% has_root_link(?Robot,?Link) is semidet.
%
has_root_link(Robot,Link) ?+>
	triple(Robot,urdf:hasRootLink,Link).

%% has_child_link(?Joint,?Link) is semidet.
%
has_child_link(Joint,Link) ?+>
	triple(Joint,urdf:hasChildLink,Link).

%% has_parent_link(?Joint,?Link) is semidet.
%
has_parent_link(Joint,Link) ?+>
	triple(Joint,urdf:hasParentLink,Link).

%% has_joint_hard_limits(?Joint,?Pos,?Vel,?Eff) is semidet.
%
has_joint_hard_limits(J, [LL,UL], VelMax, EffMax) ?>
	triple(J,urdf:hasJointLimits,Lim),
	has_hard_limit_data(Lim, [LL,UL], VelMax, EffMax).

has_joint_hard_limits(J, [LL,UL], VelMax, EffMax) +>
	{ tabled_hard_limits(Lim,[LL,UL],VelMax,EffMax) },
	triple(J,urdf:hasJointLimits,Lim).

%%
:- table tabled_hard_limits/4.
tabled_hard_limits(Saf,L,VelMax,EffMax) :-
	tell([
		has_type(Saf,urdf:'JointLimits'),
		has_hard_limit_data(Saf,L,VelMax,EffMax)
	]).

%%
has_hard_limit_data(Lim, [LL,UL], VelMax, EffMax) ?+>
	triple(Lim,urdf:hasLowerLimit,LL),
	triple(Lim,urdf:hasUpperLimit,UL),
	triple(Lim,urdf:hasMaxJointVelocity,VelMax),
	triple(Lim,urdf:hasMaxJointEffort,EffMax).

%% has_joint_soft_limits(?Joint,?Pos,?KP,?KV) is semidet.
%
has_joint_soft_limits(J, [LL,UL], KP, KV) ?>
	triple(J,urdf:hasJointSoftLimits,Saf),
	has_soft_limit_data(Saf, [LL,UL], KP, KV).

has_joint_soft_limits(J, [LL,UL], KP, KV) +>
	{ tabled_soft_limits(Saf,[LL,UL],KP,KV) },
	triple(J,urdf:hasJointSoftLimits,Saf).

%%
:- table tabled_soft_limits/4.
tabled_soft_limits(Saf,L,KP,KV) :-
	tell([
		has_type(Saf,urdf:'JointSoftLimits'),
		has_soft_limit_data(Saf,L,KP,KV)
	]).

%%
has_soft_limit_data(Saf, [LL,UL], KP, KV) ?+>
	triple(Saf,urdf:hasLowerLimit,LL),
	triple(Saf,urdf:hasUpperLimit,UL),
	triple(Saf,urdf:hasKPosition,KP),
	triple(Saf,urdf:hasKVelocity,KV).
  
%% has_joint_calibration(?Joint,?FallingEdge,?RisingEdge) is semidet.
%
has_joint_calibration(J,Falling,Rising) ?>
	triple(J,urdf:hasJointReferencePositions,Calib),
	triple(Calib,urdf:hasRisingEdge,Rising),
	triple(Calib,urdf:hasFallingEdge,Falling).

has_joint_calibration(J,Falling,Rising) +>
	{ tabled_joint_calibration(Calib,Falling,Rising) },
	triple(J,urdf:hasJointReferencePositions,Calib).

%%
:- table tabled_joint_calibration/3.
tabled_joint_calibration(Calib,Falling,Rising) :-
	tell([
		has_type(Calib,urdf:'JointReferencePositions'),
		triple(Calib,urdf:hasRisingEdge,Rising),
		triple(Calib,urdf:hasFallingEdge,Falling)
	]).

%% has_joint_axis(?Joint,?Axis) is semidet.
%
has_joint_axis(J,AxisData) ?>
	triple(J,urdf:hasJointAxis,Axis),
	triple(Axis,urdf:hasAxisVector,term(AxisData)).

has_joint_axis(J,AxisData) +>
	{ tabled_joint_axis(Axis,AxisData) },
	triple(J,urdf:hasJointAxis,Axis).

%%
:- table tabled_joint_axis/2.
tabled_joint_axis(Axis,AxisData) :-
	tell([
		has_type(Axis,urdf:'JointAxis'),
		triple(Axis,urdf:hasAxisVector,term(AxisData))
	]).

%% has_joint_friction(?Joint,?Friction) is semidet.
%
has_joint_friction(J, Friction) ?>
	triple(J,ease_obj:hasFrictionAttribute,X),
	triple(X,ease_obj:hasFrictionValue,Friction).

has_joint_friction(J, Friction) +>
	{ tabled_joint_friction(Attribute,Friction) },
	triple(J,ease_obj:hasFrictionAttribute,Attribute).

%%
:- table tabled_joint_friction/2.
tabled_joint_friction(Attribute,Friction) :-
	tell([
		has_type(Attribute,ease_obj:'StaticFrictionAttribute'),
		triple(Attribute,ease_obj:hasFrictionValue,Friction)
	]).
  
%% has_joint_damping(?Joint,?Damping) is semidet.
%
has_joint_damping(J, Damping) ?>
	triple(J,urdf:hasDampingAttribute,X),
	triple(X,urdf:hasDampingValue,Damping).

has_joint_damping(J, DampingValue) +>
	{ tabled_joint_damping(Attribute,DampingValue) },
	triple(J,urdf:hasDampingAttribute,Attribute).

%%
:- table tabled_joint_damping/2.
tabled_joint_damping(Attribute,Damping) :-
	tell([
		has_type(Attribute,urdf:'DampingAttribute'),
		triple(Attribute,urdf:hasDampingValue,Damping)
	]).

%% has_link_mass(+Link, ?MassValue) is semidet.
%
has_link_mass(Link,MassValue) ?>
	triple(Link,ease_obj:hasMassAttribute,Mass),
	triple(Mass,ease_obj:hasMassValue,MassValue).

has_link_mass(Link,MassValue) +>
	{ tabled_link_mass(Attribute,MassValue) },
	triple(Link,ease_obj:hasMassAttribute,Attribute).

%%
:- table tabled_link_mass/2.
tabled_link_mass(Attribute,MassValue) :-
	tell([
		has_type(Attribute,ease_obj:'MassAttribute'),
		triple(Attribute,ease_obj:hasMassValue,MassValue)
	]).

%% has_link_inertia(+Link, ?Matrix) is semidet.
%
has_link_inertia(Link,Data,Inertia) ?>
	triple(Link,urdf:hasInertia,Inertia),
	has_inertia_data(Inertia,Data).

has_link_inertia(Link,Data,Inertia) +>
	{ tabled_inertia(Inertia,Data) },
	triple(Link,urdf:hasInertia,Inertia).

%%
:- table tabled_inertia/2.
tabled_inertia(Inertia,InertiaData) :-
	tell([
		has_type(Inertia,urdf:'Inertia'),
		has_inertia_data(Inertia,InertiaData)
	]).

%%
has_inertia_data(Inertia,[XX,XY,XZ,YY,YZ,ZZ]) ?+>
	triple(Inertia,urdf:hasInertia_ixx,XX),
	triple(Inertia,urdf:hasInertia_ixy,XY),
	triple(Inertia,urdf:hasInertia_ixz,XZ),
	triple(Inertia,urdf:hasInertia_iyy,YY),
	triple(Inertia,urdf:hasInertia_iyz,YZ),
	triple(Inertia,urdf:hasInertia_izz,ZZ).

%% has_link_visual(+Link, ?ShapeTerm, ?Origin) is semidet.
%
has_link_visual(Link,ShapeTerm,Origin) ?>
	has_visual_shape(Link,ShapeTerm,Shape),
	has_urdf_name(Link,Name),
	has_urdf_origin(Shape,Name,Origin).

%% has_link_collision(+Link, ?ShapeTerm, ?Origin) is semidet.
%
has_link_collision(Link,ShapeTerm,Origin) ?>
	has_collision_shape(Link,ShapeTerm,Shape),
	has_urdf_name(Link,Name),
	has_urdf_origin(Shape,Name,Origin).

		 /*******************************
		 *		  Shapes		*
		 *******************************/

%%
has_visual_shape(Link,Geom,Shape) ?>
	triple(Link,ease_obj:hasShape,Shape),
	has_shape_data(Shape,Geom).

has_visual_shape(Link,Geom,Shape) +>
	{ tabled_shape(Shape,Geom) },
	triple(Link,ease_obj:hasShape,Shape).

%%
has_collision_shape(Link,Geom,Shape) ?>
	triple(Link,urdf:hasCollisionShape,Shape),
	has_shape_data(Shape,Geom).

has_collision_shape(Link,Geom,Shape) +>
	{ tabled_shape(Shape,Geom) },
	triple(Link,urdf:hasCollisionShape,Shape).

%%
:- table tabled_shape/2.
tabled_shape(Shape,Geom) :- tell(has_shape_data(Shape,Geom)).
  
%%
has_shape_data(Shape, box(X,Y,Z)) ?+>
	has_type(Shape, ease_obj:'BoxShape'),
	triple(Shape, ease_obj:hasWidth,  X),
	triple(Shape, ease_obj:hasHeight, Y),
	triple(Shape, ease_obj:hasDepth,  Z).

has_shape_data(Shape, cylinder(Radius, Length)) ?+>
	has_type(Shape, ease_obj:'CircularCylinder'),
	triple(Shape, ease_obj:hasRadius, Radius),
	triple(Shape, ease_obj:hasLength, Length).

has_shape_data(Shape, sphere(Radius)) ?+>
	has_type(Shape, ease_obj:'SphereShape'),
	triple(Shape, ease_obj:hasRadius, Radius).

has_shape_data(Shape, mesh(Filename,[X,Y,Z])) ?+>
	has_type(Shape, ease_obj:'MeshShape'),
	triple(Shape, ease_obj:hasFilePath, Filename),
	triple(Shape, knowrob:hasXScale, X),
	triple(Shape, knowrob:hasYScale, Y),
	triple(Shape, knowrob:hasZScale, Z).

		 /*******************************
		 *		  Poses		*
		 *******************************/

%%
%
has_urdf_origin(Entity,Frame,OriginData) ?>
	triple(Entity,urdf:hasOrigin,Origin),
	has_origin_data(Origin,Frame,OriginData),
	{ ! }.

has_urdf_origin(_,_,pose([0.0,0.0,0.0], [1.0,0.0,0.0,0.0])) ?>
	{ ! }.

has_urdf_origin(Entity,Frame,OriginData) +>
	{ tabled_origin(Origin,Frame,OriginData) },
	triple(Entity,urdf:hasOrigin,Origin).

%%
:- table tabled_origin/3.
tabled_origin(Origin,Frame,OriginData) :-
	tell([
		has_type(Origin, dul:'Region'),
		has_origin_data(Origin,Frame,OriginData)
	]).

%%
has_origin_data(Origin,Frame,pose(Position,Quaternion)) ?+>
	triple(Origin, ease_obj:hasPositionVector,    term(Position)),
	triple(Origin, ease_obj:hasOrientationVector, term(Quaternion)),
	triple(Origin, ease_obj:hasReferenceFrame, Frame).

		 /************************************
		  *            Loading URDF Files    *
		  ************************************/

%% urdf_load(+Robot, +URDF_File) is det.
%
% Load a URDF file and map it into RDF triple store
% using the model defined in 'urdf.owl'.
%
urdf_load(Robot, URDF_File) :-
	rdf_split_url(_, Robot_Id, Robot),
	setup_call_cleanup(
		load_urdf_file(Robot_Id, URDF_File),
		urdf_load1(Robot, Robot_Id),
		unload_urdf_file(Robot_Id)
	).
  
urdf_load1(Robot, Robot_Id) :-
	% read from URDF file
	robot_name(Robot_Id,RobotName),
	link_names(Robot_Id,Links),
	joint_names(Robot_Id,Joints),
	root_link_name(Robot_Id,Root),
	%%
	tell(has_urdf_name(Robot,RobotName)),
get_time(T0),
	forall(
		member(LinkName,Links), 
		load_link(Robot,Robot_Id,Root,LinkName)
	),
get_time(T1),
	forall(
		member(JointName,Joints), 
		load_joint(Robot,Robot_Id,JointName)
	),
get_time(T2),
XX is T1-T0,
YY is T2-T1,
write('time links: '), writeln(XX),
write('time joints: '), writeln(YY)
.
  
		 /*******************************
		 *		  Loading Links		*
		 *******************************/

%%
load_link(Robot,Robot_Id,Root,Name) :-
	%%
	atomic_list_concat([Robot,'_',Name],'',Link),
	tell([
		has_type(Link,urdf:'Link'),
		has_urdf_name(Link,Name)
	]),
	(	Root=Name
	->	tell(has_root_link(Robot,Link))
	;	tell(has_link(Robot,Link))
	),
	(	link_inertial_mass(Robot_Id,Name,MassValue)
	->	tell(has_link_mass(Link,MassValue))
	;	true
	),
	(	\+ link_inertial_inertia(Robot_Id,Name,_)
	;	load_link_inertia(Robot_Id,Name,Link)
	),
	!,
	load_link_visual(Robot_Id,Name,Link),
	load_link_collisions(Robot_Id,Name,Link).

%%
load_link_inertia(Robot_Id,Name,Link) :-
	link_inertial_inertia(Robot_Id,Name,InertiaData),
	%% FIXME: origin should also be tabled
	tell(has_link_inertia(Link,InertiaData,Inertia)),
	(	link_inertial_origin(Robot_Id,Name,Pose_data) 
	->	tell(has_urdf_origin(Inertia,Name,Pose_data))
	;	true
	),
	!.

%%
load_link_visual(Robot_Id,Name,Link) :-
	link_num_visuals(Robot_Id,Name,Count),
	N is Count - 1,
	forall(
		between(0,N,Index),
		load_link_visual1(Robot_Id,Name,Index,Link)
	).

load_link_visual1(Robot_Id,Name,Index,Link) :-
	%%
	link_visual_geometry(Robot_Id,Name,Index,Geom),
	%% FIXME: origin should also be tabled
	tell(has_visual_shape(Link,Geom,Shape)),
	%%
	(	link_visual_origin(Robot_Id,Name,Index,Pose_data)
	->	tell(has_urdf_origin(Shape,Name,Pose_data))
	;	true
	).

%%
load_link_collisions(Robot_Id,Name,Link) :-
	link_num_collisions(Robot_Id,Name,Count),
	N is Count - 1,
	forall(
		between(0,N,Index),
		load_link_collisions1(Robot_Id,Name,Index,Link)
	).

load_link_collisions1(Robot_Id,Name,Index,Link) :-
	%%
	link_collision_geometry(Robot_Id,Name,Index,Geom),
	%% FIXME: origin should also be tabled
	tell(has_collision_shape(Link,Geom,Shape)),
	%%
	(	link_collision_origin(Robot_Id,Name,Index,Pose_data)
	->	tell(has_urdf_origin(Shape,Name,Pose_data))
	;	true
	).

		 /*******************************
		 *		  Loading Joints	*
		 *******************************/

:- rdf_meta(rdf_joint_type(?,r)).

%%
rdf_joint_type(revolute,   urdf:'RevoluteJoint').
rdf_joint_type(continuous, urdf:'ContinuousJoint').
rdf_joint_type(prismatic,  urdf:'PrismaticJoint').
rdf_joint_type(fixed,      urdf:'FixedJoint').
rdf_joint_type(floating,   urdf:'FloatingJoint').
rdf_joint_type(planar,     urdf:'PlanarJoint').

%%
load_joint(Robot,Robot_Id,Name) :-
	joint_type(Robot_Id,Name,Type),
	joint_child_link(Robot_Id,Name,ChildName),
	joint_parent_link(Robot_Id,Name,ParentName),
	atomic_list_concat([Robot,'_',Name],'',Joint),
	atomic_list_concat([Robot,'_',ChildName],'',Child),
	atomic_list_concat([Robot,'_',ParentName],'',Parent),
	%%
	once(rdf_joint_type(Type,JointType)),
	tell([
		has_type(Joint,JointType),
		has_urdf_name(Joint,Name),
		has_joint(Robot,Joint),
		has_child_link(Joint,Child),
		has_parent_link(Joint,Parent)
	]),
	%%
	(	joint_origin(Robot_Id,Name,Pose_data)
	->	tell(has_urdf_origin(Joint,Name,Pose_data))
	;	true
	),
	%% joint dynamics (optional)
	(	\+ joint_has_dynamics(Robot_Id,Name)
	;	load_joint_dynamics(Robot_Id,Name,Joint)
	),!,
	%% joint kinematics
	(	Type = fixed
	;	load_joint_kinematics(Robot_Id,Name,Type,Joint)
	),!.

%%
load_joint_dynamics(Robot_Id,Name,Joint) :-
	(	joint_dynamics_damping(Robot_Id,Name,DampingValue)
	->	tell(has_joint_damping(Joint,DampingValue))
	;	true
	),
	(	joint_dynamics_friction(Robot_Id,Name,FrictionValue)
	->	tell(has_joint_friction(Joint,FrictionValue))
	;	true
	).

%%
load_joint_kinematics(Robot_Id,Name,Type,Joint) :-
	load_joint_axis(Robot_Id,Name,Joint),
	%% joint limits (only revolute or prismatic)
	(	\+ member(Type,[revolute,prismatic])
	;	load_joint_limits(Robot_Id,Name,Joint)
	),
	%% joint calibration (optional)
	(	\+ joint_has_calibration(Robot_Id,Name)
	;	load_joint_calibration(Robot_Id,Name,Joint)
	),
	%% joint safety (optional)
	(	\+ joint_has_safety(Robot_Id,Name)
	;	load_joint_safety(Robot_Id,Name,Joint)
	).

%%
load_joint_axis(Robot_Id,Name,Joint) :-
	(	joint_axis(Robot_Id,Name,Axis)
	;	Axis = [1,0,0]
	),
	!,
	tell(has_joint_axis(Joint,Axis)).

%%
load_joint_calibration(Robot_Id,Name,Joint) :-
	(	joint_calibration_falling(Robot_Id,Name,Falling)
	;	Falling is 0.0
	),
	(	joint_calibration_rising(Robot_Id,Name,Rising)
	;	Rising is 0.0
	),
	!,
	tell(has_joint_calibration(Joint,Falling,Rising)).

%%
load_joint_limits(Robot_Id,Name,Joint) :-
	joint_velocity_limit(Robot_Id,Name,VelMax),
	joint_effort_limit(Robot_Id,Name,EffMax),
	(	joint_lower_pos_limit(Robot_Id,Name, LL)
	;	LL is 0
	),
	(	joint_upper_pos_limit(Robot_Id,Name, UL)
	;	UL is 0
	),
	!,
	tell(has_joint_hard_limits(Joint, [LL,UL], VelMax, EffMax)).

%%
load_joint_safety(Robot_Id,Name,Joint) :-
	joint_safety_kv(Robot_Id,Name, KV),
	(	joint_safety_kp(Robot_Id,Name, KP)
	;	KP is 0.0
	),
	(	joint_safety_lower_limit(Robot_Id,Name, LL)
	;	LL is 0.0
	),
	(	joint_safety_upper_limit(Robot_Id,Name, UL)
	;	UL is 0.0
	),
	!,
	tell(has_joint_soft_limits(Joint, [LL,UL], KP, KV)).

%%
joint_has_calibration(Robot_Id,Name) :-
	(	joint_calibration_rising(Robot_Id,Name,_)
	;	joint_calibration_falling(Robot_Id,Name,_)
	),
	!.

%%
joint_has_dynamics(Robot_Id,Name) :-
	(	joint_dynamics_damping(Robot_Id,Name,_)
	;	joint_dynamics_friction(Robot_Id,Name,_)
	),
	!.

%%
joint_has_safety(Robot_Id,Name) :-
	joint_safety_kv(Robot_Id,Name,_).
