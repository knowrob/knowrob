:- module(rdf_urdf,
    [
    rdf_urdf_name/2,
    rdf_urdf_joint/2,
    rdf_urdf_joint_origin/2,
    rdf_urdf_link/2,
    rdf_urdf_has_child/2,
    rdf_urdf_has_parent/2,
    rdf_urdf_kinematics/2,
    rdf_urdf_calibration/3,
    rdf_urdf_friction/2,
    rdf_urdf_damping/2,
    rdf_urdf_axis/2,
    rdf_urdf_visual/2,
    rdf_urdf_visual/3,
    rdf_urdf_collision/2,
    rdf_urdf_collision/3,
    rdf_urdf_shape/2,
    rdf_urdf_inertial/3,
    rdf_urdf_inertial_origin/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/transforms')).

:- use_module('urdf_parser').

:-  rdf_meta
    rdf_urdf_name(r,?),
    rdf_urdf_joint(r,t),
    rdf_urdf_joint_origin(r,?),
    rdf_urdf_link(r,t),
    rdf_urdf_has_child(r,r),
    rdf_urdf_has_parent(r,r),
    rdf_urdf_kinematics(r,r),
    rdf_urdf_calibration(r,?,?),
    rdf_urdf_friction(r,?),
    rdf_urdf_damping(r,?),
    rdf_urdf_axis(r,?),
    rdf_urdf_visual(r,?),
    rdf_urdf_visual(r,?,?),
    rdf_urdf_collision(r,?),
    rdf_urdf_collision(r,?,?),
    rdf_urdf_shape(r,?),
    rdf_urdf_inertial(r,?,?),
    rdf_urdf_inertial_origin(r,?).

rdf_urdf_name(Entity,Name) :-
  rdf_has(Entity,urdf:hasURDFName,literal(type(_,Name))).

rdf_urdf_joint(Robot,Joint) :-
  rdf_has(Robot,urdf:hasJoint,Joint).

rdf_urdf_link(Robot,Link) :-
  rdf_has(Robot,urdf:hasLink,Link).

rdf_urdf_has_child(Joint,Link) :-
  rdf_has(Joint,urdf:hasChildLink,Link).

rdf_urdf_has_parent(Joint,Link) :-
  rdf_has(Joint,urdf:hasParentLink,Link).

rdf_urdf_kinematics(Entity,Kin) :-
  rdf_has(Entity,urdf:hasKinematics,Kin).

rdf_urdf_friction(Entity, Friction) :-
  rdf_has(Entity,urdf:hasDynamics,Dyn),
  rdf_has(Dyn,urdf:hasFriction,X),
  rdf_has_prolog(X,dul:hasDataValue,Friction).
  
rdf_urdf_damping(Entity, Damping) :-
  rdf_has(Entity,urdf:hasDynamics,Dyn),
  rdf_has(Dyn,urdf:hasDamping,X),
  rdf_has_prolog(X,dul:hasDataValue,Damping).
  
rdf_urdf_calibration(Entity,Falling,Rising) :-
  rdf_urdf_kinematics(Entity,Kin),
  rdf_has(Kin,urdf:hasJointCalibration,Calib),
  rdf_has_prolog(Calib,urdf:hasRisingEdge,Rising),
  rdf_has_prolog(Calib,urdf:hasFallingEdge,Falling).

rdf_urdf_axis(Entity,[X,Y,Z]) :-
  rdf_urdf_kinematics(Entity,Kin),
  rdf_has(Kin,urdf:hasJointAxis,Axis),
  rdf_has_prolog(Axis,ease:hasXComponent,X),
  rdf_has_prolog(Axis,ease:hasYComponent,Y),
  rdf_has_prolog(Axis,ease:hasZComponent,Z).

rdf_urdf_joint_origin(Joint,[_,_,Pos,Rot]) :- % TODO: what is the reference frame?
  rdf_has(Joint,urdf:hasJointOrigin,Origin),!,
  transform_data(Origin,(Pos,Rot)).

rdf_urdf_origin(Entity,RefFrame,[_,RefFrame,Pos,Rot]) :-
  rdf_has(Entity,urdf:hasOrigin,Origin),!,
  transform_data(Origin,(Pos,Rot)).

rdf_urdf_origin(_Entity,RefFrame,[_,RefFrame,
  [0.0,0.0,0.0],[1.0,0.0,0.0,0.0]]).

rdf_urdf_inertial(Entity,MassValue,[XX,XY,XZ,YY,YZ,ZZ]) :-
  rdf_has(Entity,urdf:hasDynamics,Dyn),
  %
  rdf_has(Dyn,urdf:hasMass,Mass),
  rdf_has_prolog(Mass,dul:hasDataValue,MassValue),
  %
  rdf_has(Dyn,urdf:hasInertiaMatrix,Mat),
  rdf_has_prolog(Mat,urdf:hasInertia_ixx,XX),
  rdf_has_prolog(Mat,urdf:hasInertia_ixy,XY),
  rdf_has_prolog(Mat,urdf:hasInertia_ixz,XZ),
  rdf_has_prolog(Mat,urdf:hasInertia_iyy,YY),
  rdf_has_prolog(Mat,urdf:hasInertia_iyz,YZ),
  rdf_has_prolog(Mat,urdf:hasInertia_izz,ZZ).

rdf_urdf_inertial_origin(Joint,[_,RefFrame,Pos,Rot]) :-
  rdf_urdf_name(Joint,RefFrame),
  rdf_has(Joint,urdf:hasDynamics,Dyn),
  rdf_has(Dyn,urdf:hasInertiaOrigin,Origin),!,
  transform_data(Origin,(Pos,Rot)).
  
%%
rdf_urdf_shape(Shape,box(X, Y, Z)) :-
  rdfs_individual_of(Shape,ease_obj:'Box'),
  rdf_has_prolog(Shape, ease_obj:hasWidth, X),
  rdf_has_prolog(Shape, ease_obj:hasHeight, Y),
  rdf_has_prolog(Shape, ease_obj:hasDepth, Z).

rdf_urdf_shape(Shape,cylinder(Radius, Length)) :-
  rdfs_individual_of(Shape,ease_obj:'Cylinder'),
  rdf_has_prolog(Shape, ease_obj:hasRadius, Radius),
  rdf_has_prolog(Shape, ease_obj:hasLength, Length).

rdf_urdf_shape(Shape,sphere(Radius)) :-
  rdfs_individual_of(Shape,ease_obj:'Sphere'),
  rdf_has_prolog(Shape, ease_obj:hasRadius, Radius).

rdf_urdf_shape(Shape,mesh(Filename,Scale)) :-
  rdfs_individual_of(Shape,ease_obj:'Mesh'),
  rdf_has_prolog(Shape, ease_obj:hasFilePath, Filename),
  rdf_urdf_scale(Shape, Scale).

rdf_urdf_scale(Shape, [X,Y,Z]) :-
  rdf_has_prolog(Shape, ease_obj:hasXScale, X),
  rdf_has_prolog(Shape, ease_obj:hasYScale, Y),
  rdf_has_prolog(Shape, ease_obj:hasZScale, Z),!.
rdf_urdf_scale(_Shape, [1.0,1.0,1.0]).

rdf_urdf_visual(Entity,ShapeTerm) :-
  rdf_urdf_visual(Entity,ShapeTerm,_).

rdf_urdf_visual(Entity,ShapeTerm,Origin) :-
  rdf_urdf_name(Entity,RefFrame),
  rdf_has(Entity,urdf:hasVisual,Visual),
  rdf_has(Visual,ease_obj:hasShape,Shape),
  rdf_urdf_shape(Shape,ShapeTerm),
  rdf_urdf_origin(Visual,RefFrame,Origin).

rdf_urdf_collision(Entity,ShapeTerm) :-
  rdf_urdf_collision(Entity,ShapeTerm,_).

rdf_urdf_collision(Entity,ShapeTerm,Origin) :-
  rdf_urdf_name(Entity,RefFrame),
  rdf_has(Entity,urdf:hasCollisionModel,Coll),
  rdf_has(Coll,ease_obj:hasShape,Shape),
  rdf_urdf_shape(Shape,ShapeTerm),
  rdf_urdf_origin(Coll,RefFrame,Origin).
