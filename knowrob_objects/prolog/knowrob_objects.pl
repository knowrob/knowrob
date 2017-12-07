/** <module> Utilities for reasoning about objects

  Copyright (C) 2011-2014 Moritz Tenorth
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

  @author Moritz Tenorth
  @license BSD
*/

:- module(knowrob_objects,
    [
      current_object_pose/2,
      object_pose_at_time/3,
      object_pose_at_time/4,
      object_color/2,
      object_mesh_path/2,
      object_dimensions/4,
      object_distance/3,
      object_assert_dimensions/4,
      object_assert_color/2,
      create_joint_information/9,
      update_joint_information/7,
      read_joint_information/9,
      delete_joint_information/1,
      comp_pose/2,
      comp_pose_at_time/3,
      storagePlaceFor/2,
      storagePlaceForBecause/3,
      object_queries/2,
      object_query/4
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_math')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('owl_parser')).

:- owl_parser:owl_parse('package://knowrob_objects/owl/knowrob_objects.owl').

:-  rdf_meta
    current_object_pose(r,-),
    current_object_pose(r,r,-),
    object_pose_at_time(r,r,?),
    object_pose_at_time(r,r,?,?),
    object_pose(+,+,-),
    object_color(r, ?),
    object_dimensions(r, ?, ?, ?),
    object_distance(r,r,-),
    object_assert_dimensions(r, +, +, +),
    object_assert_color(r, +),
    comp_pose(r, r),
    comp_pose_at_time(r, r, +),
    create_joint_information(r, r, r, +, ?, +, +, +, r),
    update_joint_information(r, r, +, ?, +, +, +),
    read_joint_information(r, r, r, r, -, -, -, -, -),
    delete_joint_information(r),
    storagePlaceFor(r,r),
    storagePlaceForBecause(r,r,r),
    object_query(r,?,?,?),
    object_queries(r,?).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

% TODO
% - don't use srdl2comp! should use more general property.

comp_pose(Obj, Pose) :-
    get_timepoint(Instant),
    comp_pose_at_time(Obj, Pose, Instant).

comp_pose_at_time(Obj, Pose, Instant) :-
  not( is_list(Instant) ), % FIXME: howto handle interval case?
  object_detection(Obj, Instant, Detection),
  rdf_triple(knowrob:eventOccursAt, Detection, Pose),
  rdf_has(Detection, knowrob:startTime, StartTime),
  time_term(StartTime, Begin),
  % query time interval of next object perception
  (  rdf_has(Next, knowrob:previousDetectionOfObject, Detection)
  -> (
      rdf_has(Next, knowrob:startTime, EndTime),
      time_term(EndTime, End),
      PoseInterval = [Begin,End]
  ) ; (
      PoseInterval = [Begin]
  )),
  (  ground(Interval)
  -> interval_during(Interval, PoseInterval)
  ;  Interval = PoseInterval
  ), !.

knowrob_temporal:holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', Pose, [Interval,Interval]) :- comp_pose_at_time(Obj, Pose, Instant).


%% current_object_pose(+ObjInstance, -PoseList) is nondet.
%
% Get the pose of an object based on the latest perception
%
% @param Obj       Instance of a subclass of SpatialThing-Localized
% @param PoseList  Row-based representation of the object by translation and quaternion list[7]
% 
current_object_pose(Obj, [TX,TY,TZ,QX,QY,QZ,QW]) :-
  current_time(T),
  object_pose_at_time(Obj, T, pose([TX, TY, TZ], [QX,QY,QZ,QW])),!.

%% current_object_pose(+ObjInstance, -PoseList) is nondet.
%
% Get the pose of an object based on the latest perception
%
% @param Obj       Instance of a subclass of SpatialThing-Localized
% @param PoseList  Row-based representation of the object's 4x4 pose matrix as list[16]
% 
current_object_pose(Obj, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-
  current_time(T),
  object_pose_at_time(Obj, T, mat([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33])),!.

%% object_pose_at_time(+ObjInstance, +Time, -Position, -Quaternion) is nondet.
%
% Get the pose of an object based on the latest perception before Time
%
% @param Obj         Instance of a subclass of SpatialThing-Localized
% @param Time        Instance of a TimePoint
% @param Position    list[3] that represents the position of the object
% @param Quaternion  list[4] that represents the rotation of the object
% 
object_pose_at_time(Obj, Time, Pose) :-
  atom(Time),
  time_term(Time, Time_val),
  object_pose_at_time(Obj, Time_val, Pose, [Time_val,Time_val]).

object_pose_at_time(Obj, Time, Pose) :-
  number(Time),
  object_pose_at_time(Obj, Time, Pose, [Time,Time]).

object_pose_at_time(Obj, Time, pose([X,Y,Z], [QX,QY,QZ,QW]), Interval) :-
  object_pose_holds(Obj, Time, Pose, Interval),
  object_pose(Pose, Time, pose([X,Y,Z], [QX,QY,QZ,QW])),!.

object_pose_at_time(Obj, Time, pose([X,Y,Z], [QX,QY,QZ,QW]), Interval) :-
  interval_during(Time, Interval), % FIXME: what is this case?
  object_pose(Obj, Time, pose([X,Y,Z], [QX,QY,QZ,QW])),!.

object_pose_at_time(Obj, Time, mat(Matrix), Interval) :-
  object_pose_holds(Obj, Time, Pose, Interval),
  object_pose(Pose, Time, mat(Matrix)),!.

object_pose_at_time(Obj, Time, mat(Matrix), Interval) :-
  interval_during(Time, Interval), % FIXME: what is this case?
  object_pose(Obj, Time, mat(Matrix)),!.

object_pose_at_time(Obj, Time, Pose, Interval) :-
  object_pose_holds(Obj, Time, Pose, Interval). % FIXME: what is this case?


object_pose_holds(Pose, _, Pose, [0.0]) :-
  nonvar(Pose), rdfs_individual_of(Pose, knowrob:'Pose'), !.

object_pose_holds(Pose, _, Pose, [0.0]) :-
  nonvar(Pose), rdfs_individual_of(Pose, knowrob:'Matrix'), !.

object_pose_holds(Obj, Time, Pose, Interval) :-
  nonvar(Obj),
  rdf_has(Obj, srdl2comp:baseLinkOfComposition, BaseLink),
  object_pose_holds(BaseLink, Time, Pose, Interval), !.

object_pose_holds(Obj, Time, Pose, Interval) :-
  holds( knowrob:pose(Obj,Pose), Interval ),
  interval_during(Time, Interval), !.


% TransformationMatrix
object_pose(Matrix, _, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  is_list(Matrix), !,
  matrix_rotation(Matrix, [QX,QY,QZ,QW]),
  matrix_translation(Matrix, [X,Y,Z]).

object_pose(Matrix, _, mat(Matrix)) :-
  is_list(Matrix), !.

% Quaternion and position
object_pose(Pose, _, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  position_to_list(Pose, [X,Y,Z]),
  quaternion_to_list(Pose, [QX,QY,QZ,QW]), !.

object_pose(Pose, _, mat(Mat)) :-
  position_to_list(Pose, [X,Y,Z]),
  quaternion_to_list(Pose, [QX,QY,QZ,QW]),
  matrix([X,Y,Z], [QX,QY,QZ,QW], Mat), !.

object_pose(Pose, _, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  rdfs_individual_of(Pose, knowrob:'Matrix'),
  rotmat_to_list(Pose, Matrix),
  matrix_rotation(Matrix, [QX,QY,QZ,QW]),
  matrix_translation(Matrix, [X,Y,Z]), !.

object_pose(Pose, _, mat(Matrix)) :-
  rdfs_individual_of(Pose, knowrob:'Matrix'),
  rotmat_to_list(Pose, Matrix), !.


%% object_dimensions(?Obj:iri, ?Depth:float, ?Width:float, ?Height:float) is semidet
%
% True if Width x Height x Depth are (exactly) the extends of the bounding box of Obj.
%
% @param Obj    Instance of a subclass of EnduringThing-Localized
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
% FIXME: shouldn't it be W-H-D ?
object_dimensions(Obj, Depth, Width, Height) :-
  owl_has(Obj, knowrob:'boundingBoxSize', literal(type(_, ScaleVector))),
  parse_vector(ScaleVector, [Depth, Width, Height]),!.
  
object_dimensions(Obj, Depth, Width, Height) :-
  owl_has(Obj, knowrob:depthOfObject,  literal(type(_, Depth_))),
  owl_has(Obj, knowrob:widthOfObject,  literal(type(_, Width_))),
  owl_has(Obj, knowrob:heightOfObject, literal(type(_, Height_))),
  ((number(Depth_), Depth=Depth_); atom_number(Depth_, Depth)),
  ((number(Width_), Width=Width_); atom_number(Width_, Width)),
  ((number(Height_), Height=Height_); atom_number(Height_, Height)),!.

object_dimensions(Obj, Depth, Width, Height) :-
  owl_has(Obj, srdl2comp:'box_size', literal(type(_, ScaleVector))),
  parse_vector(ScaleVector, [Depth, Width, Height]), !.

object_dimensions(Obj, 0, 0, 0) :-
  rdfs_individual_of(Obj, knowrob:'Point'), !.

%% object_assert_dimensions(+Obj:iri, +Depth:float, +Width:float, +Height:float) is det
%
% Assert object dimension properties.
%
% @param Obj    Instance of a subclass of EnduringThing-Localized
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_assert_dimensions(Obj, Depth, Width, Height) :-
  atomic_list_concat([Depth, Width, Height], ' ', V),
  rdf_assert(Obj, knowrob:'boundingBoxSize', literal(type(xsd:string, V))).

%% object_color(?Obj:iri, ?Col:list) is det
%
% Get the main color of the object.
% The color is returned as [float red, green, blue, alpha], on a scale of 0-1.
% If there is no color given for an object in the knowledge base,
% then a default [0.5, 0.5, 0.5, 1] is returned.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color(Obj, Col) :-
  holds(knowrob:mainColorOfObject(Obj, literal(type(_, ColAtom)))),
  parse_vector(ColAtom, Col), !.
object_color(_Obj, [0.5, 0.5, 0.5, 1.0]).

%% object_assert_color(+Obj:iri, +Col:list) is det
%
% Assert object main color property.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_assert_color(Obj, [R,G,B]) :-
  object_assert_color(Obj, [R,G,B,1.0]), !.
object_assert_color(Obj, [R,G,B,A]) :-
  atomic_list_concat([R,G,B,A], ' ', ColRGBA),
  object_assert_color(Obj, ColRGBA), !.
object_assert_color(Obj, Col) :-
   atom(Col),
   rdf_assert(Obj, knowrob:'mainColorOfObject',literal(type(xsd:string, Col))), !.

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det.
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) :-
  holds(knowrob:pathToCadModel(ObjectId, literal(type(_, FilePath)))).

% distance in 3d
% TODO: distance computable instead!
%    - add distance datatype property
%    - use qudt unit
%    - there are classes called Distance and such, remove them?
object_distance(A,B,D):-
  current_object_pose(A, [AX,AY,AZ,_,_,_,_]),
  current_object_pose(B, [BX,BY,BZ,_,_,_,_]),
  DX is AX - BX,
  DY is AY - BY,
  DZ is AZ - BZ,
  D is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Reasoning about function and storage location of objects
% TODO: add utility rules resoning about object function (i.e., how to use it for action?)

%% storagePlaceFor(St, ObjT) is nondet.
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% 
storagePlaceFor(St, ObjT) :-
  storagePlaceForBecause(St, ObjT, _).

%% storagePlaceForBecause(St, ObjType, ObjT) is nondet.
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% In addition to the storage place, this predicate further returns the superclass
% of Obj for which this information is asserted (e.g. Perishable)
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% @param ObjType  Class for which information about the storage place has been asserted
%

% two instances
storagePlaceForBecause(St, Obj, ObjT) :-
  owl_subclass_of(StT, knowrob:'StorageConstruct'),
  owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-containerFor', some_values_from(ObjT))),
  owl_individual_of(Obj, ObjT),
  owl_individual_of(St, StT).

% obj type, storage instance
storagePlaceForBecause(St, ObjType, ObjT) :-
  owl_subclass_of(StT, knowrob:'StorageConstruct'),
  owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-containerFor', some_values_from(ObjT))),
  owl_individual_of(St, StT),
  owl_subclass_of(ObjType, ObjT).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Joints of objects
  
%% create_joint_information(+Type, +Parent, +Child, +Pose, +Direction, +Radius, +Qmin, +Qmax, -Joint) is det.
%
% Create a joint of class Type at pose Pose, linking Parent and Child
% Qmin and Qmax are joint limits as used in the ROS articulation stack
%
% Usage:
% create_joint_information('HingedJoint', knowrob:'cupboard1', knowrob:'door1', [1,0,0,...], [], 0.23, 0.42, -Joint)
% create_joint_information('PrismaticJoint', knowrob:'cupboard1', knowrob:'drawer1', [1,0,0,...], [1,0,0], 0.23, 0.42, -Joint)
%
% @param Type       Type of the joint instance (knowrob:HingedJoint or knowrob:PrismaticJoint)
% @param Parent     Parent object instance (e.g. cupboard)
% @param Child      Child object instance (e.g. door)
% @param Pose       Pose matrix of the joint as list float[16]
% @param Direction  Direction vector of the joint. float[3] for prismatic joints, [] for rotational joints
% @param Radius     Radius of a rotational joint
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
% @param Joint      Joint instance that has been created
%
create_joint_information(Type, Parent, Child, Pose, Dir, Radius, Qmin, Qmax, Joint) :-

  % create individual
  create_object_perception(Type, Pose, ['TouchPerception'], Joint),

  % set parent and child
  rdf_assert(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_assert(Joint, knowrob:'connectedTo-Rigidly', Child),
  rdf_assert(Joint, knowrob:'connectedTo-Rigidly', Parent),

  % set joint limits
  rdf_assert(Joint, knowrob:'minJointValue', literal(type(xsd:float, Qmin))),
  rdf_assert(Joint, knowrob:'maxJointValue', literal(type(xsd:float, Qmax))),

  % set joint-specific information
  ( (Type = 'PrismaticJoint') -> (

      Dir = [DirX, DirY, DirZ],

      rdf_assert(Parent, knowrob:'prismaticallyConnectedTo', Child),

      rdf_instance_from_class(knowrob:'Vector', DirVec),
      rdf_assert(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
      rdf_assert(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
      rdf_assert(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),

      rdf_assert(Joint, knowrob:'direction', DirVec)

    ) ; (
      rdf_assert(Parent, knowrob:'hingedTo', Child),
      rdf_assert(Joint, knowrob:'turnRadius', literal(type(xsd:float, Radius)))
    ) ).

%% update_joint_information(+Joint, +Type, +Pose, +Direction, +Radius, +Qmin, +Qmax)
%
% Update type, pose and articulation information for a joint after creation.
% Leaves Parent and Child untouched, i.e. assumes that only the estimated
% joint parameters have changed.
%
% @param Joint      Joint instance to be updated
% @param Type       Type of the joint instance (knowrob:HingedJoint or knowrob:PrismaticJoint)
% @param Pose       Pose matrix of the joint as list float[16]
% @param Direction  Direction vector of the joint. float[3] for prismatic joints, [] for rotational joints
% @param Radius     Radius of a rotational joint
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
%
update_joint_information(Joint, Type, Pose, Dir, Radius, Qmin, Qmax) :-

  % % % % % % % % % % % % % % % % % % %
  % update joint type
  rdf_retractall(Joint, rdf:type, _),
  rdf_assert(Joint, rdf:type, Type),

  % % % % % % % % % % % % % % % % % % %
  % update pose by creating a new perception instance (remembering the old data)
  knowrob_perception:create_perception_instance(['TouchPerception'], Perception),
  knowrob_perception:set_perception_pose(Perception, Pose),
  knowrob_perception:set_object_perception(Joint, Perception),

  % % % % % % % % % % % % % % % % % % %
  % update joint limits
  rdf_retractall(Joint, knowrob:'minJointValue', _),
  rdf_retractall(Joint, knowrob:'maxJointValue', _),
  rdf_assert(Joint, knowrob:'minJointValue', literal(type(xsd:float, Qmin))),
  rdf_assert(Joint, knowrob:'maxJointValue', literal(type(xsd:float, Qmax))),

  % % % % % % % % % % % % % % % % % % %
  % update connectedTo:

  % determine parent/child
  rdf_has(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Parent),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Child),!,

  % retract old connections between parent and child
  rdf_retractall(Parent, knowrob:'prismaticallyConnectedTo', Child),
  rdf_retractall(Parent, knowrob:'hingedTo', Child),

  % remove direction vector if set
  ((rdf_has(Joint, knowrob:direction, OldDirVec),
    rdf_retractall(OldDirVec, _, _),
    rdf_retractall(_, _, OldDirVec)
    ) ; true),

  % set new articulation information
  ( (Type = 'PrismaticJoint') -> (

      Dir = [DirX, DirY, DirZ],

      rdf_assert(Parent, knowrob:'prismaticallyConnectedTo', Child),

      rdf_instance_from_class(knowrob:'Vector', DirVec),
      rdf_assert(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
      rdf_assert(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
      rdf_assert(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),

      rdf_assert(Joint, knowrob:'direction', DirVec)

    ) ; (
      rdf_assert(Parent, knowrob:'hingedTo', Child),
      (rdf_retractall(Joint, knowrob:'turnRadius', _); true),
      rdf_assert(Joint, knowrob:'turnRadius', literal(type(xsd:float, Radius)))
    ) ).

%% read_joint_information(+Joint, -Type, -Parent, -Child, -Pose, -Direction, -Radius, -Qmin, -Qmax) is nondet.
%
% Read information stored about a particular joint.
%
% @param Joint      Joint instance to be read
% @param Type       Type of the joint instance (knowrob:HingedJoint or knowrob:PrismaticJoint)
% @param Parent     Parent object instance (e.g. cupboard)
% @param Child      Child object instance (e.g. door)
% @param Pose       Pose matrix of the joint as list float[16]
% @param Direction  Direction vector of the joint. float[3] for prismatic joints, [] for rotational joints
% @param Radius     Radius of a rotational joint
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
%
read_joint_information(Joint, Type, Parent, Child, Pose, Direction, Radius, Qmin, Qmax) :-

  rdf_has(Joint, rdf:type, Type),

  rdf_has(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_has(Joint, knowrob:'connectedTo', Parent),
  rdf_has(Joint, knowrob:'connectedTo', Child),

  current_object_pose(Joint, Pose),

  ((rdf_has(Joint, knowrob:'direction', DirVec),
    rdf_has(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
    rdf_has(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
    rdf_has(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),
    Direction=[DirX, DirY, DirZ]);
    (Direction=[])),

  (rdf_has(Joint, knowrob:'turnRadius', literal(type(xsd:float, Radius))); (true,!)),

  rdf_has(Joint, knowrob:'minJointValue', literal(type(xsd:float, Qmin))),
  rdf_has(Joint, knowrob:'maxJointValue', literal(type(xsd:float, Qmax))).

%% delete_joint_information(Joint) is det.
%
% Remove joint instance and all information stored about this joint
%
% @param Joint Joint instance to be deleted
%
delete_joint_information(Joint) :-

  % remove pose/perception instances
  % removes timepoint, pose, perception itself
  findall(Perception, (rdf_has(Perception, knowrob:objectActedOn, Joint),
                       rdf_retractall(Perception, _, _)), _),

  % remove connection between parent and child
  rdf_retractall(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_retractall(Joint, knowrob:'connectedTo-Rigidly', Parent),
  rdf_retractall(Joint, knowrob:'connectedTo-Rigidly', Child),

  rdf_retractall(Parent, knowrob:'prismaticallyConnectedTo', Child),
  rdf_retractall(Parent, knowrob:'hingedTo', Child),

  % remove direction vector if set
  ((rdf_has(Joint, knowrob:direction, OldDirVec),
    rdf_retractall(OldDirVec, _, _),
    rdf_retractall(_, _, OldDirVec)
    ) ; true),

  % remove everything directly connected to the joint instance
  rdf_retractall(Joint, _, _),
  rdf_retractall(_, _, Joint).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Asking queries about objects.
% % % % % note: Possible queries may be stated in the ontology.

%% object_queries(+Obj:iri, -Queries:list) is det
%
% gather facts about queries that can be asked about an object.
% queries are represented as [string category, string title, string query].
% Category and title are primary used for displaying possible queries
% to the user.
%
% @param Obj the object name
% @param Queries list of queries that can be asked about Obj
%
object_queries(Obj, Queries) :-
  findall([Category,Title,Query],
          object_query(Obj,Category,Title,Query),
          QueriesUnsorted),
  sort(QueriesUnsorted, Queries).

%% object_query(+Obj:iri, ?QueryGroup:atom, ?QueryTitle:atom, ?Query:atom) is det
%
% True for objects Obj for which a query exists belonging to the group QueryGroup
% and labeled with QueryTitle.
%
% @param Obj the object name
% @param QueryGroup category of query
% @param QueryTitle name of the query
% @param Query the Prolog-encoded query string
%
object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific individuals
  rdf_has(QueryIndividual, knowrob:'queryAbout', Obj),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).

object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific types
  rdfs_individual_of(Obj, IndividualClass),
  % FIXME: queryAbout some Class is non OWL! use restrictions instead!
  rdf_has(QueryIndividual, knowrob:'queryAbout', IndividualClass),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).
