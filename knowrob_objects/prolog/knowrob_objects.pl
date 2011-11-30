%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides methods for reasoning about objects
%% in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


:- module(knowrob_objects,
    [
      storagePlaceFor/2,
      storagePlaceForBecause/3,
      current_object_pose/2,
      rotmat_to_list/2,
      create_joint_information/8,
      update_joint_information/6,
      read_joint_information/8,
      delete_joint_information/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_perception')).


:-  rdf_meta
    storagePlaceFor(r,r),
    storagePlaceForBecause(r,r,r),
    current_object_pose(r,-),
    rotmat_to_list(r,-),
    create_joint_information(r, r, r, +, ?, +, +, r),
    update_joint_information(r, r, +, ?, +, +),
    read_joint_information(r, r, r, r, -, -, -, -),
    delete_joint_information(r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).



  storagePlaceFor(St, ObjT) :-
    storagePlaceForBecause(St, ObjT, _).

  % two instances
  storagePlaceForBecause(St, Obj, ObjT) :-
    owl_subclass_of(StT, knowrob:'StorageConstruct'),
    owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-StoragePlaceFor', some_values_from(ObjT))),
    owl_individual_of(Obj, ObjT),
    owl_individual_of(St, StT).

  % obj type, storage instance
  storagePlaceForBecause(St, ObjType, ObjT) :-
    owl_subclass_of(StT, knowrob:'StorageConstruct'),
    owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-StoragePlaceFor', some_values_from(ObjT))),
    owl_individual_of(St, StT),
    owl_subclass_of(ObjType, ObjT).



%% current_object_pose(+ObjInstance, -PoseList) is det.
%
% Get the pose of an object based on the latest perception
%
current_object_pose(Obj, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#orientation',Obj,Pose),!,
  rotmat_to_list(Pose, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]).



%% rotmat_to_list(+RotMatInstance, -PoseList) is det.
%
% Read the pose values for an instance of a rotation matrix
%
rotmat_to_list(Pose, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m00',Pose,M00literal), strip_literal_type(M00literal, M00a), term_to_atom(M00, M00a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m01',Pose,M01literal), strip_literal_type(M01literal, M01a), term_to_atom(M01, M01a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m02',Pose,M02literal), strip_literal_type(M02literal, M02a), term_to_atom(M02, M02a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m03',Pose,M03literal), strip_literal_type(M03literal, M03a), term_to_atom(M03, M03a),

    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m10',Pose,M10literal), strip_literal_type(M10literal, M10a), term_to_atom(M10, M10a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m11',Pose,M11literal), strip_literal_type(M11literal, M11a), term_to_atom(M11, M11a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m12',Pose,M12literal), strip_literal_type(M12literal, M12a), term_to_atom(M12, M12a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m13',Pose,M13literal), strip_literal_type(M13literal, M13a), term_to_atom(M13, M13a),

    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m20',Pose,M20literal), strip_literal_type(M20literal, M20a), term_to_atom(M20, M20a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m21',Pose,M21literal), strip_literal_type(M21literal, M21a), term_to_atom(M21, M21a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m22',Pose,M22literal), strip_literal_type(M22literal, M22a), term_to_atom(M22, M22a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m23',Pose,M23literal), strip_literal_type(M23literal, M23a), term_to_atom(M23, M23a),

    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m30',Pose,M30literal), strip_literal_type(M30literal, M30a), term_to_atom(M30, M30a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m31',Pose,M31literal), strip_literal_type(M31literal, M31a), term_to_atom(M31, M31a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m32',Pose,M32literal), strip_literal_type(M32literal, M32a), term_to_atom(M32, M32a),
    rdf_triple('http://ias.cs.tum.edu/kb/knowrob.owl#m33',Pose,M33literal), strip_literal_type(M33literal, M33a), term_to_atom(M33, M33a),!.




%% create_joint_information(+Type, +Parent, +Child, +Pose, +Direction, +Qmin, +Qmax, -Joint) is det.
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
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
% @param Joint      Joint instance that has been created
%
create_joint_information(Type, Parent, Child, Pose, [DirX, DirY, DirZ], Qmin, Qmax, Joint) :-

  % create individual
  create_object_perception(Type, Pose, ['TouchPerception'], Joint),

  % set parent and child
  rdf_assert(Joint, knowrob:'properPhysicalParts', Child),
  rdf_assert(Joint, knowrob:'connectedTo-Rigidly', Parent),

  % set joint limits
  rdf_assert(Joint, knowrob:'minJointValue', literal(type(xsd:float, Qmin))),
  rdf_assert(Joint, knowrob:'maxJointValue', literal(type(xsd:float, Qmax))),

  % set joint-specific information
  ( (Type = 'PrismaticJoint') -> (
      rdf_assert(Parent, knowrob:'prismaticallyConnectedTo', Child),

      rdf_instance_from_class(knowrob:'Vector', DirVec),
      rdf_assert(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
      rdf_assert(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
      rdf_assert(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),

      rdf_assert(Joint, knowrob:'direction', DirVec)

    ) ; (
      rdf_assert(Parent, knowrob:'hingedTo', Child)
    ) ).

%% update_joint_information(+Joint, +Type, +Pose, +Direction, +Qmin, +Qmax)
%
% Update type, pose and articulation information for a joint after creation.
% Leaves Parent and Child untouched, i.e. assumes that only the estimated
% joint parameters have changed.
%
% @param Joint      Joint instance to be updated
% @param Type       Type of the joint instance (knowrob:HingedJoint or knowrob:PrismaticJoint)
% @param Pose       Pose matrix of the joint as list float[16]
% @param Direction  Direction vector of the joint. float[3] for prismatic joints, [] for rotational joints
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
%
update_joint_information(Joint, Type, Pose, [DirX, DirY, DirZ], Qmin, Qmax) :-

  % % % % % % % % % % % % % % % % % % %
  % update joint type
  rdf_retractall(Joint, rdf:type, _),
  rdf_assert(Joint, rdf:type, Type),

  % % % % % % % % % % % % % % % % % % %
  % update pose by creating a new perception instance (remembering the old data)
  create_perception_instance(['TouchPerception'], Perception),
  set_perception_pose(Perception, Pose),
  set_object_perception(Joint, Perception),

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
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Child),

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
      rdf_assert(Parent, knowrob:'prismaticallyConnectedTo', Child),

      rdf_instance_from_class(knowrob:'Vector', DirVec),
      rdf_assert(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
      rdf_assert(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
      rdf_assert(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),

      rdf_assert(Joint, knowrob:'direction', DirVec)

    ) ; (
      rdf_assert(Parent, knowrob:'hingedTo', Child)
    ) ).



%% read_joint_information(+Joint, -Type, -Parent, -Child, -Pose, -Direction, -Qmin, -Qmax) is nondet.
%
% Read information stored about a particular joint.
%
% @param Joint      Joint instance to be read
% @param Type       Type of the joint instance (knowrob:HingedJoint or knowrob:PrismaticJoint)
% @param Parent     Parent object instance (e.g. cupboard)
% @param Child      Child object instance (e.g. door)
% @param Pose       Pose matrix of the joint as list float[16]
% @param Direction  Direction vector of the joint. float[3] for prismatic joints, [] for rotational joints
% @param Qmin       Minimal configuration value (joint limit)
% @param Qmax       Minimal configuration value (joint limit)
%
read_joint_information(Joint, Type, Parent, Child, Pose, Direction, Qmin, Qmax) :-

  rdf_has(Joint, rdf:type, Type),

  rdf_has(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Parent),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Child),

  current_object_pose(Joint, Pose),

  ((rdf_has(Joint, knowrob:'direction', DirVec),
    rdf_has(DirVec, knowrob:'vectorX', literal(type(xsd:float, DirX))),
    rdf_has(DirVec, knowrob:'vectorY', literal(type(xsd:float, DirY))),
    rdf_has(DirVec, knowrob:'vectorZ', literal(type(xsd:float, DirZ))),
    Direction=[DirX, DirY, DirZ]);
    (Direction=[])),

  rdf_has(Joint, knowrob:'minJointValue', literal(type(xsd:float, Qmin))),
  rdf_has(Joint, knowrob:'maxJointValue', literal(type(xsd:float, Qmax))).



%% delete_joint_information(Joint) is det.
%
% Remove joint instance and all information stored for this joint
%
% @param Joint Joint instance to be deleted
%
delete_joint_information(Joint) :-

  % remove pose/perception instances
  % removes timepoint, pose, perception itself
  findall(Perception, (rdf_has(Perception, knowrob:objectActedOn, Joint),
                       rdf_retractall(Perception, _, _)), _),

  % remove connection between parent and child
  rdf_has(Parent, knowrob:'properPhysicalParts', Joint),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Parent),
  rdf_has(Joint, knowrob:'connectedTo-Rigidly', Child),

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







% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% KnowRob-Base: holds() and related predicates
%


%% holds_tt(+Goal, +StartEndList) is nondet.
%
% General definition of holds_tt that uses holds(..) to check if a relation
% holds throughout a time span (i.e. for each time point during the time span)
%
% @param Goal  The goal that is to be checked
% @param StartEndList Start time and end time of the time span under consideration [Start, End]
%
holds_tt(Goal, [Start, End]) :-

    rdf_assert(knowrob:'holds_tt', rdf:type, knowrob:'TimeInterval'),
    rdf_assert(knowrob:'holds_tt', knowrob:startTime, Start),
    rdf_assert(knowrob:'holds_tt', knowrob:endTime,   End),

    holds(Goal, Start),
    holds(Goal, End),

    % find all detections of the objects at hand
    arg(1, Goal, Arg1),arg(2, Goal, Arg2),
    findall([D_i,Arg1], ( (rdf_has(D_i, knowrob:objectActedOn, Arg1);rdf_has(D_i, knowrob:objectActedOn, Arg2)),
                           rdfs_individual_of(D_i,  knowrob:'MentalEvent')), Detections),

      forall( ( member(D_O, Detections), nth0(0, D_O, Detection),
                rdf_triple(knowrob:startTime, Detection, DStT),
                rdf_triple(knowrob:temporallySubsumes, knowrob:'holds_tt', DStT) ),
              holds(Goal, DStT) ),

    rdf_retractall(knowrob:'holds_tt', _, _).



%% latest_detection_of_instance(+Object, -LatestDetection) is nondet.
%
% Get the lastest detection of the object instance Object
%
% A detection is an instance of MentalEvent, i.e. can be a perception
% process as well as an inference result
%
% @param Object          An object instance
% @param LatestDetection Latest MentalEvent associated with this instance
%
latest_detection_of_instance(Object, LatestDetection) :-

    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
                              rdf_triple(knowrob:startTime, D_i, StTg),
                              rdf_split_url(_, StTl, StTg),
                              atom_concat('timepoint_', StTa, StTl),
                              term_to_atom(St, StTa)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection).



%% latest_detection_of_type(+Type, -LatestDetection) is nondet.
%
% Get the lastest detection of an object of type Type
%
% A detection is an instance of MentalEvent, i.e. can be a perception
% process as well as an inference result
%
% @param Object          An object type
% @param LatestDetection Latest MentalEvent associated with any instance of this type
%
latest_detection_of_type(Type, LatestDetection) :-

    findall([D_i,Object,St], (rdfs_individual_of(Object, Type),
                              rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
                              rdf_triple(knowrob:startTime, D_i, StTg),
                              rdf_split_url(_, StTl, StTg),
                              atom_concat('timepoint_', StTa, StTl),
                              term_to_atom(St, StTa)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection).


%% latest_perception_of_type(+Type, -LatestPerception) is nondet.
%
% Get the lastest perception of an object of type Type
%
% @param Object          An object type
% @param LatestPerception Latest MentalEvent associated with any instance of this type
%
latest_perception_of_type(Type, LatestPerception) :-

    findall([P_i,Object,St], (rdfs_individual_of(Object, Type),
                              rdf_has(P_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(P_i,  knowrob:'VisualPerception'),
                              rdf_triple(knowrob:startTime, P_i, StTg),
                              rdf_split_url(_, StTl, StTg),
                              atom_concat('timepoint_', StTa, StTl),
                              term_to_atom(St, StTa)), Perceptions),

    predsort(compare_object_detections, Perceptions, Psorted),

    % compute the homography for the newest perception
    nth0(0, Psorted, Latest),
    nth0(0, Latest, LatestPerception).


%% latest_perceptions_of_types(+Type, -LatestPerceptions) is nondet.
%
% Get the lastest perceptions of all objects of type Type
%
% @param Object          An object type
% @param LatestPerceptions Latest MentalEvents associated with instances of this type
%
latest_perceptions_of_types(Type, LatestPerceptions) :-

    findall(Obj, rdfs_individual_of(Obj, Type), Objs),

    findall(LatestDetection,
            ( member(Object, Objs),
              latest_detection_of_instance(Object, LatestDetection),
              rdfs_individual_of(LatestDetection, knowrob:'VisualPerception') ),
            LatestPerceptions).



%% latest_inferred_object_set(-Object) is nondet.
%
% Ask for the objects inferred in the last inference run
%
% @param Objects   Set of object instances inferred in the latest inference run
%
latest_inferred_object_set(Objects) :-

    findall([D_i,_,St],  (rdfs_individual_of(D_i,  knowrob:'Reasoning'),
                          rdf_has(Inf, knowrob:probability, InfProb),
                          term_to_atom(Prob, InfProb),
                          >(Prob, 0),
                          rdf_triple(knowrob:startTime, D_i, StTg),
                          rdf_split_url(_, StTl, StTg),
                          atom_concat('timepoint_', StTa, StTl),
                          term_to_atom(St, StTa)), Inferences),

    predsort(compare_object_detections, Inferences, Psorted),

    % compute the newest perception
    nth0(0, Psorted, Latest),
    nth0(0, Latest, LatestInf),

    % find other inferences performed at the same time
    findall(OtherInf, (rdf_has(LatestInf, knowrob:'startTime', St), rdf_has(OtherInf, knowrob:'startTime', St)), OtherInfs),

    predsort(compare_inferences_by_prob, OtherInfs, SortedInfs),

    findall(Obj, (member(Inf, SortedInfs), rdf_has(Inf, knowrob:'objectActedOn', Obj)), Objects).


%% latest_inferred_object_types(-ObjectTypes) is nondet.
%
% Ask for the object types inferred in the last inference run
%
% @param ObjectTypes   Set of object types inferred in the latest inference run
%
latest_inferred_object_types(ObjectTypes) :-

    latest_inferred_object_set(Objects),
    findall(ObjT, (member(Obj, Objects), rdf_has(Obj, rdf:type, ObjT)), ObjectTypes).




%% object_detection(+Object, ?Time, -Detection) is nondet.
%
% Find all detections of the Object that are valid at time point Time
%
% @param Object     Object instance of interest
% @param Time       Time point of interest. If unbound, all detections of the object are returned.
% @param Detection  Detections of the object that are assumed to be valid at time Time
%
object_detection(Object, Time, Detection) :-

    findall([D_i,Object], (rdf_has(D_i, knowrob:objectActedOn, Object),
                           rdfs_individual_of(D_i,  knowrob:'MentalEvent')), Detections),

    member(P_O, Detections),
    nth0(0, P_O, Detection),
    nth0(1, P_O, Object),

    ((var(Time))
      -> (
        true
      ) ; (
        temporally_subsumes(Detection, Time)
    )).


%% temporally_subsumes(+Long, +Short) is nondet.
%
% Verify whether Long temporally subsumes Short
%
% @param Long   The longer time span (e.g. detection of an object)
% @param Short  The shorter time span (e.g. detection of an object)
%
temporally_subsumes(Long, Short) :-

      detection_starttime(Long, LongSt),!,
      detection_endtime(Long,   LongEt),!,

      detection_starttime(Short, ShortSt),!,
      detection_endtime(Short,   ShortEt),!,

      % compare the start and end times
      (ShortSt=<ShortEt),
      (LongSt=<ShortSt), (ShortSt<LongEt),
      (LongSt=<ShortEt), (ShortEt<LongEt).


%% detection_starttime(+Detection, -StartTime) is nondet.
%
% Determine the start time of an object detection as numerical value.
% Simply reads the asserted knowrob:startTime and transforms the timepoint
% into a numeric value.
%
% @param Detection  Instance of an event with asserted startTime
% @param StartTime  Numeric value describing the start time
%
detection_starttime(Detection, StartTime) :-

  % start time is asserted
  rdf_triple(knowrob:startTime, Detection, StartTtG),
  rdf_split_url(_, StartTt, StartTtG),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom).


%% detection_endtime(+Detection, -EndTime) is nondet.
%
% Determine the end time of an object detection as numerical value.
% If the knowrob:endTime is asserted, it is read and and transformed
% into a numeric value. Otherwise, the predicate searches for later
% perceptions of the same object and takes the startTime of the first
% subsequent detection as the endTime of the current detection. If
% there is neither an asserted endTime nor any later detection of the
% object, it is assumed that the observation is still valid and the
% current time + 1s is returned (to avoid problems with time glitches).
%
% @param Detection  Instance of an event
% @param EndTime    Numeric value describing the ent time
%
detection_endtime(Detection, EndTime) :-

  % end time is asserted
  rdf_triple(knowrob:endTime, Detection, EndTtG),
  rdf_split_url(_, EndTt, EndTtG),
  atom_concat('timepoint_', EndTAtom, EndTt),
  term_to_atom(EndTime, EndTAtom),!;

  % search for later detections of the object
  ( rdf_has(Detection, knowrob:objectActedOn, Object),
    rdf_has(LaterDetection, knowrob:objectActedOn, Object),
    LaterDetection \= Detection,
    rdfs_individual_of(LaterDetection,  knowrob:'MentalEvent'),
    rdf_triple(knowrob:startTime, Detection, StT),
    rdf_triple(knowrob:startTime, LaterDetection, EndTtG),
    rdf_triple(knowrob:after, StT, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % check if the object has been destroyed in the meantime
  ( rdf_has(Detection, knowrob:objectActedOn, Object),
    rdf_has(Destruction, knowrob:inputsDestroyed, Object),
    Destruction \= Detection,
    rdfs_individual_of(Destruction,  knowrob:'PhysicalDestructionEvent'),
    rdf_triple(knowrob:startTime, Detection, StT),
    rdf_triple(knowrob:startTime, Destruction, EndTtG),
    rdf_triple(knowrob:after, StT, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % otherwise take the current time (plus a second to avoid glitches)
  ( get_time(ET), EndTime is ET + 1.0).




%% compare_object_detections(-Delta, +P1, +P2) is det.
%
% Sort detections by their start time
%
% @param Delta  One of '>', '<', '='
% @param P1     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
%
compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).

%% compare_inferences_by_prob(-Delta, +P1, +P2) is det.
%
% Sort inference results by their probability
%
% @param Delta  One of '>', '<'
% @param P1     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
%

compare_inferences_by_prob('>', Inf1, Inf2) :-
  rdf_has(Inf1,knowrob:probability,Pr1), term_to_atom(P1,Pr1),
  rdf_has(Inf2,knowrob:probability,Pr2), term_to_atom(P2,Pr2),
  P1 < P2.

compare_inferences_by_prob('<', Inf1, Inf2) :-
  rdf_has(Inf1,knowrob:probability,Pr1), term_to_atom(P1,Pr1),
  rdf_has(Inf2,knowrob:probability,Pr2), term_to_atom(P2,Pr2),
  P1 >= P2.
