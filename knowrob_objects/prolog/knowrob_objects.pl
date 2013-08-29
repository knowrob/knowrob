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
      object_pose_at_time/3,
      rotmat_to_list/2,
      create_joint_information/9,
      update_joint_information/7,
      read_joint_information/9,
      delete_joint_information/1,
      delete_object_information/1,
      delete_object_information_recursive/1,
      object_detection/3,
      comp_m00/2,
      comp_m01/2,
      comp_m02/2,
      comp_m03/2,
      comp_m04/2,
      comp_m05/2,
      comp_m10/2,
      comp_m11/2,
      comp_m12/2,
      comp_m13/2,
      comp_m14/2,
      comp_m15/2,
      comp_m20/2,
      comp_m21/2,
      comp_m22/2,
      comp_m23/2,
      comp_m23/2,
      comp_m25/2,
      comp_m30/2,
      comp_m31/2,
      comp_m32/2,
      comp_m33/2,
      comp_m34/2,
      comp_m35/2,
      comp_m40/2,
      comp_m41/2,
      comp_m42/2,
      comp_m43/2,
      comp_m44/2,
      comp_m45/2,
      comp_m50/2,
      comp_m51/2,
      comp_m52/2,
      comp_m53/2,
      comp_m54/2,
      comp_m55/2,
      comp_xCoord/2,
      comp_yCoord/2,
      comp_zCoord/2,
      comp_orientation/2,
      instantiate_at_position/3,
      update_instance_from_class_def/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_perception')).

:- owl_parser:owl_parse('../owl/knowrob_objects.owl', false, false, true).



:-  rdf_meta
    storagePlaceFor(r,r),
    storagePlaceForBecause(r,r,r),
    current_object_pose(r,-),
    current_object_pose(r,r,-),
    rotmat_to_list(r,-),
    comp_orientation(r, r),
    instantiate_at_position(r,+,r),
    transform_relative_to(r,r,-),
    update_instance_from_class_def(r,r),
    create_joint_information(r, r, r, +, ?, +, +, +, r),
    update_joint_information(r, r, +, ?, +, +, +),
    read_joint_information(r, r, r, r, -, -, -, -, -),
    delete_joint_information(r),
    delete_object_information(r),
    delete_object_information_recursive(r),
    comp_xCoord(r, r), comp_yCoord(r, r), comp_zCoord(r, r),
    comp_m00(r, r),    comp_m01(r, r),    comp_m02(r, r),    comp_m03(r, r),    comp_m04(r, r),    comp_m05(r, r),
    comp_m10(r, r),    comp_m11(r, r),    comp_m12(r, r),    comp_m13(r, r),    comp_m14(r, r),    comp_m15(r, r),
    comp_m20(r, r),    comp_m21(r, r),    comp_m22(r, r),    comp_m23(r, r),    comp_m23(r, r),    comp_m25(r, r),
    comp_m30(r, r),    comp_m31(r, r),    comp_m32(r, r),    comp_m33(r, r),    comp_m34(r, r),    comp_m35(r, r),
    comp_m40(r, r),    comp_m41(r, r),    comp_m42(r, r),    comp_m43(r, r),    comp_m44(r, r),    comp_m45(r, r),
    comp_m50(r, r),    comp_m51(r, r),    comp_m52(r, r),    comp_m53(r, r),    comp_m54(r, r),    comp_m55(r, r).


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


%% object_pose_at_time(+ObjInstance, +Time, -PoseList) is det.
%
% Get the pose of an object based on the latest perception before Time
%
object_pose_at_time(Obj, Time, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  object_detection(Obj, Time, Detection),
  rdf_triple(knowrob:eventOccursAt, Detection, Pose),!,
  
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








%% instantiate_at_position(+ObjClassDef, +PoseList, -ObjInst) is det.
%
% reads all parts of the object described at the class level (ObjClassDef)
% and instantiates the object such that the main object is at pose PoseList
% and all physical parts of that object are at the correct poses relative
% to PoseList. Relies on the physical parts being specified using a poseRelativeTo
% description
%
% @param ObjClassDef Object class with physical parts described by restrictions on properPhysicalParts
% @param PoseList    List of numeric values describing the pose where the center of the main object is to be instantiated
% @param ObjInst     Instance of ObjClassDef that was created (and that is linked to the instances of the physical parts that have also been created)
%
:- dynamic class_to_pose/2.

instantiate_at_position(ObjClassDef, PoseList, ObjInst) :-

    % create main object
    create_object_perception(ObjClassDef, PoseList, ['VisualPerception'], ObjInst),

    % remember class-pose relation
    asserta(class_to_pose(ObjClassDef, PoseList)),

    % read parts of the object, check if their poses are relative to
    % something else, and transform them before instantiation
    findall(PartInst, (
        instantiate_physical_part(ObjClassDef, ObjInst, PartInst)
    ), _Parts),


    % read object properties
    findall([Cinst, P, Oinst], (class_to_pose(C, _),
                     class_properties_nosup(C, P, O),
                     owl_individual_of(P, owl:'ObjectProperty'),
                     \+ rdfs_subproperty_of(P, knowrob:parts),
                     \+ rdfs_subproperty_of(P, knowrob:orientation),
                     rdfs_individual_of(Cinst, C),
                     rdfs_individual_of(Oinst, O)), ObjPs),
    sort(ObjPs, ObjPsSorted),

    findall(O,(member([Cinst,P,O], ObjPsSorted),
               rdf_assert(Cinst, P, O)), _ObjRestrs),

    % read data properties
    findall([Cinst, P, O], (class_to_pose(C, _),
                     class_properties(C, P, O),
                     rdfs_individual_of(Cinst, C),
                     owl_individual_of(P, owl:'DatatypeProperty')), DataPs),
    sort(DataPs, DataPsSorted),

    findall(O, (member([Cinst, P,O], DataPsSorted),
                rdf_assert(Cinst, P, O)), _Os),

    retractall(class_to_pose(_,_)).


%% instantiate_physical_part(-ObjClassDef, -ObjInst, -PartInst)
%
% Internal helper predicate to recursively read the physical parts of an object
% and create the respective instances together with the correctly transformed
% poses.
%
% @param ObjClassDef Object class, being the type of ObjInst and further describing physical parts in terms of restrictions
% @param ObjInst     Instance of ObjClassDef for which the physical parts are to be created
% @param PartInst    Instance of the physical part that is created by this predicate
%
instantiate_physical_part(ObjClassDef, ObjInst, PartInst) :-

    findall(P, class_properties(ObjClassDef, knowrob:properPhysicalParts, P), Ps),
    member(Part, Ps),
print('Ps: '), print(Ps), print('\n'),
    findall(Pp, class_properties(Part, knowrob:orientation, Pp), Pps),
    member(PartPose, Pps),

    % transform into global coordinates if relativeTo relation is given
    (( owl_has(PartPose, knowrob:relativeTo, PartRef),
       class_to_pose(PartRef, RefPose),
       knowrob_objects:rotmat_to_list(PartPose, PartPoseList),
       pose_into_global_coord(PartPoseList, RefPose, PartPoseGlobal) ) ;
    (  PartPoseGlobal = PartPose) ),

    create_object_perception(Part, PartPoseGlobal, ['VisualPerception'], PartInst),
    rdf_assert(ObjInst, knowrob:properPhysicalParts, PartInst),
print('PartInst: '), print(PartInst), print('\n'),
    asserta(class_to_pose(Part, PartPoseGlobal)),

    % recursively create parts
    instantiate_physical_part(Part, PartInst, _).







%% update_instance_from_class_def(+ObjClassDef, -ObjInst) is det.
%
% Update a complex object instance based on the TBOX definition in ObjClassDef.
%
% The main use case is that an object, composed of multiple parts (e.g. a
% cupboard with a door) has been instantiated in a map, and the class-level
% description of this type of object has changed (e.g. including joint parameters
% that have been estimated). In this case, this predicate allows to update the
% existing instance according to the class description.
%
% @param ObjClassDef Object class with physical parts described by restrictions on properPhysicalParts
% @param ObjInst     Instance of ObjClassDef that was created (and that is linked to the instances of the physical parts that have also been created)
%
:- dynamic class_to_inst/2.

update_instance_from_class_def(ObjClassDef, ObjInst) :-

%     % create main object
%     create_object_perception(ObjClassDef, PoseList, ['VisualPerception'], ObjInst),

    % remember class-inst relation
    asserta(class_to_inst(ObjClassDef, ObjInst)),

    % read parts of the object, check if their poses are relative to
    % something else, and transform them before instantiation
    findall(PartInst, (
        update_physical_part_from_class_def(ObjClassDef, ObjInst, PartInst)
    ), _Parts),

    % collect non-fulfilled object properties
    findall([Inst, P, Oinst], (find_missing_objprops(Inst, P, Oinst)), ObjPs),
    sort(ObjPs, ObjPsSorted),print(ObjPsSorted),

    findall(O,(member([Cinst,P,O], ObjPsSorted), % assert missing properties
               rdf_assert(Cinst, P, O)), _ObjRestrs),

    % collect non-fulfilled data properties
    findall([Inst, P, O], (find_missing_dataprops(Inst, P, O)), DataPs),
    sort(DataPs, DataPsSorted),

    findall(O, (member([Cinst, P,O], DataPsSorted),
                rdf_assert(Cinst, P, O)), _Os),

    retractall(class_to_inst(_,_)).


find_missing_objprops(Inst, P, Oinst) :-
    class_to_inst(C, Inst),
    class_properties_nosup(C, P, O),
    owl_individual_of(P, owl:'ObjectProperty'),
    P\='http://ias.cs.tum.edu/kb/knowrob.owl#spatiallyRelated',
    (rdfs_individual_of(Inst, C) ; (rdfs_individual_of(Inst, Csup), owl_direct_subclass_of(C, Csup))),
    (rdfs_individual_of(Oinst, O) ; (owl_direct_subclass_of(O, Osup), rdfs_individual_of(Oinst, Osup))),
    \+rdf_has(Inst, P, Oinst).

find_missing_dataprops(Inst, P, O) :-
    class_to_inst(C, Inst),
    class_properties_nosup(C, P, O),
    owl_individual_of(P, owl:'DatatypeProperty'),
    (rdfs_individual_of(Inst, C) ; (rdfs_individual_of(Inst, Csup), owl_direct_subclass_of(C, Csup))),
    \+rdf_has(Inst, P, O).


%% update_physical_part_from_class_def(+ObjClassDef, +ObjInst, -PartInst)
%
% Helper predicate for update_instance_from_class_def
%
% @param ObjClassDef Object class, being the type of ObjInst and further describing physical parts in terms of restrictions
% @param ObjInst     Instance of ObjClassDef for which the physical parts are to be created
% @param PartInst    Instance of the physical part that is created by this predicate
%
update_physical_part_from_class_def(ObjClassDef, ObjInst, PartInst) :-

    findall(P, class_properties_nosup(ObjClassDef, knowrob:properPhysicalParts, P), Ps),
    member(Part, Ps),

    findall(Pp, class_properties_nosup(Part, knowrob:orientation, Pp), Pps),
    member(PartPose, Pps),

    % transform into global coordinates if relativeTo relation is given
    (( owl_has(PartPose, knowrob:relativeTo, PartRef),
       ((rdfs_individual_of(RefInst, PartRef),!) ; (owl_direct_subclass_of(PartRef, Csup), rdfs_individual_of(RefInst, Csup),!)),
       current_object_pose(RefInst, RefPose),
       knowrob_objects:rotmat_to_list(PartPose, PartPoseList),
       pose_into_global_coord(PartPoseList, RefPose, PartPoseGlobalList) ) ;
    (  rotmat_to_list(PartPose, PartPoseGlobalList)) ),


    % check if part exists and is a part of obj
    ((owl_direct_subclass_of(Part, PartT), % necessary since export is subClassOf object type
      owl_individual_of(PartInst, PartT),
      owl_has(ObjInst, knowrob:properPhysicalParts, PartInst),!) ->

    ( knowrob_perception:create_perception_instance(['VisualPerception'], Perception), % part exists, only create new perception for parts
      knowrob_perception:set_object_perception(PartInst, Perception),
      knowrob_perception:set_perception_pose(Perception, PartPoseGlobalList),
      rdf_assert(ObjInst, knowrob:properPhysicalParts, PartInst)) ;

    ( create_object_perception(Part, PartPoseGlobalList, ['VisualPerception'], PartInst) )), % create part

    asserta(class_to_inst(Part, PartInst)),

    update_physical_part_from_class_def(Part, PartInst, _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% tboxify playground

:- dynamic(tboxified/2).
tboxify_object_inst(ObjInst, ClassName, ReferenceObj, ReferenceObjCl, SourceRef) :-

  assert(tboxified(ObjInst, ClassName)),

  % assert types as superclasses
  findall(T, (rdf_has(ObjInst, rdf:type, T), T\= 'http://www.w3.org/2002/07/owl#NamedIndividual'), Ts),
  findall(T, (member(T, Ts), rdf_assert(ClassName, rdfs:subClassOf, T, SourceRef)), _),

  % check if there is a providesModelFor
  findall(M, (member(T, Ts), rdf_has(M, 'http://www.roboearth.org/kb/roboearth.owl#providesModelFor', T)), Ms),
  findall(M, (member(M, Ms), rdf_assert(M, 'http://www.roboearth.org/kb/roboearth.owl#providesModelFor', ClassName, SourceRef)), _),


  % read object pose if ReferenceObj is set (obj is part of another obj)
  ((ReferenceObj \= ObjInst,

    % read pose and transform into relative pose
    transform_relative_to(ObjInst, ReferenceObj, RelativePoseList),

    % create new pose matrix instance
    create_pose(RelativePoseList, RelativePose),
    rdf_assert(RelativePose, knowrob:relativeTo, ReferenceObjCl),

    % add pose restriction to the class definition
    create_restr(ClassName, knowrob:orientation, RelativePose, owl:hasValue, SourceRef, _PoseRestr),!)
  ; true),


  % read object properties
  findall([P, O], (rdf_has(ObjInst, P, O),
                   owl_individual_of(P, owl:'ObjectProperty'),
                   \+ rdfs_subproperty_of(P, knowrob:parts), % physicalParts and connectedTo need to refer to class descriptions
                   \+ rdfs_subproperty_of(P, knowrob:connectedTo)), ObjPs),
  sort(ObjPs, ObjPsSorted),

  findall(ObjRestr,(member([P,O], ObjPsSorted),
                    create_restr(ClassName, P, O, owl:someValuesFrom, SourceRef, ObjRestr)), _ObjRestrs),

  % read data properties
  findall([P, O], ((rdf_has(ObjInst, P, O),
                    owl_individual_of(P, owl:'DatatypeProperty'));
                   (rdf_has(ObjInst, rdf:type, T),
                    P = 'http://ias.cs.tum.edu/kb/knowrob.owl#pathToCadModel',
                    class_properties(T, P, O))), DataPs),
  sort(DataPs, DataPsSorted),

  findall(DataRestr, (member([P,O], DataPsSorted),
                      create_restr(ClassName, P, O, owl:hasValue, SourceRef, DataRestr)), _DataRestrs),



  % iterate over physicalParts
  findall(Part, (rdf_has(ObjInst, P, Part),
                 rdfs_subproperty_of(P, knowrob:parts)), Parts),
  sort(Parts, PartsSorted),

  findall(Part, (member(Part, PartsSorted),
                 ((tboxified(Part, PartClassName)) -> true ;
                  (rdf_unique_class_id('http://ias.cs.tum.edu/kb/knowrob.owl#SpatialThing', SourceRef, PartClassName))),

                 create_restr(ClassName, knowrob:properPhysicalParts, PartClassName, owl:someValuesFrom, SourceRef, ObjRestr),

                 ((not(tboxified(Part,_)),
%                    assert(tboxified(Part, PartClassName)),
                   tboxify_object_inst(Part, PartClassName, ReferenceObj, ReferenceObjCl, SourceRef)) ; true ) ), _),


  % iterate over connectedTo objects
  findall(Connected, (rdf_has(ObjInst, P, Connected),
                      rdfs_subproperty_of(P, knowrob:connectedTo)), Connected),
  sort(Connected, ConnectedSorted),

  findall(Conn, (member(Conn, ConnectedSorted),
                ((tboxified(Conn, ConnectedClassName))  -> true ;
                 (rdf_unique_class_id('http://ias.cs.tum.edu/kb/knowrob.owl#SpatialThing', SourceRef, ConnectedClassName))),

                create_restr(ClassName, knowrob:connectedTo, ConnectedClassName, owl:someValuesFrom, SourceRef, ObjRestr),

                ((not(tboxified(Conn,_)),
%                   assert(tboxified(Conn, ConnectedClassName)),
                  tboxify_object_inst(Conn, ConnectedClassName, ReferenceObj, ReferenceObjCl, SourceRef)) ; true ) ), _),
                                                                % use referenceobj here?? -> error with relat

  retractall(tboxified).




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



%% delete_object_information(Object) is det.
%
% Remove object instance and all information stored about it
%
% @param Object Object instance to be deleted
%
delete_object_information(Object) :-

  % remove pose/perception instances
  % removes timepoint, pose, perception itself
  findall(Perception, (rdf_has(Perception, knowrob:objectActedOn, Object),
                       rdf_retractall(Perception, _, _)), _),

  % remove everything directly connected to the object instance
  rdf_retractall(Object, _, _),
  rdf_retractall(_, _, Object).


%% delete_object_information_recursive(Object) is det.
%
% Remove object instance and all information stored about it,
% recursively for the object and all children that can be reached
% by knowrob:parts or knowrob:describedInMap.
%
% TODO: does this also include the room/street/city in maps?
%
% @param Object Object instance to be deleted
%
delete_object_information_recursive(Object) :-

  % remove pose/perception instances
  % removes timepoint, pose, perception itself
  findall(Perc-Mat, (rdf_has(Perc, knowrob:objectActedOn, Object),
                     rdf_has(Perc, knowrob:eventOccursAt, Mat)), Perceptions),

  findall(P-M, (member(P-M, Perceptions),
                rdf_retractall(P, _, _),
                rdf_retractall(M, _, _)), _),

  findall(Child, (rdf_has(Object, knowrob:parts, Child);
                  rdf_has(Child, knowrob:describedInMap, Object)), Children),

  % remove everything directly connected to the object instance
  rdf_retractall(Object, _, _),
  rdf_retractall(_, _, Object),

  ((Children \= []) ->
   ( findall(Child, (member(Child, Children),
                     delete_object_information_recursive(Child)), _)) ;
     true).




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

% TODO: update this in order to use the linked list (go back until end time, then collect until start time)

    % find all detections of the objects at hand
    arg(1, Goal, Arg1),arg(2, Goal, Arg2),
    findall([D_i,Arg1], ( (rdf_has(D_i, knowrob:objectActedOn, Arg1);rdf_has(D_i, knowrob:objectActedOn, Arg2)),
                           rdfs_individual_of(D_i,  knowrob:'MentalEvent')), Detections),

      forall( ( member(D_O, Detections), nth0(0, D_O, Detection),
                rdf_triple(knowrob:startTime, Detection, DStT),
                rdf_triple(knowrob:temporallySubsumes, knowrob:'holds_tt', DStT) ), % MT: change this line to get rid of asserts?
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

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).



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
                              latest_detection_of_instance(Object, D_i),
                              detection_starttime(D_i, St)), Detections),

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
                              detection_starttime(P_i, St)), Perceptions),

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
                          detection_starttime(D_i, St)), Inferences),

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
%<
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



%% comp_orientation(+Object, -Orientation) is nondet.
%
% Wrapper to compute the (deprecated) object orientation property by
% finding the most current object detection. Intended as convenience
% and compatibility predicate.
%
% @param Object      Object identifier
% @param Pose Identifier of the pose matrix
%
comp_orientation(Object, Pose) :-

    latest_detection_of_instance(Object, LatestDetection),
    rdf_triple(knowrob:eventOccursAt, LatestDetection, PoseLocal),

    % transform into global coordinates if relativeTo relation is given
    (( owl_has(PoseLocal, knowrob:relativeTo, RefObj),
       current_object_pose(RefObj, RefObjPose),
       knowrob_objects:rotmat_to_list(PoseLocal, PoseList),
       pose_into_global_coord(PoseList, RefObjPose, PoseGlobal),
       append([['http://ias.cs.tum.edu/kb/knowrob.owl#rotMat3D'],PoseGlobal], PoseListGlobal),
       atomic_list_concat(PoseListGlobal, '_', Pose),! ) ;
    (  Pose = PoseLocal) ),!.




%% temporally_subsumes(+Long, +Short) is nondet.
%
% Verify whether Long temporally subsumes Short
%
% @param Long   The longer time span (e.g. detection of an object)
% @param Short  The shorter time span (e.g. detection of an object)
%
temporally_subsumes(Long, Short) :-

      once(detection_starttime(Short, ShortSt)),
      once(detection_endtime(Short,   ShortEt)),
      
      once(detection_starttime(Long, LongSt)),
      once(detection_endtime(Long,   LongEt)),

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
  term_to_atom(StartTime, StartTAtom),! ;

  rdf_split_url(_, StartTt, Detection),
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




%% comp_xCoord(+Point, ?X) is semidet.
%
% Compute the x-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param X   The x coordinate value
% @see comp_center
    comp_xCoord(Point, X) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m03, Mat, X0),
      ( (X0=literal(type(_,X)));
         X0=X ).

%% comp_yCoord(+Point, ?Y) is semidet.
%
% Compute the y-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param Y   The y coordinate value
% @see comp_center
    comp_yCoord(Point, Y) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m13, Mat, Y0),
      ( (Y0=literal(type(_,Y)));
         Y0=Y ).

%% comp_zCoord(+Point, ?Z) is semidet.
%
% Compute the z-coordinate of the point
%
% @param Obj The point identifier as a String 'translation_<rotation matrix identifier>'
% @param Z   The z coordinate value
% @see comp_center
    comp_zCoord(Point, Z) :-
      rdf_split_url(G, L, Point),
      atom_concat('translation_', RotMat, L),
      rdf_split_url(G, RotMat, Mat),
      rdf_triple(knowrob:m23, Mat, Z0),
      ( (Z0=literal(type(_,Z)));
         Z0=Z ).



%% comp_m00(+Matrix, ?M00) is semidet.
%
% Extract component m(0,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M00 Component m(0,0)
%
    comp_m00(Matrix, M00) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,M00,_,_,_,   _,_,_,_,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m00(Matrix, M00) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,M00,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m01(+Matrix, ?M01) is semidet.
%
% Extract component m(0,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M01 Component m(0,1)
%
    comp_m01(Matrix, M01) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,M01,_,_,   _,_,_,_,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m01(Matrix, M01) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,M01,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m02(+Matrix, ?M02) is semidet.
%
% Extract component m(0,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M02 Component m(0,2)
%
    comp_m02(Matrix, M02) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,M02,_,   _,_,_,_,   _,_,_,_   ,_,_,_,_], '_', O),!.
    comp_m02(Matrix, M02) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,M02,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m03(+Matrix, ?M03) is semidet.
%
% Extract component m(0,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M03 Component m(0,3)
%
    comp_m03(Matrix, M03) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,M03,   _,_,_,_,   _,_,_,_   ,_,_,_,_], '_', O),!.
    comp_m03(Matrix, M03) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,M03,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m04(+Matrix, ?M04) is semidet.
%
% Extract component m(0,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M04 Component m(0,4)
%
    comp_m04(Matrix, M04) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,M04,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m05(+Matrix, ?M05) is semidet.
%
% Extract component m(0,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M05 Component m(0,5)
%
    comp_m05(Matrix, M05) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,M05,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.




%% comp_m10(+Matrix, ?M10) is semidet.
%
% Extract component m(1,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M10 Component m(1,0)
%
    comp_m10(Matrix, M10) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   M10,_,_,_,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m10(Matrix, M10) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   M10,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m11(+Matrix, ?M11) is semidet.
%
% Extract component m(1,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M11 Component m(1,1)
%
    comp_m11(Matrix, M11) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,M11,_,_,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m11(Matrix, M11) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,M11,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m12(+Matrix, ?M12) is semidet.
%
% Extract component m(1,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M12 Component m(1,2)
%
    comp_m12(Matrix, M12) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,M12,_,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m12(Matrix, M12) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,M12,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m13(+Matrix, ?M13) is semidet.
%
% Extract component m(1,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M13 Component m(1,3)
%
    comp_m13(Matrix, M13) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,M13,   _,_,_,_,   _,_,_,_], '_', O),!.
    comp_m13(Matrix, M13) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,M13,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m14(+Matrix, ?M14) is semidet.
%
% Extract component m(1,4) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M14 Component m(1,4)
%
    comp_m14(Matrix, M14) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,M14,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m15(+Matrix, ?M15) is semidet.
%
% Extract component m(1,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M15 Component m(1,5)
%
    comp_m15(Matrix, M15) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,M15,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.




%% comp_m20(+Matrix, ?M20) is semidet.
%
% Extract component m(2,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M20 Component m(2,0)
%
    comp_m20(Matrix, M20) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   M20,_,_,_,   _,_,_,_], '_', O),!.
    comp_m20(Matrix, M20) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   M20,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m21(+Matrix, ?M21) is semidet.
%
% Extract component m(2,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M21 Component m(2,1)
%
    comp_m21(Matrix, M21) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,M21,_,_,   _,_,_,_], '_', O),!.
    comp_m21(Matrix, M21) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,M21,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m22(+Matrix, ?M22) is semidet.
%
% Extract component m(2,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M22 Component m(2,2)
%
    comp_m22(Matrix, M22) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,M22,_,   _,_,_,_], '_', O),!.
    comp_m22(Matrix, M22) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,M22,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m23(+Matrix, ?M23) is semidet.
%
% Extract component m(2,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M23 Component m(2,3)
%
    comp_m23(Matrix, M23) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,_,M23,   _,_,_,_], '_', O),!.
    comp_m23(Matrix, M23) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,M23,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m24(+Matrix, ?M24) is semidet.
%
% Extract component m(2,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M24 Component m(2,4)
%
    comp_m24(Matrix, M24) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,M24,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m25(+Matrix, ?M25) is semidet.
%
% Extract component m(2,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M25 Component m(2,5)
%
    comp_m25(Matrix, M25) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,M25,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.




%% comp_m30(+Matrix, ?M30) is semidet.
%
% Extract component m(3,0) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M30 Component m(3,0)
%
    comp_m30(Matrix, M30) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,_,_,   M30,_,_,_], '_', O),!.
    comp_m30(Matrix, M30) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   M30,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m31(+Matrix, ?M31) is semidet.
%
% Extract component m(3,1) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M31 Component m(3,1)
%
    comp_m31(Matrix, M31) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,_,_,   _,M31,_,_], '_', O),!.
    comp_m31(Matrix, M31) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,M31,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m32(+Matrix, ?M32) is semidet.
%
% Extract component m(3,2) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M32 Component m(3,2)
%
    comp_m32(Matrix, M32) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,_,_,   _,_,M32,_], '_', O),!.
    comp_m32(Matrix, M32) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,M32,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m33(+Matrix, ?M33) is semidet.
%
% Extract component m(3,3) from a matrix
%
% @param Matrix The matrix (4x4 or 6x6, represented as String rotMat3D_M00_M01_M02_M03_M10_M11_M12_M13_M20_M21_M22_M23_M30_M31_M32_M33 for 4x4 and covMat3D_M00_M01... for 6x6)
% @param M33 Component m(3,3)
%
    comp_m33(Matrix, M33) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([rotMat3D,_,_,_,_,   _,_,_,_,   _,_,_,_,   _,_,_,M33], '_', O),!.
    comp_m33(Matrix, M33) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,M33,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m34(+Matrix, ?M34) is semidet.
%
% Extract component m(3,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M34 Component m(3,4)
%
    comp_m34(Matrix, M34) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,M34,_,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m35(+Matrix, ?M35) is semidet.
%
% Extract component m(3,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M35 Component m(3,5)
%
    comp_m35(Matrix, M35) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,M35,   _,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.


%% comp_m40(+Matrix, ?M40) is semidet.
%
% Extract component m(4,0) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M40 Component m(4,0)
%
    comp_m40(Matrix, M40) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   M40,_,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m41(+Matrix, ?M41) is semidet.
%
% Extract component m(4,1) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M41 Component m(4,1)
%
    comp_m41(Matrix, M41) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,M41,_,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m42(+Matrix, ?M42) is semidet.
%
% Extract component m(4,2) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M42 Component m(4,2)
%
    comp_m42(Matrix, M42) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,M42,_,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m43(+Matrix, ?M43) is semidet.
%
% Extract component m(4,3) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M43 Component m(4,3)
%
    comp_m43(Matrix, M43) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,M43,_,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m44(+Matrix, ?M44) is semidet.
%
% Extract component m(4,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M44 Component m(4,4)
%
    comp_m44(Matrix, M44) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,M44,_,   _,_,_,_,_,_], '_', O),!.

%% comp_m45(+Matrix, ?M45) is semidet.
%
% Extract component m(4,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M45 Component m(4,5)
%
    comp_m45(Matrix, M45) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,M45,   _,_,_,_,_,_], '_', O),!.


%% comp_m50(+Matrix, ?M50) is semidet.
%
% Extract component m(5,0) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M50 Component m(5,0)
%
    comp_m50(Matrix, M50) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   M50,_,_,_,_,_], '_', O),!.

%% comp_m51(+Matrix, ?M51) is semidet.
%
% Extract component m(5,1) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M51 Component m(5,1)
%
    comp_m51(Matrix, M51) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,M51,_,_,_,_], '_', O),!.

%% comp_m52(+Matrix, ?M52) is semidet.
%
% Extract component m(5,2) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M52 Component m(5,2)
%
    comp_m52(Matrix, M52) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,M52,_,_,_], '_', O),!.

%% comp_m53(+Matrix, ?M53) is semidet.
%
% Extract component m(5,3) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M53 Component m(5,3)
%
    comp_m53(Matrix, M53) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,M53,_,_], '_', O),!.

%% comp_m54(+Matrix, ?M54) is semidet.
%
% Extract component m(5,4) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M54 Component m(5,4)
%
    comp_m54(Matrix, M54) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,M54,_], '_', O),!.

%% comp_m55(+Matrix, ?M55) is semidet.
%
% Extract component m(5,5) from a matrix
%
% @param Matrix The matrix (6x6, represented as String covMat3D_M00_M01...)
% @param M55 Component m(5,5)
%
    comp_m55(Matrix, M55) :-
        rdf_split_url(_, O, Matrix),
        atomic_list_concat([covMat3D,_,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,_,   _,_,_,_,_,M55], '_', O),!.





