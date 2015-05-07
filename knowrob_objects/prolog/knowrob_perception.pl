/** <module> Methods for creating object instances based on perceptual information

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

:- module(knowrob_perception,
    [
      create_object_perception/4,
      create_pose/2,
      create_perception_instance/2,
      create_perception_instance/3,
      set_object_perception/2,
      set_perception_pose/2,
      create_pose/2,
      set_perception_cov/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


:-  rdf_meta
    create_object_perception(r,+,+,+,-).


%% create_object_perception(+ObjClass, +ObjPose, +PerceptionTypes, -ObjInst)
%
% Create the complete structure of an object perception, including the object
% instance, the perception instance, and the pose matrix where the object was
% perceived.
%
% @param ObjClass          Type of the perceived object
% @param ObjPose           Pose of the perceived object as row-based list containing the 4x4 rotation matrix
% @param PerceptionTypes   List of perception types, e.g. 'VisualPerception' (only the class names, no namespaces)
% @param ObjInst           Object instance that has been created by this predicate
% 
create_object_perception(ObjClass, ObjPose, PerceptionTypes, ObjInst) :-
    rdf_instance_from_class(ObjClass, ObjInst),
    create_perception_instance(PerceptionTypes, Perception),
    set_object_perception(ObjInst, Perception),
    set_perception_pose(Perception, ObjPose).



%% create_perception_instance(PerceptionTypes, Perception) is det.
%% create_perception_instance(PerceptionTypes, ModelTypes, Perception) is det.
%
% Create perception instance having all the types in PerceptionTypes, and set
% the types of recognition models that have been used for perceiving the object,
% if given.
%
% @param PerceptionTypes   List of perception types, e.g. 'VisualPerception' (only the class names, no namespaces)
% @param PerceptionInst    Perception instance that has been created by this predicate
% 
create_perception_instance(PerceptionTypes, Perception) :-

  % create individual from first type in the list
  nth0(0, PerceptionTypes, PType),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', PType, PClass),
  rdf_instance_from_class(PClass, Perception),

  % set all other types
  findall(PC, (member(PT, PerceptionTypes),
               atom_concat('http://knowrob.org/kb/knowrob.owl#', PT, PC),
               rdf_assert(Perception, rdf:type, PC)), _),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).


create_perception_instance(PerceptionTypes, ModelTypes, Perception) :-

  % create individual from first type in the list
  nth0(0, PerceptionTypes, PType),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', PType, PClass),
  rdf_instance_from_class(PClass, Perception),

  % set all other types
  findall(PC, (member(PT, PerceptionTypes),
               atom_concat('http://knowrob.org/kb/knowrob.owl#', PT, PC),
               rdf_assert(Perception, rdf:type, PC)), _),

  % set the perceivedUsingModel relation
  findall(MC, (member(MT, ModelTypes),
               atom_concat('http://knowrob.org/kb/knowrob.owl#', MT, MC),
               rdf_assert(Perception, knowrob:perceivedUsingModel, MC)), _),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).




%% set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
% @param Object        Object instance
% @param Perception    Perception instance
% 
set_object_perception(Object, Perception) :-

  % add perception to linked list of object detections,
  ((rdf_has(Object, knowrob:latestDetectionOfObject, Prev)) -> (

    rdf_update(Object, knowrob:latestDetectionOfObject, Prev, object(Perception)),
    rdf_assert(Perception, knowrob:previousDetectionOfObject, Prev)

  ) ; (
    rdf_assert(Object, knowrob:latestDetectionOfObject, Perception)
  )),

  % update latestDetectionOfObject pointer to list head
  rdf_assert(Perception, knowrob:objectActedOn, Object).


%% set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception to the value given as PoseList
%
% @param Perception  Perception instance
% @param PoseList    Pose of the perceived object as row-based list containing the 4x4 rotation matrix
% 
set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], PoseInst),
  rdf_assert(Perception, knowrob:eventOccursAt, PoseInst).

%% create_pose(PoseList, PoseInst) is det.
%
% Set the pose of an object perception to the value given as PoseList
%
% @param Perception  Instance of a RotationMatrix3D with all elements asserted as datatype properties
% @param PoseList    Pose as row-based list containing the 4x4 rotation matrix
% 
create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], PoseInst) :-

  rdf_instance_from_class(knowrob:'RotationMatrix3D', PoseInst),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m00',literal(type(xsd:float, M00))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m01',literal(type(xsd:float, M01))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(xsd:float, M02))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(xsd:float, M03))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m10',literal(type(xsd:float, M10))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m11',literal(type(xsd:float, M11))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(xsd:float, M12))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(xsd:float, M13))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m20',literal(type(xsd:float, M20))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m21',literal(type(xsd:float, M21))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(xsd:float, M22))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(xsd:float, M23))),

  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m30',literal(type(xsd:float, M30))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m31',literal(type(xsd:float, M31))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(xsd:float, M32))),
  rdf_assert(PoseInst,'http://knowrob.org/kb/knowrob.owl#m33',literal(type(xsd:float, M33))).




%% set_perception_cov(+Perception, +CovList) is det.
%
% Set the covariance of an object perception
%
% @param Perception  Instance of a CovarianceMatrix with all elements asserted as datatype properties
% @param PoseList    Row-based list containing the 6x6 covariance matrix
% 
set_perception_cov(Perception, [M00, M01, M02, M03, M04, M05, M10, M11, M12, M13, M14, M15, M20, M21, M22, M23, M24, M25, M30, M31, M32, M33, M34, M35, M40, M41, M42, M43, M44, M45, M50, M51, M52, M53, M54, M55]) :-

  rdf_instance_from_class(knowrob:'CovarianceMatrix', Cov),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m00',literal(type(xsd:float, M00))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m01',literal(type(xsd:float, M01))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m02',literal(type(xsd:float, M02))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(xsd:float, M03))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(xsd:float, M04))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m03',literal(type(xsd:float, M05))),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m10',literal(type(xsd:float, M10))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m11',literal(type(xsd:float, M11))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m12',literal(type(xsd:float, M12))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(xsd:float, M13))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(xsd:float, M14))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m13',literal(type(xsd:float, M15))),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m20',literal(type(xsd:float, M20))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m21',literal(type(xsd:float, M21))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m22',literal(type(xsd:float, M22))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(xsd:float, M23))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(xsd:float, M24))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m23',literal(type(xsd:float, M25))),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m30',literal(type(xsd:float, M30))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m31',literal(type(xsd:float, M31))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m32',literal(type(xsd:float, M32))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m33',literal(type(xsd:float, M33))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m33',literal(type(xsd:float, M34))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m33',literal(type(xsd:float, M35))),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m40',literal(type(xsd:float, M40))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m41',literal(type(xsd:float, M41))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m42',literal(type(xsd:float, M42))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m43',literal(type(xsd:float, M43))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m43',literal(type(xsd:float, M44))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m43',literal(type(xsd:float, M45))),

  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m50',literal(type(xsd:float, M50))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m51',literal(type(xsd:float, M51))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m52',literal(type(xsd:float, M52))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m53',literal(type(xsd:float, M53))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m53',literal(type(xsd:float, M54))),
  rdf_assert(Cov,'http://knowrob.org/kb/knowrob.owl#m53',literal(type(xsd:float, M55))),

  rdf_assert(Perception, knowrob:covariance, Cov).

