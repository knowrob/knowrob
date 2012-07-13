%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides methods for creating object instances in KnowRob
%% based on perceptual information.
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


:- module(knowrob_perception,
    [
      create_object_perception/4,
      create_pose/2,
      create_perception_instance/2,
      create_perception_instance/3,
      create_object_instance/3,
      set_object_perception/2,
      set_perception_pose/2,
      create_pose/2,
      set_perception_cov/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).


:-  rdf_meta
    create_object_perception(r,+,+,+,-).

%% create_object_perception(+ObjClass, +ObjPose, +PerceptionTypes, -ObjInst)
%
% Convenience predicate: create the complete structure of object instance,
% perception instance, and the pose matrix where the object was perceived.
%
create_object_perception(ObjClass, ObjPose, PerceptionTypes, ObjInst) :-
    rdf_instance_from_class(ObjClass, ObjInst),
    create_perception_instance(PerceptionTypes, Perception),
    set_object_perception(ObjInst, Perception),
    set_perception_pose(Perception, ObjPose).



%% create_perception_instance(-Perception) is det.
%% create_perception_instance(+ModelTypes, -Perception) is det.
%
% Create perception instance having all the types in PerceptionTypes and set
% model types if given
%

create_perception_instance(PerceptionTypes, Perception) :-

  % create individual from first type in the list
  nth0(0, PerceptionTypes, PType),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PType, PClass),
  rdf_instance_from_class(PClass, Perception),

  % set all other types
  findall(PC, (member(PT, PerceptionTypes),
               atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PT, PC),
               rdf_assert(Perception, rdf:type, PC)), _),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).


create_perception_instance(PerceptionTypes, ModelTypes, Perception) :-

  % create individual from first type in the list
  nth0(0, PerceptionTypes, PType),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PType, PClass),
  rdf_instance_from_class(PClass, Perception),

  % set all other types
  findall(PC, (member(PT, PerceptionTypes),
               atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', PT, PC),
               rdf_assert(Perception, rdf:type, PC)), _),

  % set the perceivedUsingModel relation
  findall(MC, (member(MT, ModelTypes),
               atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', MT, MC),
               rdf_assert(Perception, knowrob:perceivedUsingModel, MC)), _),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).






%% create_object_instance(+ObjTypes, +ObjID, -Obj) is det.
%
% Create object instance having all the types in ObjTypes
%
create_object_instance(ObjTypes, ObjID, Obj) :-

  % map types to object classes
  member(ObjType, ObjTypes),
  string_to_atom(ObjType, TypeAtom),
%   to_knowrob(TypeAtom, TypeAtomKnowrob),
  TypeAtomKnowrob = TypeAtom,

  %TODO: replace this with real entity resolution
  % here, we just update the first instance of
  % the respective type or create a new one if it
  % does not exist
  (
    (
     (rdfs_individual_of(Obj, TypeAtomKnowrob),!);
     (atom_concat(TypeAtomKnowrob, ObjID, Obj))
    ),
    rdf_assert(Obj, rdf:type, TypeAtomKnowrob)
  ).


%% set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
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
% Set the pose of an object perception
%
set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], Loc),
  rdf_assert(Perception, knowrob:eventOccursAt, Loc).


create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], Loc) :-

  rdf_instance_from_class(knowrob:'RotationMatrix3D', Loc),

  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m00',literal(type(xsd:float, M00))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m01',literal(type(xsd:float, M01))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m02',literal(type(xsd:float, M02))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m03',literal(type(xsd:float, M03))),

  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m10',literal(type(xsd:float, M10))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m11',literal(type(xsd:float, M11))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m12',literal(type(xsd:float, M12))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m13',literal(type(xsd:float, M13))),

  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m20',literal(type(xsd:float, M20))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m21',literal(type(xsd:float, M21))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m22',literal(type(xsd:float, M22))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m23',literal(type(xsd:float, M23))),

  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m30',literal(type(xsd:float, M30))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m31',literal(type(xsd:float, M31))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m32',literal(type(xsd:float, M32))),
  rdf_assert(Loc,'http://ias.cs.tum.edu/kb/knowrob.owl#m33',literal(type(xsd:float, M33))).




%% set_perception_cov(+Perception, +CovList) is det.
%
% Set the covariance of an object perception
%
set_perception_cov(Perception, [M00, M01, M02, M03, M04, M05, M10, M11, M12, M13, M14, M15, M20, M21, M22, M23, M24, M25, M30, M31, M32, M33, M34, M35, M40, M41, M42, M43, M44, M45, M50, M51, M52, M53, M54, M55]) :-


  rdf_instance_from_class(knowrob:'CovarianceMatrix', Cov),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m00',literal(type(xsd:float, M00))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m01',literal(type(xsd:float, M01))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m02',literal(type(xsd:float, M02))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m03',literal(type(xsd:float, M03))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m03',literal(type(xsd:float, M04))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m03',literal(type(xsd:float, M05))),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m10',literal(type(xsd:float, M10))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m11',literal(type(xsd:float, M11))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m12',literal(type(xsd:float, M12))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m13',literal(type(xsd:float, M13))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m13',literal(type(xsd:float, M14))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m13',literal(type(xsd:float, M15))),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m20',literal(type(xsd:float, M20))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m21',literal(type(xsd:float, M21))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m22',literal(type(xsd:float, M22))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m23',literal(type(xsd:float, M23))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m23',literal(type(xsd:float, M24))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m23',literal(type(xsd:float, M25))),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m30',literal(type(xsd:float, M30))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m31',literal(type(xsd:float, M31))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m32',literal(type(xsd:float, M32))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m33',literal(type(xsd:float, M33))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m33',literal(type(xsd:float, M34))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m33',literal(type(xsd:float, M35))),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m40',literal(type(xsd:float, M40))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m41',literal(type(xsd:float, M41))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m42',literal(type(xsd:float, M42))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m43',literal(type(xsd:float, M43))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m43',literal(type(xsd:float, M44))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m43',literal(type(xsd:float, M45))),

  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m50',literal(type(xsd:float, M50))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m51',literal(type(xsd:float, M51))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m52',literal(type(xsd:float, M52))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m53',literal(type(xsd:float, M53))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m53',literal(type(xsd:float, M54))),
  rdf_assert(Cov,'http://ias.cs.tum.edu/kb/knowrob.owl#m53',literal(type(xsd:float, M55))),

  rdf_assert(Perception, knowrob:covariance, Cov).




%% compatible_obj_types(?A, ?B) is det.
%
% Check if the CoP results A and B are compatible.
%
% This predicate maps the returned classes against KnowRob classes and determines
% if these classes are disjoint (i.e. the results are incompatible)
%
% Example: compatible_obj_types('green', 'Knife')  are compatible results
%          compatible_obj_types('green', 'orange') are incompatible
%
% @param CopIdentifier     Atom identifying something in CoP
% @param KnowrobIdentifier Corresponding atom identifying something in KnowRob
%
compatible_obj_types(A, B) :-
  ((nonvar(A))->(downcase_atom(A, Alower));(Alower=A)),
  ((nonvar(B))->(downcase_atom(B, Blower));(Blower=B)),

  cop_to_knowrob(Alower, Akr),
  cop_to_knowrob(Blower, Bkr),
  not(owl_disjoint_with(Akr, Bkr)).


