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
      create_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).



%% create_object_perception(ObjClass, ObjPose, PerceptionTypes, ObjInst)
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
  to_knowrob(TypeAtom, TypeAtomKnowrob),

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
  rdf_assert(Perception, knowrob:objectActedOn, Object).



%% set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], Loc),
  rdf_assert(Perception, knowrob:eventOccursAt, Loc).


create_pose([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33], Loc) :-

  % set the pose
  atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
  rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D').




%% set_perception_cov(+Perception, +CovList) is det.
%
% Set the covariance of an object perception
%
set_perception_cov(Perception, [M00, M01, M02, M03, M04, M05, M10, M11, M12, M13, M14, M15, M20, M21, M22, M23, M24, M25, M30, M31, M32, M33, M34, M35, M40, M41, M42, M43, M44, M45, M50, M51, M52, M53, M54, M55]) :-

  % set the pose
  atomic_list_concat(['covMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M04,'_',M05,'_',M10,'_',M11,'_',M12,'_',M13,'_',M14,'_',M15,'_',M20,'_',M21,'_',M22,'_',M23,'_',M24,'_',M25,'_',M30,'_',M31,'_',M32,'_',M33,'_',M34,'_',M35,'_',M40,'_',M41,'_',M42,'_',M43,'_',M44,'_',M45,'_',M50,'_',M51,'_',M52,'_',M53,'_',M54,'_',M55], CovIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', CovIdentifier, Cov),
  rdf_assert(Cov, rdf:type, knowrob:'CovarianceMatrix'),
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


