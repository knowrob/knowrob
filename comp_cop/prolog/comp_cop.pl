/** <module> comp_cop

  This module provides routines to interface the CoP cognitive perception
  system, i.e. to read data from CoP, send queries, and interpret the results.

  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(comp_cop,
    [
      cop_compatible_results/2,
      cop_to_knowrob/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_cop, 'http://ias.cs.tum.edu/kb/comp_cop.owl#', [keep(true)]).

:- rdf_meta cop_to_knowrob(r,r).

%% cop_listener(-Listener)
cop_listener(Listener) :-
    jpl_new('edu.tum.cs.ias.knowrob.CopROSClient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startCopModelDBListener', ['/knowrob/cop_db', '/cop/in'], _),
    jpl_call(Listener, 'startCopObjDetectionsListener', ['/kipla/cop_reply'], _).

%% odufinder_listener(-Listener)
odufinder_listener(Listener) :-
    jpl_new('edu.tum.cs.ias.knowrob.CopROSClient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startCopObjDetectionsListener', ['/odu_finder/TemplateName'], _).

%% missing_obj_listener(-Listener)
missing_obj_listener(Listener) :-
    jpl_new('edu.tum.cs.ias.knowrob.CopROSClient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startCopObjDetectionsListener', ['/synthetic_percepts/tabletop_percepts'], _).


%% cop_create_model_instance(+ModelType, +ObjectType) is det.
%
% Create instance of a CopPerception model that provides recognition
% services for the ObjectType
%
cop_create_model_instance(ModelType, ObjectType) :-

  atom_concat('http://ias.cs.tum.edu/kb/comp_cop.owl#', ModelType, ModelT),
  rdf_instance_from_class(ModelT, ModelInst),

  rdf_assert(ModelInst, knowrob:providesModelFor, ObjectType).




%% cop_create_perception_instance(+ModelTypes, -Perception) is det.
%
% Create perception instance having all the types in ModelTypes
%
cop_create_perception_instance(ModelTypes, Perception) :-

  rdf_instance_from_class('http://ias.cs.tum.edu/kb/comp_cop.owl#CopPerception', Perception),

  findall(MC, (member(MT, ModelTypes),
               atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', MT, MC),
               rdf_assert(Perception, knowrob:perceivedUsingModel, MC)), _),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).



%% cop_create_object_instance(+ObjTypes, +CopID, -Obj) is det.
%
% Create object instance having all the types in ObjTypes
%
cop_create_object_instance(ObjTypes, CopID, Obj) :-

  member(ObjType, ObjTypes),
  string_to_atom(ObjType, TypeAtom),
%   atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocalTypeAtom, TypeAtom),
  atom_concat(TypeAtom, CopID, Obj),

  (rdf_has(Obj, rdf:type, TypeAtom),!;
  rdf_assert(Obj, rdf:type, TypeAtom)),

  string_to_atom(CopID, CopIDAtom),
  term_to_atom(CopIDTerm, CopIDAtom),
  rdf_assert(Obj, knowrob:copID, literal(type(xsd:int, CopIDTerm))).



%% cop_set_loid(+Perception, +LoID) is det.
%
% Set the lo ID of the perception instance
%
cop_set_loid(Perception, LoID) :-
  string_to_atom(LoID, LoIDAtom),
  term_to_atom(LoIDTerm, LoIDAtom),
  rdf_assert(Perception, knowrob:loID, literal(type(xsd:int, LoIDTerm))).



%% cop_set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
cop_set_object_perception(Object, Perception) :-
  rdf_assert(Perception, knowrob:objectActedOn, Object).



%% cop_set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
cop_set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  % set the pose
  atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
  rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
  rdf_assert(Perception, knowrob:eventOccursAt, Loc).



%% cop_set_perception_cov(+Perception, +CovList) is det.
%
% Set the pose of an object perception
%
cop_set_perception_cov(Perception, [M00, M01, M02, M03, M04, M05, M10, M11, M12, M13, M14, M15, M20, M21, M22, M23, M24, M25, M30, M31, M32, M33, M34, M35, M40, M41, M42, M43, M44, M45, M50, M51, M52, M53, M54, M55]) :-

  % set the pose
  atomic_list_concat(['covMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M04,'_',M05,'_',M10,'_',M11,'_',M12,'_',M13,'_',M14,'_',M15,'_',M20,'_',M21,'_',M22,'_',M23,'_',M24,'_',M25,'_',M30,'_',M31,'_',M32,'_',M33,'_',M34,'_',M35,'_',M40,'_',M41,'_',M42,'_',M43,'_',M44,'_',M45,'_',M50,'_',M51,'_',M52,'_',M53,'_',M54,'_',M55], CovIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', CovIdentifier, Cov),
  rdf_assert(Cov, rdf:type, knowrob:'CovarianceMatrix'),
  rdf_assert(Perception, knowrob:covariance, Cov).




%% cop_compatible_results(?A, ?B) is det.
%
% Check if the CoP results A and B are compatible.
%
% This predicate maps the returned classes against KnowRob classes and determines
% if these classes are disjoint (i.e. the results are incompatible)
%
% Example: cop_compatible_results('green', 'Knife')  are compatible results
%          cop_compatible_results('green', 'orange') are incompatible
%
% @param CopIdentifier     Atom identifying something in CoP
% @param KnowrobIdentifier Corresponding atom identifying something in KnowRob
%
cop_compatible_results(A, B) :-
  ((nonvar(A))->(downcase_atom(A, Alower));(Alower=A)),
  ((nonvar(B))->(downcase_atom(B, Blower));(Blower=B)),

  cop_to_knowrob(Alower, Akr),
  cop_to_knowrob(Blower, Bkr),
  not(owl_disjoint_with(Akr, Bkr)).


%% cop_to_knowrob(?CopIdentifier, ?KnowrobIdentifier) is det.
%
% This predicate defines a simple mapping to translate from any kind of identifier in CoP
% (class names, model names, etc) into the corresponding identifiers in KnowRob.
%
% @param CopIdentifier     Atom identifying something in CoP
% @param KnowrobIdentifier Corresponding atom identifying something in KnowRob
%
cop_to_knowrob('placemat', K)    :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#PlaceMat',!.
cop_to_knowrob('mug', K)         :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingMug',!.
cop_to_knowrob('plate', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#DinnerPlate',!.

cop_to_knowrob('knife', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#TableKnife',!.
cop_to_knowrob('forknoteeth', K) :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Fork-SilverwarePiece',!.
cop_to_knowrob('forkwteeth', K)  :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Fork-SilverwarePiece',!.
cop_to_knowrob('spoon', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Spoon-SilverwarePiece',!.

cop_to_knowrob('icetea', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Tea-Iced',!.
cop_to_knowrob('iceteafront', K) :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Tea-Iced',!.
cop_to_knowrob('bottle', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle',!.
cop_to_knowrob('assamblend', K)  :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Tea-Beverage',!.

cop_to_knowrob('caltab', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#CalibrationPlate',!.
cop_to_knowrob('marker33', K)    :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#CalibrationPlate',!.

cop_to_knowrob('handle', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Handle',!.
cop_to_knowrob('transobject', K) :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#ColorlessThing',!.
cop_to_knowrob('cluster', K)     :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#PointCloud',!.

cop_to_knowrob('face', K)        :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#FaceOfAnimal',!.
cop_to_knowrob('wallobj', K)     :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#WallOfAConstruction',!.

cop_to_knowrob('black', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#BlackColor',!.
cop_to_knowrob('white', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#WhiteColor',!.
cop_to_knowrob('orange', K)      :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#OrangeColor',!.
cop_to_knowrob('red', K)         :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#RedColor',!.
cop_to_knowrob('green', K)       :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#GreenColor',!.
cop_to_knowrob('blue', K)        :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#BlueColor',!.

cop_to_knowrob('transobject', K)        :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#ColorlessThing',!.
cop_to_knowrob('defaulttableobject', K) :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#PointCloud',!.

cop_to_knowrob('bowl-eating', K)        :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#Bowl-Eating',!.
cop_to_knowrob('cowsmilk-product', K)   :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#CowsMilk-Product',!.
cop_to_knowrob('breakfast-cereal', K)   :- K= 'http://ias.cs.tum.edu/kb/knowrob.owl#BreakfastCereal',!.





% specially for germandeli: check if there is a class with the respective productID
cop_to_knowrob(GermanDeliID, GermanDeliOWLClass) :-
    owl_has(Restr, owl:hasValue, literal(type(xsd:string, GermanDeliID))),
    owl_has(Restr, owl:onProperty, 'http://ias.cs.tum.edu/kb/germandeli.owl#productID'),
    owl_direct_subclass_of(GermanDeliOWLClass, Restr),
    owl_subclass_of(GermanDeliOWLClass, 'http://ias.cs.tum.edu/kb/germandeli.owl#GermanDeliObject'),!.

% Fallback: directly use the class idenfier
cop_to_knowrob(GermanDeliID, KnowrobIdentifier):-
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', GermanDeliID, KnowrobIdentifier).
