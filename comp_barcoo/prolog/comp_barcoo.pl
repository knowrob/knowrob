/** <module> comp_barcoo

  This module provides routines to interface the ODUfinder cognitive perception
  system, i.e. to read data and interpret the results.

  Copyright (C) 2012 by Nacer Khalil

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

@author Nacer Khalil
@license GPL
*/

:- module(comp_barcoo,
    [
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(comp_barcoo, 'http://www.barcoo.com/barcoo.owl#', [keep(true)]).


%% odufinder_listener(-Listener)
odufinder_listener(Listener) :-
    jpl_new('edu.tum.cs.ias.knowrob.comp_barcoo.BarcooROSclient', ['json_prolog'], Listener),
    jpl_call(Listener, 'startCopObjDetectionsListener', ['/knowrobTopic'], _).

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
  rdf_assert(Obj, knowrob:copID, literal(type(xsd:string, CopIDTerm))).

