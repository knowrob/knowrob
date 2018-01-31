%%
%% Copyright (C) 2016 by Daniel Beßler
%%
%% This file contains tests for the knowrob_temporal module
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

:- begin_tests(knowrob_owl).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/entity')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/temporal')).

:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob_owl_test.owl').

:- rdf_db:rdf_register_prefix(test_owl, 'http://knowrob.org/kb/knowrob_owl_test.owl#', [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OWL entity descriptions

%% Intervals

test(assert_timepoint) :-
  entity_assert(knowrob:'timepoint_20.0', [a, timepoint, 20.0]).

test(assert_timepoint_with_value_key) :-
  entity_assert(knowrob:'timepoint_30.0', [a, timepoint, [value, 30.0]]).

test(assert_timepoint_with_name_key) :-
  entity_assert(knowrob:'timepoint_40.0', [a, timepoint, [name, knowrob:'timepoint_40.0']]).

test(query_timepoint) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, 20.0]).

test(query_timepoint_with_value_key) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, [value, '20.0']]).

test(query_timepoint_with_name_key) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, [name, knowrob:'timepoint_20.0']]).

test(generate_timepoint_description) :-
  entity(knowrob:'timepoint_20.0', X),
  X = [a, timepoint, 20.0].

test(generate_timepoint_description_unexisting, [fail]) :-
  entity(knowrob:'timepoint_4530.0', X), X = [a, timepoint, 4530.0].

test(assert_interval) :-
  entity_assert(knowrob:'TimeInterval_20.0_40.0', [an, interval, [20.0,40.0]]).

test(assert_interval_with_properties) :-
  entity_assert(knowrob:'TimeInterval_20.0_40.0', [an, interval,
      [start_time, [a, timepoint, 20.0]],
      [end_time, [a, timepoint, 40.0]]]).

test(generate_interval_description) :-
  entity(knowrob:'TimeInterval_20.0_40.0', X), X = [an, interval, [20.0,40.0]].


%% Events

test(assert_event) :-
  entity_assert(Evt, [an, event, [type, thinking]]),
  rdfs_individual_of(Evt, knowrob:'Thinking').

test(assert_event_with_property) :-
  entity_assert(Evt, [an, event, [type, thinking], [start_time, [a, timepoint, 20.0]]]),
  rdfs_individual_of(Evt, knowrob:'Thinking'),
  rdf_has(Evt, knowrob:'startTime', knowrob:'timepoint_20.0').


%% Objects

test(assert_object) :-
  entity_assert(Obj, [an, object, [type, dough]]),
  rdfs_individual_of(Obj, knowrob:'Dough').

test(assert_object_with_property) :-
  entity_assert(Obj, [an, object, [type, dough], [volume_of_object, 10.0]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  rdf_has(Obj, knowrob:'volumeOfObject', literal(type(xsd:float,10.0))).

test(generate_refrigerator_description) :-
  entity(test_owl:'Refrigerator_fg45543', X),
  X = [an, object, [type, refrigerator]].

test(query_refrigerator, [nondet]) :-
  entity(Cont, [an, object, [type, refrigerator]]),
  rdf_equal(Cont, test_owl:'Refrigerator_fg45543').

test(query_containerFo1, [nondet,fail]) :-
  entity_assert(Obj, [an, object, [type, storage_construct]]),
  entity(Obj, [an, object,
    [type, storage_construct],
    [type, restriction(
      knowrob:'typePrimaryFunction-containerFor',
      some_values_from(knowrob:'Perishable'))]]).
  
test(query_containerFor2, [nondet]) :-
  entity(Obj, [an, object,
    [type, storage_construct],
    [type, restriction(
      knowrob:'typePrimaryFunction-containerFor',
      some_values_from(knowrob:'Perishable'))]]),
  rdf_equal(Obj, test_owl:'Refrigerator_fg45543').

test(query_primary_storage_place, [nondet]) :-
  entity(Obj, [an, object,
    [type, storage_construct],
    [type, restriction(
        knowrob:'typePrimaryFunction-containerFor',
        some_values_from(knowrob:'Perishable'))]
  ]),
  rdf_equal(Obj, test_owl:'Refrigerator_fg45543').

test(query_refrigerator_by_name, [nondet]) :-
  entity(Cont, [an, object, [name, test_owl:'Refrigerator_fg45543']]),
  rdf_equal(Cont, test_owl:'Refrigerator_fg45543').

test(query_cup_by_nameString_prop, [nondet]) :-
  entity(Cont, [an, object, [name_string, literal(type(_,'cup_name'))]]),
  rdf_equal(Cont, test_owl:'Cup_sfd498th').

test(query_cup_by_nameString_prop_2, [nondet]) :-
  entity(Cont, [an, object, [name_string, X]]),
  X = 'cup_name',
  rdf_equal(Cont, test_owl:'Cup_sfd498th').

test(query_container, [nondet]) :-
  entity(Cont, [an, object, [type, container]]),
  rdf_equal(Cont, test_owl:'Refrigerator_fg45543').

test(query_cup, [nondet]) :-
  entity(Cup, [an, object, [type, cup]]),
  rdf_equal(Cup, test_owl:'Cup_sfd498th').


%% Poses

test(assert_pose) :-
  entity_assert(Pose, [a, pose, [0.0,2.0,0.0], [1.0,0.0,0.1,0.2]]),
  rdf_split_url(_, 'Pose_MapFrame_0.0_2.0_0.0_1.0_0.0_0.1_0.2', Pose),
  rdfs_individual_of(knowrob:'Pose_MapFrame_0.0_2.0_0.0_1.0_0.0_0.1_0.2', knowrob:'Pose').

test(generate_pose_description) :-
  entity(knowrob:'Pose_MapFrame_0.0_2.0_0.0_1.0_0.0_0.1_0.2', X),
  X = [a, pose, [0.0,2.0,0.0], [1.0,0.0,0.1,0.2]].


%% Locations

test(assert_location) :-
  entity_assert(Loc, [a, location]),
  rdfs_individual_of(Loc, knowrob:'SpaceRegion').

test(assert_location_with_property) :-
  entity_assert(Loc, [a, location, [in-cont_generic, [an, object, [type, container]]]]),
  rdfs_individual_of(Loc, knowrob:'SpaceRegion'),
  rdf_has(Loc, knowrob:'in-ContGeneric', O),
  rdfs_individual_of(O, knowrob:'Container').

test(generate_location_description) :-
  entity(knowrob:'Location_on-Physical_Refrigerator_fg45543', X),
  X = [a, location|Descr],
  once(entity_has(Descr, 'on-physical', _)).


%% Fluents

test(assert_temporal_part) :-
  entity_assert(Obj, [an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]]]),
  atom(Obj), rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ).

test(assert_temporal_part_changing_value) :-
  entity_assert(Obj, [an, object, [type, dough],
        [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]],
        [volume_of_object, 15.0, during, [an, interval, [20.0,30.0]]]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), 25.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), [20.0,30.0]).

test(generate_temporal_part_description) :-
  Descr=[an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [60.0,70.0]]]],
  entity_assert(Obj, Descr),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  entity(Obj, Descr_),
  Descr_=Descr.

%% TBOX
test(assert_class_and_properties) :-
  rdf_assert(knowrob:'MarsianTime', rdf:type, owl:'Class'),
  rdf_assert(knowrob:'MarsianTime', rdfs:subClassOf, knowrob:'TimePoint'),
  rdf_assert(knowrob:'MarsLocation', rdf:type, owl:'Class'),
  owl_restriction_assert(restriction('http://knowrob.org/kb/knowrob.owl#locationOf', some_values_from(knowrob:'MarsLocation')), Rstr1),
  rdf_assert(knowrob:'MarsianTime', rdfs:subClassOf, Rstr1),
  findall( Val, owl_class_properties(knowrob:'MarsianTime', _Prop, Val), ValuesA),
  length(ValuesA, 1),
  rdf_assert(knowrob:'MarsianCoreTime', rdf:type, owl:'Class'),
  rdf_assert(knowrob:'MarsianCoreTime', rdfs:subClassOf, knowrob:'MarsianTime'),
  rdf_assert(knowrob:'MarsOuterSpace', rdf:type, owl:'Class'),
  owl_restriction_assert(restriction('http://knowrob.org/kb/knowrob.owl#toLocation', all_values_from(knowrob:'MarsOuterSpace')), Rstr2),
  rdf_assert(knowrob:'MarsianCoreTime', rdfs:subClassOf, Rstr2),
  findall( Val, owl_class_properties(knowrob:'MarsianCoreTime', _Prop, Val), ValuesB),
  length(ValuesB, 1),
  findall( Val,owl_class_properties_some(knowrob:'MarsianCoreTime', _Prop, Val), ValuesC),
  length(ValuesC, 1),
  findall( Val, owl_class_properties_all(knowrob:'MarsianCoreTime', _Prop, Val), ValuesD),
  length(ValuesD, 1),
  findall( Val,owl_class_properties_value(knowrob:'MarsianCoreTime', _Prop, Val), ValuesE),
  length(ValuesE, 0),
  findall( Val,owl_class_properties_nosup(knowrob:'MarsianCoreTime', _Prop, Val), ValuesF),
  length(ValuesF, 0).


  
:- end_tests(knowrob_owl).
