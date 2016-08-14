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
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_temporal')).

:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob_owl_test.owl').

:- rdf_db:rdf_register_prefix(knowrob_owl_test, 'http://knowrob.org/kb/knowrob_owl_test.owl#', [keep(true)]).

:- rdf_meta fluent_begin(r,r,t,r),
            fluent_test_assert(r,r,t).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OWL reasoning
%% TODO: write some OWL reasoning tests


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
  entity(knowrob_owl_test:'Refrigerator_fg45543', X),
  X = [an, object, [type, refrigerator]].


%% Poses

test(assert_pose) :-
  entity_assert(knowrob:'Pose_0.0_2.0_0.0_1.0_0.0_0.1_0.2', [a, pose, [0.0,2.0,0.0], [1.0,0.0,0.1,0.2]]),
  rdfs_individual_of(knowrob:'Pose_0.0_2.0_0.0_1.0_0.0_0.1_0.2', knowrob:'Pose').

test(generate_pose_description) :-
  entity(knowrob:'Pose_0.0_2.0_0.0_1.0_0.0_0.1_0.2', X),
  X = [a, pose, [0.0,2.0,0.0], [1.0,0.0,0.1,0.2]].


%% Locations

test(assert_location) :-
  entity_assert(Loc, [a, location]),
  rdfs_individual_of(Loc, knowrob:'SpaceRegion').

test(assert_location_with_property) :-
  % FIXME: [an, object, ...] does not exist, howto handle this?
  entity_assert(Loc, [a, location, [inside_of, [an, object, [type, container]]]]),
  rdfs_individual_of(Loc, knowrob:'SpaceRegion'),
  rdf_has(Loc, knowrob:'insideOf', O),
  rdfs_individual_of(O, knowrob:'Container').

test(generate_location_description) :-
  entity(knowrob:'Location_on-Physical_Refrigerator_fg45543', X),
  X = [a, location|Descr],
  entity_has(Descr, 'on-physical', _).


%% Fluents

test(assert_fluent) :-
  entity_assert(Obj, [an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]]]),
  atom(Obj), rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ).

test(assert_fluent_changing_value) :-
  entity_assert(Obj, [an, object, [type, dough],
        [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]],
        [volume_of_object, 15.0, during, [an, interval, [20.0,30.0]]]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), 25.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), [20.0,30.0]).

test(generate_fluent_description) :-
  Descr=[an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [60.0,70.0]]]],
  entity_assert(Obj, Descr),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  entity(Obj, Descr_),
  Descr_=Descr.

:- end_tests(knowrob_owl).
