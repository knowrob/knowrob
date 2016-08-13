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

:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob_temporal.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob_temporal_test.owl').

:- rdf_db:rdf_register_prefix(test, 'http://knowrob.org/kb/knowrob_temporal_test.owl#', [keep(true)]).

:- rdf_meta fluent_begin(r,r,t,r),
            fluent_test_assert(r,r,t).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OWL reasoning

test(class_properties) :-
  fail. % TODO: write test with object property


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OWL entity descriptions

test(entity_assert_timepoint1) :-
  entity_assert(knowrob:'timepoint_20.0', [a, timepoint, 20.0]).
test(entity_assert_timepoint2) :-
  entity_assert(knowrob:'timepoint_30.0', [a, timepoint, [value, 30.0]]).
test(entity_assert_timepoint3) :-
  entity_assert(knowrob:'timepoint_40.0', [a, timepoint, [name, knowrob:'timepoint_40.0']]).

test(entity_timepoint1) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, 20.0]).
test(entity_timepoint2) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, [value, '20.0']]).
test(entity_timepoint3) :-
  entity(knowrob:'timepoint_20.0', [a, timepoint, [name, knowrob:'timepoint_20.0']]).
test(entity_timepoint4) :-
  entity(knowrob:'timepoint_20.0', X),
  X = [a, timepoint, 20.0].

test(entity_assert_interval1) :-
  entity_assert(knowrob:'TimeInterval_20.0_40.0', [an, interval, [20.0,40.0]]).
test(entity_assert_interval2) :-
  entity_assert(knowrob:'TimeInterval_20.0_40.0', [an, interval,
      [start_time, [a, timepoint, 20.0]],
      [end_time, [a, timepoint, 40.0]]]).
test(entity_assert_interval3) :-
  entity(knowrob:'TimeInterval_20.0_40.0', X), X = [an, interval, [20.0,40.0]].

test(entity_assert_event1) :-
  entity_assert(Evt, [an, event, [type, thinking]]),
  rdfs_individual_of(Evt, knowrob:'Thinking').
test(entity_assert_event2) :-
  entity_assert(Evt, [an, event, [type, thinking], [start_time, [a, timepoint, 20.0]]]),
  rdfs_individual_of(Evt, knowrob:'Thinking'),
  rdf_has(Evt, knowrob:'startTime', knowrob:'timepoint_20.0').

test(entity_assert_object1) :-
  entity_assert(Obj, [an, object, [type, dough]]),
  rdfs_individual_of(Obj, knowrob:'Dough').
test(entity_assert_obj_property0) :-
  entity_assert(Obj, [an, object, [type, dough], [volume_of_object, 10.0]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  rdf_has(Obj, knowrob:'volumeOfObject', literal(type(xsd:float,10.0))).

test(entity_fluent0) :-
  entity_assert(Obj, [an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ).
test(entity_fluent1) :-
  entity_assert(Obj, [an, object, [type, dough],
        [volume_of_object, 10.0, during, [an, interval, [0.0,20.0]]],
        [volume_of_object, 15.0, during, [an, interval, [20.0,30.0]]]]),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), 5.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,20.0]),
  not( holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,10.0))), [0.0,21.0]) ),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), 25.0),
  holds(knowrob:'volumeOfObject'(Obj, literal(type(xsd:float,15.0))), [20.0,30.0]).
test(entity_gen_fluent0) :-
  Descr=[an, object, [type, dough], [volume_of_object, 10.0, during, [an, interval, [60.0,70.0]]]],
  entity_assert(Obj, Descr),
  rdfs_individual_of(Obj, knowrob:'Dough'),
  entity(Obj, Descr_),
  Descr_=Descr.

:- end_tests(knowrob_owl).
