%%
%% Copyright (C) 2015 by Alexey Reshetnyak
%%
%% This file contains tests for the knowrob_objects module
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

:- begin_tests(knowrob_objects).

:- use_module(library(semweb/rdf_db)).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob/perception')).

:- owl_parser:owl_parse('package://knowrob_objects/owl/knowrob_objects.owl').
:- owl_parser:owl_parse('package://knowrob_objects/owl/test_knowrob_objects.owl').

:- rdf_db:rdf_register_prefix(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_prefix(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test_knowrob_objects, 'http://knowrob.org/kb/test_knowrob_objects.owl#', [keep(true)]).

test(storagePlaceFor1) :-
  storagePlaceFor(test_knowrob_objects:'Dishwasher1', test_knowrob_objects:'Cup1'), !.

test(storagePlaceFor2) :-
  storagePlaceFor(test_knowrob_objects:'Dishwasher1', test_knowrob_objects:'Cup2'), !.

test(storagePlaceFor3) :-
  storagePlaceFor(test_knowrob_objects:'Dishwasher1', knowrob:'FoodVessel'), !.

test(storagePlaceForBecause1) :-
  storagePlaceForBecause(test_knowrob_objects:'Dishwasher1', test_knowrob_objects:'Cup1',X),
  owl_description(X,union_of([class('http://knowrob.org/kb/knowrob.owl#FoodUtensil'),
                              class('http://knowrob.org/kb/knowrob.owl#FoodVessel')])), !.

test(current_object_pose) :-
  current_object_pose(test_knowrob_objects:'Cup1', ['map','Cup1',
      [1.1277804,2.6304414,0.816479],
      [0.0,0.0,0.04428184028938965,0.9990190820981245]]).

test(object_pose_at_time) :-
  object_pose_at_time(test_knowrob_objects:'Cup1', knowrob:'timepoint_1331040458', ['map','Cup1',
      [1.1277804,2.6304414,0.816479],
      [0.0,0.0,0.04428184028938965,0.9990190820981245]]).

test(object_dimensions) :-
  object_dimensions(test_knowrob_objects:'Handle1', 0.015, 0.015, 0.015),
  !.

test(object_assert_color) :-
  object_assert_color(test_knowrob_objects:'Cup1', '0.3 0.5 0.6 1'),
  object_color(test_knowrob_objects:'Cup1', [0.3, 0.5, 0.6, 1]).

test(object_color) :-
  object_color(test_knowrob_objects:'Cup1', [0.3, 0.5, 0.6, 1]).

test(object_assert_dimensions) :-
  object_assert_dimensions(test_knowrob_objects:'Cup2', 0.032, 0.032, 0.12),
  object_dimensions(test_knowrob_objects:'Cup2', 0.032, 0.032, 0.12),
  !.

%test(object_detection, fixme) :-
%  object_detection(test_knowrob_objects:'Cup1', test_knowrob_objects:'timepoint_1331040458', test_knowrob_objects:'SemanticMapPerception2').

:- end_tests(knowrob_objects).

