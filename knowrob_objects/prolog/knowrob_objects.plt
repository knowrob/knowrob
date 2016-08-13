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
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_objects')).

:- owl_parser:owl_parse('package://knowrob_objects/owl/knowrob_objects.owl').
:- owl_parser:owl_parse('package://knowrob_objects/owl/test_knowrob_objects.owl').

:- rdf_db:rdf_register_prefix(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_prefix(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_prefix(test, 'http://knowrob.org/kb/test_knowrob_objects.owl#', [keep(true)]).

test(storagePlaceFor1) :-
  storagePlaceFor(test:'Dishwasher1', test:'Cup1'), !.

test(storagePlaceFor2) :-
  storagePlaceFor(test:'Dishwasher1', test:'Cup2'), !.

test(storagePlaceFor3) :-
  storagePlaceFor(test:'Dishwasher1', knowrob:'FoodVessel'), !.

test(storagePlaceForBecause1) :-
  storagePlaceForBecause(test:'Dishwasher1', test:'Cup1', knowrob:'FoodVessel'), !.

test(storagePlaceForBecause2) :-
  storagePlaceForBecause(test:'Dishwasher1', test:'Cup1', knowrob:'FoodVessel'), !.

test(current_object_pose) :-
  current_object_pose(test:'Cup1',
                      [0.99607825, -0.08847681, 0.0, 1.1277804,
                       0.08847681,  0.99607825, 0.0, 2.6304414,
                       0.0,         0.0,        1.0, 0.816479,
                       0.0,         0.0,        0.0, 1.0]).

test(object_pose_at_time) :-
  object_pose_at_time(test:'Cup1', test:'timepoint_1331040458',
                      [0.99607825, -0.08847681, 0.0, 1.1277804,
                       0.08847681,  0.99607825, 0.0, 2.6304414,
                       0.0,         0.0,        1.0, 0.816479,
                       0.0,         0.0,        0.0, 1.0]).

test(rotmat_to_list) :-
  rotmat_to_list(test:'RotationMatrix3D_1',
                 [0.99607825, -0.08847681, 0.0, 1.1171718,
                  0.08847681,  0.99607825, 0.0, 0.44037533,
                  0.0,         0.0,        1.0, 0.4702375,
                  0.0,         0.0,        0.0, 1.0]).

test(object_dimensions) :-
  object_dimensions(test:'Handle1', 0.015, 0.015, 0.015),
  !.

test(delete_object_information1) :-
  delete_object_information(test:'Handle1').

test(delete_object_information2, [fail]) :-
  rdf_has(_, _, test:'Handle1').

test(delete_object_information_recursive1) :-
  delete_object_information_recursive(test:'Dishwasher1').

test(delete_object_information_recursive2, [fail]) :-
  rdf_has(test:'Dishwasher1', knowrob:'parts', _).

test(object_assert_color) :-
  object_assert_color(test:'Cup1', '0.3 0.5 0.6 1'),
  object_color(test:'Cup1', '0.3 0.5 0.6 1').

test(object_color) :-
  object_color(test:'Cup1', '0.3 0.5 0.6 1').

test(object_assert_dimensions) :-
  object_assert_dimensions(test:'Cup2', 0.032, 0.032, 0.12),
  object_dimensions(test:'Cup2', 0.032, 0.032, 0.12),
  !.

%test(object_detection, fixme) :-
%  object_detection(test:'Cup1', test:'timepoint_1331040458', test:'SemanticMapPerception2').

:- end_tests(knowrob_objects).

