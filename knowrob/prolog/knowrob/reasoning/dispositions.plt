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

:- begin_tests('knowrob/reasoning/dispositions').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/reasoning/dispositions')).
:- use_module(library('knowrob/model/Object'),[
    object_disposition/3,
    disposition_trigger_type/2
]).

:- owl_parser:owl_parse('package://knowrob/owl/test_knowrob_objects.owl').
:- rdf_db:rdf_register_prefix(test_obj, 'http://knowrob.org/kb/test_knowrob_objects.owl#', [keep(true)]).

test(object_disposition) :-
  object_disposition(test_obj:'Dishwasher1', _, ease_obj:'Insertion').

test(disposition_trigger_type) :-
  object_disposition(test_obj:'Dishwasher1', D, ease_obj:'Insertion'),!,
  disposition_trigger_type(D, ease_obj:'Tableware').

test(storage_place_for1) :-
  storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup1').

test(storage_place_for2) :-
  storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup2').

test(storage_place_for3) :-
  storage_place_for(test_obj:'Dishwasher1', ease_obj:'Tableware').

test(storage_place_for_because1) :-
  storage_place_for_because(test_obj:'Dishwasher1',test_obj:'Cup1',X),
  rdf_equal(X, ease_obj:'Tableware').

%test(object_dimensions) :-
  %object_dimensions(test_obj:'Handle1', 0.015, 0.015, 0.015).

%test(object_assert_color) :-
  %object_assert_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]),
  %object_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]).

%test(object_assert_dimensions) :-
  %object_assert_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12),
  %object_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12).

:- end_tests('knowrob/reasoning/dispositions').
