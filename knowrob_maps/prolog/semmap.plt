%%
%% Copyright (C) 2014 by Moritz Tenorth
%%
%% This file contains tests for the spatial reasoning
%% tools in KnowRob.
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

:- begin_tests(semmap).

:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/objects'), [current_object_pose/2]).
:- use_module(library('knowrob/perception'), [object_import_detections/1]).
:- use_module(library(semmap)).

:- owl_parser:owl_parse('package://knowrob_maps/owl/ccrl2_semantic_map.owl', belief_state).

:- rdf_db:rdf_register_ns(ias_map, 'http://knowrob.org/kb/ias_semantic_map.owl#', [keep(true)]).

test(map_instance) :-
  map_instance(ias_map:'SemanticEnvironmentMap0').

test(map_root_objects) :-
  map_root_objects(ias_map:'SemanticEnvironmentMap0', O),
  member('http://knowrob.org/kb/knowrob.owl#CounterTop205', O),
  length(O, 23),!.
  
test(map_root_object) :-
  map_root_object(ias_map:'SemanticEnvironmentMap0', knowrob:'CounterTop205'),!.

test(map_object_dimensions) :-
  map_object_dimensions(knowrob:'CounterTop205', 2.0500002, 0.57500005, 0.02),!.

test(map_child_object) :-
  map_child_object(knowrob:'Dishwasher37', knowrob:'Door40'),
  map_child_object(knowrob:'Dishwasher37', knowrob:'Handle145'),!.

test(map_object_most_similar) :-
  map_object_most_similar(knowrob:'Dishwasher37', _).

test(map_child_objects) :-
  map_child_objects(knowrob:'Dishwasher37', Objects),
  member('http://knowrob.org/kb/knowrob.owl#Door40', Objects),
  member('http://knowrob.org/kb/knowrob.owl#Handle145', Objects),
  member('http://knowrob.org/kb/knowrob.owl#Hinge40', Objects),
  length(Objects, 3), !.

test(map_import_detections) :-
  forall(
    map_root_object(ias_map:'SemanticEnvironmentMap0', Obj),
    object_import_detections(Obj)
  ).

test(map_object_pose) :-
  current_object_pose('http://knowrob.org/kb/knowrob.owl#CounterTop205',
                      [_,_,[_,_,_],[_,_,_,_]]).

test(map_object_info) :-
  map_object_info(['http://knowrob.org/kb/knowrob.owl#CounterTop205',
                   'http://knowrob.org/kb/knowrob.owl#CounterTop',
                   [  map,
                      counter_top_sink_link,
                      [1.1006587,0.54706275,0.84],
                      [0.0,0.0,0.7377251659437472,0.6751011750548789]
                   ],
                   [2.0500002,0.57500005,0.02]]), !.

test(map_object_type) :-
  map_object_type('http://knowrob.org/kb/knowrob.owl#CounterTop205',
                   'http://knowrob.org/kb/knowrob.owl#CounterTop'),!.

test(map_object_label) :-
  map_object_label('http://knowrob.org/kb/knowrob.owl#CounterTop205', 'Counter'),!.



:- end_tests(semmap).

