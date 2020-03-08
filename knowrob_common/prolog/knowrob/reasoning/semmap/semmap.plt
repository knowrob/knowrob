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

:- begin_tests('knowrob/reasoning/semmap/semmap').

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/comp/object_pose')).
:- use_module(library('knowrob/reasoning/semmap/semmap')).

:- owl_parser:owl_parse('package://knowrob_common/owl/maps/iai_room_v1.owl', belief_state).

:- rdf_db:rdf_register_ns(iai_map_v1, 'http://knowrob.org/kb/v1/IAI-Kitchen.owl#', [keep(true)]).

test(map_root_objects) :-
  rdf_equal(Drawer, iai_map_v1:'drawer_oven_lower'),
  map_root_objects(iai_map_v1, RootObjects),
  member(Drawer, RootObjects),
  length(RootObjects, 22), !.
  
test(map_root_object) :-
  map_root_object(iai_map_v1, iai_map_v1:'drawer_oven_lower'),!.

test(map_object_dimensions) :-
  map_object_dimensions(iai_map_v1:'drawer_oven_lower', 0.5, 0.59, 0.58),!.

test(map_child_object) :-
  map_child_object(iai_map_v1:'drawer_oven_right',
                   iai_map_v1:'drawer_oven_right_handle'),!.

test(map_object_most_similar) :-
  map_object_most_similar(iai_map_v1:'drawer_sinkblock_dishwasher', _).

test(map_child_objects) :-
  rdf_equal(Handle, iai_map_v1:'drawer_oven_right_handle'),
  map_child_objects(iai_map_v1:'drawer_oven_right', Objects),
  member(Handle, Objects),
  length(Objects, 2), !.

test(map_object_pose) :-
  current_object_pose(iai_map_v1:'drawer_oven_lower',
                      [_,_,[_,_,_],[_,_,_,_]]).

test(map_object_info) :-
  map_object_info([iai_map_v1:'drawer_oven_lower',
                   ease:'Drawer',
                   [  map,
                      drawer_oven_lower,
                      [1.4,1.88,0.44],
                      [0.0,0.0,0.0,1.0]
                   ],
                   [0.5,0.59,0.58]]), !.

test(map_object_type) :-
  map_object_type(iai_map_v1:'drawer_oven_lower', ease:'Drawer'),!.

:- end_tests('knowrob/reasoning/semmap/semmap').

