%%
%% Copyright (C) 2016 by Daniel Beßler
%%
%% This file contains tests for the action-effect reasoning
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

:- begin_tests('knowrob/task_planning').

:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/task_planning')).
:- use_module(library('knowrob/model/Workflow'), [ workflow_role_range/3 ]).

:- owl_parse('package://knowrob_actions/owl/pancake.owl').

:- rdf_db:rdf_register_ns(pancake, 'http://knowrob.org/kb/pancake.owl#', [keep(true)]).

test(plan_pancake_events, [nondet]) :-
  workflow_sequence(pancake:'MakingPancakes_0_WF', SubEvents),
  nth0(0, SubEvents, 'http://knowrob.org/kb/pancake.owl#Cracking_0'),
  nth0(1, SubEvents, 'http://knowrob.org/kb/pancake.owl#Mixing_0'),
  nth0(2, SubEvents, 'http://knowrob.org/kb/pancake.owl#TurningOn_0'),
  nth0(3, SubEvents, 'http://knowrob.org/kb/pancake.owl#Pouring_0'),
  nth0(4, SubEvents, 'http://knowrob.org/kb/pancake.owl#FlippingAPancake_0').

test(plan_pancake_objects, [nondet]) :-
  workflow_role_range(pancake:'MakingPancakes_0_WF', _, pancake:'Egg'),
  workflow_role_range(pancake:'MakingPancakes_0_WF', _, pancake:'Milk'),
  workflow_role_range(pancake:'MakingPancakes_0_WF', _, pancake:'WheatFlour'),
  workflow_role_range(pancake:'MakingPancakes_0_WF', _, pancake:'EggYolk'),
  workflow_role_range(pancake:'MakingPancakes_0_WF', _, pancake:'Dough').

:- end_tests('knowrob/task_planning').
