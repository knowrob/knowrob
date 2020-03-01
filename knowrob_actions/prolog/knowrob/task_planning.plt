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

test('WF_MakingPancakes_0 sequence', [nondet]) :-
  workflow_sequence(pancake:'WF_MakingPancakes_0', [
    pancake:'Mixing_0',
    pancake:'Baking_0'
  ]).

test('WF_Baking_0 sequence', [nondet]) :-
  workflow_sequence(pancake:'WF_Baking_0', [
    pancake:'TurningOn_0',
    pancake:'Pouring_0',
    pancake:'FlippingAPancake_0'
  ]).

test('WF_Mixing_0 roles', [nondet]) :-
  workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Egg'),
  workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Milk'),
  workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'WheatFlour'),
  workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'EggYolk'),
  workflow_role_range(pancake:'WF_Mixing_0', _, pancake:'Dough').

:- end_tests('knowrob/task_planning').
