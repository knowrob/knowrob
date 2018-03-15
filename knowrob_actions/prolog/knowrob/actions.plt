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

:- begin_tests(knowrob_actions).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/action_effects')).

:- owl_parse('package://knowrob_actions/owl/pancake-making.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake,  'http://knowrob.org/kb/pancake-making.owl#', [keep(true)]).

test(plan_pancake_events, [nondet]) :-
  plan_subevents(pancake:'MakingPancakes', SubEvents),
  member('http://knowrob.org/kb/pancake-making.owl#CrackingAnEgg', SubEvents),
  member('http://knowrob.org/kb/pancake-making.owl#MixPancakeDough', SubEvents),
  member('http://knowrob.org/kb/pancake-making.owl#PourDoughOntoPancakeMaker', SubEvents),
  member('http://knowrob.org/kb/pancake-making.owl#FlippingAPancake', SubEvents).

test(plan_pancake_objects, [nondet]) :-
  plan_objects(pancake:'MakingPancakes', Objs),
  member('http://knowrob.org/kb/knowrob.owl#Egg-Chickens', Objs),
  member('http://knowrob.org/kb/knowrob.owl#CowsMilk-Product', Objs),
  member('http://knowrob.org/kb/knowrob.owl#WheatFlour', Objs),
  member('http://knowrob.org/kb/knowrob.owl#EggYolk-Food', Objs),
  member('http://knowrob.org/kb/knowrob.owl#Dough', Objs),
  member('http://knowrob.org/kb/knowrob.owl#Baked', Objs).

:- end_tests(knowrob_actions).
