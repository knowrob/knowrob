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

:- owl_parse('package://knowrob_actions/owl/pancake.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake,  'http://knowrob.org/kb/pancake.owl#', [keep(true)]).

test(plan_pancake_events, [nondet]) :-
  plan_subevents(pancake:'MakingPancakes', SubEvents),
  nth0(0, SubEvents, 'http://knowrob.org/kb/pancake.owl#CrackingAnEgg'),
  nth0(1, SubEvents, 'http://knowrob.org/kb/pancake.owl#MixPancakeDough'),
  nth0(2, SubEvents, 'http://knowrob.org/kb/pancake.owl#PourDoughOntoPancakeMaker'),
  nth0(3, SubEvents, 'http://knowrob.org/kb/pancake.owl#FlippingAPancake').

test(plan_pancake_objects, [nondet]) :-
  plan_objects(pancake:'MakingPancakes', Objs),
  member('http://knowrob.org/kb/pancake.owl#Egg-Chickens', Objs),
  member('http://knowrob.org/kb/pancake.owl#Milk', Objs),
  member('http://knowrob.org/kb/pancake.owl#WheatFlour', Objs),
  member('http://knowrob.org/kb/pancake.owl#EggYolk', Objs),
  member('http://knowrob.org/kb/pancake.owl#Dough', Objs).

:- end_tests(knowrob_actions).
