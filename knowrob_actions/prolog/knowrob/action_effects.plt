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

:- begin_tests(action_effects).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/action_effects')).
:- use_module(library('knowrob/triple_memory')).

:- owl_parse('package://knowrob_actions/owl/blocksworld.owl').
:- owl_parse('package://knowrob_actions/owl/pancake.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(blocksworld,  'http://knowrob.org/kb/blocksworld.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake,  'http://knowrob.org/kb/pancake.owl#', [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Blocksworld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

red_on_blue   :- holds( blocksworld:ontop(blocksworld:'BlockRed_test0', blocksworld:'BlockBlue_test0') ), !.
blue_on_red   :- holds( blocksworld:ontop(blocksworld:'BlockBlue_test0', blocksworld:'BlockRed_test0') ), !.
red_on_table  :- holds( blocksworld:ontop(blocksworld:'BlockRed_test0', blocksworld:'Table_test0') ), !.
blue_on_table :- holds( blocksworld:ontop(blocksworld:'BlockBlue_test0', blocksworld:'Table_test0') ), !.
red_in_hand   :- holds( blocksworld:graspedBy(blocksworld:'BlockRed_test0', blocksworld:'Hand_test0') ), !.
blue_in_hand  :- holds( blocksworld:graspedBy(blocksworld:'BlockBlue_test0', blocksworld:'Hand_test0') ), !.

test(take_red0) :-
  mem_drop,
  \+ red_in_hand,
  red_on_table,
  action_effects_apply(blocksworld:'Take_red'),
  red_in_hand,
  \+ red_on_table.

test(put_red_on_blue) :-
  \+ red_on_blue,
  red_in_hand,
  blue_on_table,
  action_effects_apply(blocksworld:'Put_red_on_blue'),
  red_on_blue,
  \+ red_in_hand.

test(take_red1) :-
  red_on_blue,
  \+ red_in_hand,
  rdf_retractall(blocksworld:'Take_red', _, _, action_projection), % force re-projection
  action_effects_apply(blocksworld:'Take_red'),
  red_in_hand,
  \+ red_on_blue.

test(put_red_on_table) :-
  \+ red_on_table,
  red_in_hand,
  action_effects_apply(blocksworld:'Put_red_on_table'),
  \+ red_in_hand,
  red_on_table.

test(take_blue) :-
  blue_on_table,
  \+ blue_in_hand,
  action_effects_apply(blocksworld:'Take_blue'),
  blue_in_hand,
  \+ blue_on_table.

test(put_blue_on_red) :-
  \+ blue_on_red,
  blue_in_hand,
  red_on_table,
  action_effects_apply(blocksworld:'Put_blue_on_red'),
  blue_on_red,
  \+ blue_in_hand.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Pancake Making
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(turning_on_effect_device_state_on, [nondet]) :-
  action_effect_on_object(knowrob:'TurningOnHeatingDevice',
    updated(knowrob:stateOfObject,knowrob:'DeviceStateOn')).

test(cracking_effect_destroyed_egg, [nondet]) :-
  action_effect_on_object(pancake:'CrackingAnEgg', destroyed).

test(pancake_making_turn_on_maker) :-
  % turning on the pancake maker creates heating process
  action_effects_apply(pancake:'TurningOnPancakeMaker_0'),
  rdf_has(pancake:'TurningOnPancakeMaker_0', knowrob:processStarted, Heating),
  rdfs_individual_of(Heating, pancake:'HeatingProcess').
  
test(pancake_making_crack_egg) :-
  % create some egg yolk and egg shells
  action_effects_apply(pancake:'CrackingAnEgg_0'),
  once((rdf_has(pancake:'CrackingAnEgg_0', knowrob:outputsCreated, Yolk),
        rdfs_individual_of(Yolk, pancake:'EggYolk'))).

:- rdf_meta test_unsattisifed_restriction(r, t).
test_unsattisifed_restriction(Resource, Restr) :-
  owl_unsatisfied_restriction(Resource, RestrId),
  owl_restriction(RestrId, Restr), !.
  
test(pancake_making_mix_dough) :-
  % objectActedOn restriction not satisfied yet!
  test_unsattisifed_restriction(pancake:'MixPancakeDough_0',
      restriction(knowrob:'inputsDestroyed', cardinality(1,1,pancake:'EggYolk'))),
  % create dough by mixing flour and milk
  plan_constrained_objects(pancake:'MakingPancakes', pancake:'MixPancakeDough_0',
        [pancake:'CrackingAnEgg_0']),
  % objectActedOn restriction now satisfied!
  \+ test_unsattisifed_restriction(pancake:'MixPancakeDough_0',
      restriction(knowrob:'inputsDestroyed', cardinality(1,1,pancake:'EggYolk'))),
  % Check projection in store
  once((rdf_has(pancake:'MixPancakeDough_0', knowrob:objectActedOn, Yolk),
        rdfs_individual_of(Yolk, pancake:'EggYolk'))),
  action_effects_apply(pancake:'MixPancakeDough_0'),
  once((rdf_has(pancake:'MixPancakeDough_0', knowrob:outputsCreated, Dough),
        rdfs_individual_of(Dough, pancake:'Dough'))).
  
test(pancake_making_pour_dough) :-
  % pour ontop of pancake maker
  plan_constrained_objects(pancake:'MakingPancakes', pancake:'PourDoughOntoPancakeMaker_0',
        [pancake:'CrackingAnEgg_0', pancake:'MixPancakeDough_0']),
  rdf_has(pancake:'PourDoughOntoPancakeMaker_0', knowrob:objectTransported, Dough),
  rdfs_individual_of(Dough, pancake:'Dough'),
  action_effects_apply(pancake:'PourDoughOntoPancakeMaker_0'),
  % just assert that dough is on pancake maker as required by baking rules
  rdf_assert(Dough, knowrob:thermicallyConnectedTo, pancake:'PancakeMaker_0'),
  rdf_assert(pancake:'PancakeMaker_0', knowrob:thermicallyConnectedTo, Dough).

:- end_tests(action_effects).
