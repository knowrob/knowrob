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

:- begin_tests('knowrob/action_effects').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_effects')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/objects')).

:- use_module(library('knowrob/model/Event'), [ event_participant/3, event_set_begin_time/2, event_set_end_time/2 ]).
:- use_module(library('knowrob/model/Action'), [ action_create/3, action_set_task/2 ]).
:- use_module(library('knowrob/model/Task'), [ task_create/3, task_parameter/3, task_role/3 ]).
:- use_module(library('knowrob/model/Object'), [ object_is_alive/1 ]).

:- owl_parse('package://knowrob_actions/owl/blocksworld.owl').
:- owl_parse('package://knowrob_actions/owl/pancake.owl').

:- rdf_db:rdf_register_ns(blocksworld,  'http://knowrob.org/kb/blocksworld.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake,  'http://knowrob.org/kb/pancake.owl#', [keep(true)]).

:- rdf_meta create_input_dict(t,t).

create_input_dict(Dict,List) :-
  findall(X-Y, member([X,Y],List),Pairs),
  dict_pairs(Dict,_,Pairs).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Blocksworld
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

yellow_on_blue  :- holds( ease_obj:isOntopOf(blocksworld:'BlockYellow_0', blocksworld:'BlockBlue_0') ), !.
yellow_on_red   :- holds( ease_obj:isOntopOf(blocksworld:'BlockYellow_0', blocksworld:'BlockRed_0') ), !.
red_on_blue     :- holds( ease_obj:isOntopOf(blocksworld:'BlockRed_0', blocksworld:'BlockBlue_0') ), !.

:- rdf_meta create_action_for_task_(r,r,r).

% helper
create_action_for_task_(TskType,Act,Tsk) :-
  current_time(T0),
  task_create(TskType,Tsk,belief_state),
  action_create(dul:'Action',Act,belief_state),
  action_set_task(Act,Tsk),
  event_set_begin_time(Act,T0),
  event_set_end_time(Act,T0).

test('blocksworld deposited') :-
  action_effect(blocksworld:'Stack', deposited(dul:'Object')).
test('blocksworld deposited-unbound',[nondet]) :-
  action_effect(blocksworld:'Stack', X),
  rdf_global_term(deposited(dul:'Object'),X).
test('blocksworld deposited-through',[nondet]) :-
  action_effect(X, deposited(dul:'Object')),
  rdf_global_term(blocksworld:'Stack',X).

test('blocksworld extracted') :-
  action_effect(blocksworld:'Unstack', extracted(dul:'Object')).
  
test('Unstack_Y') :-
  yellow_on_blue,
  create_action_for_task_(blocksworld:'Unstack',Act,Tsk),
  task_role(Tsk,Deposit,ease_obj:'Deposit'),
  task_role(Tsk,Extracted,ease_obj:'ExtractedObject'),!,
  create_input_dict(Dict,[
      [Deposit,blocksworld:'BlockBlue_0'],
      [Extracted,blocksworld:'BlockYellow_0']
  ]),
  action_effects_apply(Act,Dict),
  \+ yellow_on_blue.

test('Stack_RB') :-
  \+ red_on_blue,
  create_action_for_task_(blocksworld:'Stack',Act,Tsk),
  task_role(Tsk,Deposit,ease_obj:'Deposit'),
  task_role(Tsk,Deposited,ease_obj:'DepositedObject'),!,
  create_input_dict(Dict,[
      [Deposit,blocksworld:'BlockBlue_0'],
      [Deposited,blocksworld:'BlockRed_0']
  ]),
  action_effects_apply(Act,Dict),
  red_on_blue.

test('Stack_YR') :-
  \+ yellow_on_red,
  create_action_for_task_(blocksworld:'Stack',Act,Tsk),
  task_role(Tsk,Deposit,ease_obj:'Deposit'),
  task_role(Tsk,Deposited,ease_obj:'DepositedObject'),!,
  create_input_dict(Dict,[
      [Deposit,blocksworld:'BlockRed_0'],
      [Deposited,blocksworld:'BlockYellow_0']
  ]),
  action_effects_apply(Act,Dict),
  yellow_on_red.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Pancake Making
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- dynamic test_mix_dough/1, test_bake_dough/1.

test(effect_turning_on) :-
  action_effect(
    pancake:'TurningOnHeatingDevice',
    altered(pancake:'PancakeMaker', ease_obj:'DeviceState')).

test(effect_cracking_destroyed_egg) :-
  action_effect(pancake:'CrackingAnEgg', destroyed(pancake:'Egg')).
test(effect_cracking_created_egg_yolk) :-
  action_effect(pancake:'CrackingAnEgg', created(pancake:'EggYolk')).
test(effect_cracking_created_egg_shell) :-
  action_effect(pancake:'CrackingAnEgg', created(pancake:'EggShell')).

test(pancake_making_turn_on_maker) :-
  %%
  kb_triple(pancake:'PancakeMaker_0_DeviceState',
    dul:hasRegion, pancake:'PancakeMaker_0_RegionOff'),
  %%
  create_action_for_task_(pancake:'TurningOnHeatingDevice',Act,Tsk),
  task_parameter(Tsk,Setpoint,ease_obj:'Setpoint'),
  task_role(Tsk,Patient,ease_obj:'AlteredObject'),!,
  create_input_dict(Dict,[
      [Setpoint,pancake:'PancakeMaker_0_RegionOn'],
      [Patient,pancake:'PancakeMaker_0']
  ]),
  %%
  action_effects_apply(Act,Dict),
  %%
  kb_triple(pancake:'PancakeMaker_0_DeviceState',
    dul:hasRegion, pancake:'PancakeMaker_0_RegionOn').
  
test(pancake_making_crack_egg) :-
  % create some egg yolk and egg shells
  object_is_alive(pancake:'Egg_0'),
  create_action_for_task_(pancake:'CrackingAnEgg',Act,Tsk),
  task_role(Tsk,Destroyed,ease_obj:'DestroyedObject'),!,
  create_input_dict(Dict,[
      [Destroyed,pancake:'Egg_0']
  ]),
  %%
  action_effects_apply(Act,Dict),
  %%
  \+ object_is_alive(pancake:'Egg_0'),
  once((
    event_participant(Act,_,pancake:'EggShell'),
    event_participant(Act,_,pancake:'EggYolk')
  )).

%%%%%%
% Task: pancake:'MixPancakeDough'
%
test(pancake_making_mix_dough) :-
  create_action_for_task_(pancake:'MixPancakeDough',Act,Tsk),
  %%
  task_role(Tsk,Egg,ease_obj:'CommitedObject'),
  property_range(Egg,dul:classifies,pancake:'Egg'),
  %%
  task_role(Tsk,Milk,ease_obj:'CommitedObject'),
  property_range(Milk,dul:classifies,pancake:'Milk'),
  %%
  task_role(Tsk,Wheat,ease_obj:'CommitedObject'),
  property_range(Wheat,dul:classifies,pancake:'WheatFlour'),!,
  %%
  create_input_dict(Dict,[
      [Egg,pancake:'Egg_0'],
      [Milk,pancake:'Milk_0'],
      [Wheat,pancake:'WheatFlour_0']
  ]),
  %%
  action_effects_apply(Act,Dict),
  assertz(test_mix_dough(Act)).

test(pancake_making_mix_dough_exists, [nondet]) :-
  % dough was created
  test_mix_dough(Act),
  event_participant(Act,_,pancake:'Dough').

test(pancake_making_mix_dough_part1, [nondet]) :-
  % dough has parts that are ingredients
  test_mix_dough(Act),
  event_participant(Act,Dough0,pancake:'Dough'),
  rdf_has(Dough0,dul:hasPart,pancake:'Egg_0').

test(pancake_making_mix_dough_part2, [nondet]) :-
  % dough has parts that are ingredients
  test_mix_dough(Act),
  event_participant(Act,Dough0,pancake:'Dough'),
  rdf_has(Dough0,dul:hasPart,pancake:'Milk_0').

test(pancake_making_mix_dough_part3, [nondet]) :-
  % dough has parts that are ingredients
  test_mix_dough(Act),
  event_participant(Act,Dough0,pancake:'Dough'),
  rdf_has(Dough0,dul:hasPart,pancake:'WheatFlour_0').

test(pancake_making_mix_dough_alive, [nondet]) :-
  % do is alive after the action
  test_mix_dough(Act),
  event_participant(Act,Dough0,pancake:'Dough'),
  object_is_alive(Dough0).

%%%%%%
% Task: pancake:'BakePancake'
%
test(pancake_making_bake_dough) :-
  test_mix_dough(MixAct),
  event_participant(MixAct,Dough0,pancake:'Dough'),
  %% the dough transforms into a dough-pancake-transient when baked.
  create_action_for_task_(pancake:'BakingAPancake',Act,Tsk),
  %%
  task_role(Tsk,Dough,ease_obj:'CommitedObject'),
  property_range(Dough,dul:classifies,pancake:'Dough'),
  %%
  task_role(Tsk,Pancake,ease_obj:'CreatedObject'),
  property_range(Pancake,dul:classifies,pancake:'MyPancake'),!,
  %%
  create_input_dict(Dict,[ [Dough,Dough0] ]),
  action_effects_apply(Act,Dict),
  assertz(test_bake_dough(Act)).

test(pancake_making_bake_not_alive_Dough, [nondet]) :-
  % the dough is not alive after the action was performed
  test_bake_dough(Act),
  event_participant(Act,Dough0,pancake:'Dough'),
  \+ object_is_alive(Dough0).

test(pancake_making_bake_created_Pancake, [nondet]) :-
  % the action has created a pancake
  test_bake_dough(Act),
  event_participant(Act,_,pancake:'MyPancake').

test(pancake_making_bake_alive_Pancake, [nondet]) :-
  % the pancake is alive after the action was performed
  test_bake_dough(Act),
  event_participant(Act,Pancake0,pancake:'MyPancake'),
  object_is_alive(Pancake0).

test(pancake_making_bake_Pancake_parts, [nondet]) :-
  % parts of the dough remain parts of the pancake
  test_bake_dough(Act),
  event_participant(Act,Pancake0,pancake:'MyPancake'),
  kb_triple(Pancake0,dul:hasPart,pancake:'Milk_0'),
  kb_triple(Pancake0,dul:hasPart,pancake:'WheatFlour_0').

:- end_tests('knowrob/action_effects').
