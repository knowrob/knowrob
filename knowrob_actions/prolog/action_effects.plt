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

:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('action_effects')).

:- owl_parse('package://knowrob_actions/owl/action-effects.owl').
:- owl_parse('package://knowrob_actions/owl/blocksworld.owl').

:- rdf_db:rdf_register_ns(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(blocksworld,  'http://knowrob.org/kb/blocksworld.owl#', [keep(true)]).

red_on_blue :-
  holds( blocksworld:ontop(blocksworld:'BlockRed_test0', blocksworld:'BlockBlue_test0') ), !.
blue_on_red :-
  holds( blocksworld:ontop(blocksworld:'BlockBlue_test0', blocksworld:'BlockRed_test0') ), !.
red_on_table :-
  holds( blocksworld:ontop(blocksworld:'BlockRed_test0', blocksworld:'Table_test0') ), !.
blue_on_table :-
  holds( blocksworld:ontop(blocksworld:'BlockBlue_test0', blocksworld:'Table_test0') ), !.
red_in_hand :-
  holds( blocksworld:graspedBy(blocksworld:'BlockRed_test0', blocksworld:'Hand_test0') ), !.
blue_in_hand :-
  holds( blocksworld:graspedBy(blocksworld:'BlockBlue_test0', blocksworld:'Hand_test0') ), !.

test(take_red0) :-
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

