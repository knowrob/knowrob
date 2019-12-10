%%
%% Copyright (C) 2016 by Daniel Beßler
%%
%% This file contains tests for the knowrob_temporal module
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

:- begin_tests('knowrob/entity').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/entity')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/memory')).

:- owl_parser:owl_parse('package://knowrob_entity/owl/test.owl').

:- rdf_db:rdf_register_prefix(entity_test, 'http://knowrob.org/kb/entity_test.owl#', [keep(true)]).

:- mem_drop.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OWL entity descriptions

%% Events

%% Objects

test(generate_refrigerator_description) :-
  entity(entity_test:'Refrigerator_fg45543', X),
  X = [an, object, [type, ease:refrigerator]].

test(query_refrigerator, [nondet]) :-
  entity(Cont, [an, object, [type, ease:refrigerator]]),
  rdf_equal(Cont, entity_test:'Refrigerator_fg45543').

test(query_cup, [nondet]) :-
  entity(Cup, [an, object, [type, ease_obj:'Crockery']]),
  rdf_equal(Cup, entity_test:'Cup_sfd498th').

test(query_containerFor, [nondet]) :-
  entity(Obj, [an, object,
    [ease_obj:hasDisposition, [
      [type,ease_obj:'Insertion'],
      [ease_obj:affordsTrigger, [
        [classifies, only(ease:'DesignedContainer')]
      ]]
    ]]
  ]),
  rdf_equal(Obj, entity_test:'Refrigerator_fg45543').

test(query_cup_by_name, [nondet]) :-
  entity(Cont, [an, object, [name, entity_test:'Cup_sfd498th']]),
  rdf_equal(Cont, entity_test:'Cup_sfd498th').

test(query_cup_by_nameString_prop, [nondet]) :-
  entity(Cont, [an, object, [label, 'cup_name']]),
  rdf_equal(Cont, entity_test:'Cup_sfd498th').

test(query_cup_by_nameString_prop_2, [nondet]) :-
  entity(Cont, [an, object, [label, X]]),
  X = 'cup_name',
  rdf_equal(Cont, entity_test:'Cup_sfd498th').
  
:- end_tests('knowrob/entity').
