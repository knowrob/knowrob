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

:- begin_tests(comp_spatial).

:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_spatial')).

:- owl_parser:owl_parse('package://comp_spatial/owl/comp_spatial.owl').
:- owl_parser:owl_parse('package://comp_spatial/owl/test_comp_spatial.owl').


:- rdf_db:rdf_register_ns(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(test_sp, 'http://ias.cs.tum.edu/kb/test_comp_spatial.owl#', [keep(true)]).



test(inCenterOf1) :-
  rdf_triple(knowrob:inCenterOf, test_sp:'cup3', A),
  rdf_equal(A, test_sp:'cupboard1'),!.
test(inCenterOf2) :-
  rdf_triple(knowrob:inCenterOf, A, test_sp:'cupboard1'),
  rdf_equal(A, test_sp:'cup3'),!.
test(inCenterOf3) :-
  rdf_triple(knowrob:inCenterOf, test_sp:'cup3', test_sp:'cupboard1'),!.


test(in_ContGeneric1) :-
  rdf_triple(knowrob:'in-ContGeneric', test_sp:'cup3', A),
  rdf_equal(A, test_sp:'cupboard1'),!.
test(in_ContGeneric2) :-
  rdf_triple(knowrob:'in-ContGeneric', A, test_sp:'cupboard1'),
  rdf_equal(A, test_sp:'cup3'),!.
test(in_ContGeneric3) :-
  rdf_triple(knowrob:'in-ContGeneric', test_sp:'cup3', test_sp:'cupboard1'),!.



test(inFrontOf1) :-
  rdf_triple(knowrob:'inFrontOf-Generally', test_sp:'cup1', A),
  rdf_equal(A, test_sp:'cup2'),!.
test(inFrontOf2) :-
  rdf_triple(knowrob:'inFrontOf-Generally', A, test_sp:'cup2'),
  rdf_equal(A, test_sp:'cup1'),!.
test(inFrontOf3) :-
  rdf_triple(knowrob:'inFrontOf-Generally', test_sp:'cup1', test_sp:'cup2'),!.



test(on_physical1) :-
  rdf_triple(knowrob:'on-Physical', test_sp:'cup1', A),
  rdf_equal(A, test_sp:'cupboard1'),!.
test(on_physical2) :-
  rdf_triple(knowrob:'on-Physical', A, test_sp:'cupboard1'),
  rdf_equal(A, test_sp:'cup1'),!.
test(on_physical3) :-
  rdf_triple(knowrob:'on-Physical', test_sp:'cup1', test_sp:'cupboard1'),!.


  
test(aboveOf1) :-
  rdf_triple(knowrob:'aboveOf', test_sp:'cup1', A),
  rdf_equal(A, test_sp:'cupboard1'),!.
test(aboveOf2) :-
  rdf_triple(knowrob:'aboveOf', A, test_sp:'cupboard1'),
  rdf_equal(A, test_sp:'cup1'),!.
test(aboveOf3) :-
  rdf_triple(knowrob:'aboveOf', test_sp:'cup1', test_sp:'cupboard1'),!.


  
% test(belowOf1) :-
%   rdf_triple(knowrob:'belowOf', test_sp:'cupboard1', A),
%   rdf_equal(A, test_sp:'cup1'),!.
test(belowOf2) :-
  rdf_triple(knowrob:'belowOf', A, test_sp:'cup1'),
  rdf_equal(A, test_sp:'cupboard1'),!.
test(belowOf3) :-
  rdf_triple(knowrob:'belowOf', test_sp:'cupboard1', test_sp:'cup1'),!.


  
test(toTheLeftOf1) :-
  rdf_triple(knowrob:'toTheLeftOf', test_sp:'cup2', A),
  rdf_equal(A, test_sp:'cup1'),!.
test(toTheLeftOf2) :-
  rdf_triple(knowrob:'toTheLeftOf', A, test_sp:'cup1'),
  rdf_equal(A, test_sp:'cup2'),!.
test(toTheLeftOf3) :-
  rdf_triple(knowrob:'toTheLeftOf', test_sp:'cup2', test_sp:'cup1'),!.


  
test(toTheRightOf1) :-
  rdf_triple(knowrob:'toTheRightOf', test_sp:'cup1', A),
  rdf_equal(A, test_sp:'cup2'),!.
test(toTheRightOf2) :-
  rdf_triple(knowrob:'toTheRightOf', A, test_sp:'cup2'),
  rdf_equal(A, test_sp:'cup1'),!.
test(toTheRightOf3) :-
  rdf_triple(knowrob:'toTheRightOf', test_sp:'cup1', test_sp:'cup2'),!.


  
test(toTheSideOf1) :-
  rdf_triple(knowrob:'toTheSideOf', test_sp:'cup1', A),
  rdf_equal(A, test_sp:'cup2'),!.
test(toTheSideOf2) :-
  rdf_triple(knowrob:'toTheSideOf', A, test_sp:'cup2'),
  rdf_equal(A, test_sp:'cup1'),!.
test(toTheSideOf3) :-
  rdf_triple(knowrob:'toTheSideOf', test_sp:'cup1', test_sp:'cup2'),!.


:- end_tests(comp_spatial).

