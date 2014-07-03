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

:- begin_tests(comp_temporal).




% :- register_ros_package('knowrob_common').
% :- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('comp_temporal')).

:- owl_parser:owl_parse('package://comp_temporal/owl/comp_temporal.owl').
:- owl_parser:owl_parse('package://comp_temporal/owl/test_comp_temporal.owl').


:- rdf_db:rdf_register_ns(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(test, 'http://ias.cs.tum.edu/kb/test_comp_temporal.owl#', [keep(true)]).


% Test if
test(comp_temporallySubsumes) :-
    comp_temporallySubsumes(test:'Long', test:'Short1'),!.

test(comp_after) :-
    comp_after(test:'timepoint_1377777000', test:'timepoint_1377777002').

test(comp_duration) :-
    comp_duration('http://ias.cs.tum.edu/kb/test_comp_temporal.owl#Long', 10).

test(comp_equalI) :-
      comp_equalI(test:'Long', test:'Long'),
    \+comp_equalI(test:'Long', test:'Short1').

test(comp_beforeI) :-
      comp_beforeI(test:'Short1',test:'Short3'),
    \+comp_beforeI(test:'Short3',test:'Short1'),
    \+comp_beforeI(test:'Short1',test:'Short2'),
    \+comp_beforeI(test:'Short2',test:'Short1').

test(comp_afterI) :-
      comp_afterI(test:'Short3',test:'Short1'),
    \+comp_afterI(test:'Short1',test:'Short3'),
    \+comp_afterI(test:'Short1',test:'Short2'),
    \+comp_afterI(test:'Short2',test:'Short1').

test(comp_overlapsI) :-
      comp_overlapsI(test:'Long', test:'Short4'),
    \+comp_overlapsI(test:'Short4', test:'Long').

test(comp_overlapsInvI) :-
      comp_overlapsInvI(test:'Short4', test:'Long'),
    \+comp_overlapsInvI(test:'Long', test:'Short4').

test(comp_meetsI) :-
    comp_meetsI(test:'Short1', test:'Short2'),
    \+comp_meetsI(test:'Short1', test:'Short3').

test(comp_meetsInvI) :- 
    comp_meetsInvI(test:'Short2', test:'Short1'),
    \+comp_meetsInvI(test:'Short3', test:'Short1').

test(comp_duringI) :-
    comp_duringI(test:'Short2',test:'Long'),
    \+ comp_duringI(test:'Short1',test:'Short2').

test(comp_duringInvI) :- 
    comp_duringInvI(test:'Long',test:'Short2').

test(comp_startsI) :- 
    comp_startsI(test:'Short1',test:'Long').

test(comp_startsInvI) :- 
    comp_startsInvI(test:'Long',test:'Short1').

test(comp_finishesI) :- 
    comp_finishesI(test:'Short3',test:'Long'),
    \+ comp_finishesInvI(test:'Short4',test:'Long').

test(comp_finishesInvI) :-
    comp_finishesInvI(test:'Long',test:'Short3'),
    \+ comp_finishesInvI(test:'Long',test:'Short4').


:- end_tests(comp_temporal).

