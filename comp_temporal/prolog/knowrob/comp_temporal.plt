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
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/comp_temporal')).

:- owl_parser:owl_parse('package://comp_temporal/owl/comp_temporal.owl').
:- owl_parser:owl_parse('package://comp_temporal/owl/test_comp_temporal.owl').


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).


% Test if
test(comp_temporallySubsumes) :-
    comp_temporallySubsumes(test_comp_temporal:'Long', test_comp_temporal:'Short1'),!.

test(comp_after) :-
    comp_after(test_comp_temporal:'timepoint_1377777002', test_comp_temporal:'timepoint_1377777000').

test(comp_duration) :-
    comp_duration('http://knowrob.org/kb/test_comp_temporal.owl#Long', 10).

test(comp_equalI) :-
      comp_equalI(test_comp_temporal:'Long', test_comp_temporal:'Long'),
    \+comp_equalI(test_comp_temporal:'Long', test_comp_temporal:'Short1').

test(comp_beforeI) :-
      comp_beforeI(test_comp_temporal:'Short1',test_comp_temporal:'Short3'),
    \+comp_beforeI(test_comp_temporal:'Short3',test_comp_temporal:'Short1'),
    \+comp_beforeI(test_comp_temporal:'Short1',test_comp_temporal:'Short2'),
    \+comp_beforeI(test_comp_temporal:'Short2',test_comp_temporal:'Short1').

test(comp_afterI) :-
      comp_afterI(test_comp_temporal:'Short3',test_comp_temporal:'Short1'),
    \+comp_afterI(test_comp_temporal:'Short1',test_comp_temporal:'Short3'),
    \+comp_afterI(test_comp_temporal:'Short1',test_comp_temporal:'Short2'),
    \+comp_afterI(test_comp_temporal:'Short2',test_comp_temporal:'Short1').

test(comp_overlapsI) :-
      comp_overlapsI(test_comp_temporal:'Long', test_comp_temporal:'Short4'),
    \+comp_overlapsI(test_comp_temporal:'Short4', test_comp_temporal:'Long').

test(comp_overlapsInvI) :-
      comp_overlapsInvI(test_comp_temporal:'Short4', test_comp_temporal:'Long'),
    \+comp_overlapsInvI(test_comp_temporal:'Long', test_comp_temporal:'Short4').

test(comp_meetsI) :-
    comp_meetsI(test_comp_temporal:'Short1', test_comp_temporal:'Short2'),
    \+comp_meetsI(test_comp_temporal:'Short1', test_comp_temporal:'Short3').

test(comp_meetsInvI) :- 
    comp_meetsInvI(test_comp_temporal:'Short2', test_comp_temporal:'Short1'),
    \+comp_meetsInvI(test_comp_temporal:'Short3', test_comp_temporal:'Short1').

test(comp_duringI) :-
    comp_duringI(test_comp_temporal:'Short2',test_comp_temporal:'Long'),
    \+ comp_duringI(test_comp_temporal:'Short1',test_comp_temporal:'Short2').

test(comp_duringInvI) :- 
    comp_duringInvI(test_comp_temporal:'Long',test_comp_temporal:'Short2').

test(comp_startsI) :- 
    comp_startsI(test_comp_temporal:'Short1',test_comp_temporal:'Long').

test(comp_startsInvI) :- 
    comp_startsInvI(test_comp_temporal:'Long',test_comp_temporal:'Short1').

test(comp_finishesI) :- 
    comp_finishesI(test_comp_temporal:'Short3',test_comp_temporal:'Long'),
    \+ comp_finishesInvI(test_comp_temporal:'Short4',test_comp_temporal:'Long').

test(comp_finishesInvI) :-
    comp_finishesInvI(test_comp_temporal:'Long',test_comp_temporal:'Short3'),
    \+ comp_finishesInvI(test_comp_temporal:'Long',test_comp_temporal:'Short4').


test(query_event_after) :-
  findall(Evt, entity(Evt, [an, event, [comp_temporal:afterI, [a, timepoint, 1377777004.0]]]), Evts),
  member('http://knowrob.org/kb/test_comp_temporal.owl#Short3', Evts),
  member('http://knowrob.org/kb/test_comp_temporal.owl#Short4', Evts),
  not( member('http://knowrob.org/kb/test_comp_temporal.owl#Short2', Evts) ), !.


:- end_tests(comp_temporal).

