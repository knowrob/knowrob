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

:- begin_tests(knowrob_temporal).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).

:- owl_parser:owl_parse('package://knowrob_common/owl/comp_temporal.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/test_comp_temporal.owl').

:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Interval algebra

test(interval_during0) :-
  interval_during(1.0, [0.0]).

test(interval_during1) :-
  interval_during(1.0, [0.0,2.0]).

test(interval_during2) :-
  interval_during([1.0,2.0], [0.0,2.0]).

test(interval_during3) :-
  interval_during([1.0], [0.0]).

test(interval_during4, [fail]) :-
  interval_during(1.0, [2.0]).

test(interval_during5, [fail]) :-
  interval_during([1.0], [2.0]).

test(interval_during6, [fail]) :-
  interval_during([1.0,3.0], [2.0]).

test(interval_during7, [fail]) :-
  interval_during(6.0, [2.0,4.0]).

test(interval_during8, [fail]) :-
  interval_during([1.0], [2.0,4.0]).

test(interval_during9, [fail]) :-
  interval_during([3.0], [2.0,4.0]).

test(interval_during10, [fail]) :-
  interval_during([2.0,5.0], [2.0,4.0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
test(rdf_interval) :-
  interval(test_comp_temporal:'Short1', [1377777000,1377777002]).

test(comp_during) :-
  holds(test_comp_temporal:'Short1', ease:during, test_comp_temporal:'Long').

test(comp_after) :-
  holds(test_comp_temporal:'timepoint_1377777002', ease:after, test_comp_temporal:'timepoint_1377777000').

test(comp_simultaneous) :-
  holds(test_comp_temporal:'Long',ease:simultaneous,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Long',ease:simultaneous,test_comp_temporal:'Short1').

test(comp_before) :-
  holds(test_comp_temporal:'Short1',ease:before,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Short3',ease:before,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short1',ease:before,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short2',ease:before,test_comp_temporal:'Short1').

test(comp_after) :-
  holds(test_comp_temporal:'Short3',ease:after,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short1',ease:after,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Short1',ease:after,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short2',ease:after,test_comp_temporal:'Short1').

test(comp_overlaps) :-
  holds(test_comp_temporal:'Long',ease:overlappedOn,test_comp_temporal:'Short4'),
  \+ holds(test_comp_temporal:'Short4',ease:overlappedOn,test_comp_temporal:'Long').

test(comp_overlappedBy) :-
  holds(test_comp_temporal:'Short4',ease:overlappedBy,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Long',ease:overlappedBy,test_comp_temporal:'Short4').

test(comp_meets) :-
  holds(test_comp_temporal:'Short1',ease:meets,test_comp_temporal:'Short2'),
  \+ holds(test_comp_temporal:'Short1',ease:meets,test_comp_temporal:'Short3').

test(comp_metBy) :- 
  holds(test_comp_temporal:'Short2',ease:metBy,test_comp_temporal:'Short1'),
  \+ holds(test_comp_temporal:'Short3',ease:metBy,test_comp_temporal:'Short1').

test(comp_during) :-
  holds(test_comp_temporal:'Short2',ease:during,test_comp_temporal:'Long'),
  \+ holds(test_comp_temporal:'Short1',ease:during,test_comp_temporal:'Short2').

test(comp_starts) :- 
  holds(test_comp_temporal:'Short1',ease:starts,test_comp_temporal:'Long').

test(comp_startedBy) :- 
  holds(test_comp_temporal:'Long',ease:startedBy,test_comp_temporal:'Short1').

test(comp_finishes) :- 
  holds(test_comp_temporal:'Short3',ease:finishes,test_comp_temporal:'Long').

test(comp_finishedBy) :-
  holds(test_comp_temporal:'Long',ease:finishedBy,test_comp_temporal:'Short3'),
  \+ holds(test_comp_temporal:'Long',ease:finishedBy,test_comp_temporal:'Short4').

:- end_tests(knowrob_temporal).
