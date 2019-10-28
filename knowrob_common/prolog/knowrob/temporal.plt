
:- begin_tests('knowrob/temporal').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).

:- owl_parser:owl_parse('package://knowrob_common/owl/test_comp_temporal.owl').
:- rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

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

:- end_tests('knowrob/temporal').
