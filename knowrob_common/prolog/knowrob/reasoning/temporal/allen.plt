
:- begin_tests('knowrob/reasoning/temporal/allen').

:- use_module(library('knowrob/reasoning/temporal/allen')).

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

:- end_tests('knowrob/reasoning/temporal/allen').
