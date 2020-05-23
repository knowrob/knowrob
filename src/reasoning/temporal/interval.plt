
:- begin_tests(time_interval).

test(time_interval0) :-
  	X is 2 + 2,
  	( X = 4 ; X = 4 ).

test(time_interval2) :-
  	fail.

test(time_interval1) :-
  	X = 'a',
  	Y = 'a',
  	X = Y.

%:- use_module(library('knowrob/reasoning/temporal/allen')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% Interval algebra

%test(interval_during0) :-
  %interval_during(1.0, [0.0]).

%test(interval_during1) :-
  %interval_during(1.0, [0.0,2.0]).

%test(interval_during2) :-
  %interval_during([1.0,2.0], [0.0,2.0]).

%test(interval_during3) :-
  %interval_during([1.0], [0.0]).

%test(interval_during4, [fail]) :-
  %interval_during(1.0, [2.0]).

%test(interval_during5, [fail]) :-
  %interval_during([1.0], [2.0]).

%test(interval_during6, [fail]) :-
  %interval_during([1.0,3.0], [2.0]).

%test(interval_during7, [fail]) :-
  %interval_during(6.0, [2.0,4.0]).

%test(interval_during8, [fail]) :-
  %interval_during([1.0], [2.0,4.0]).

%test(interval_during9, [fail]) :-
  %interval_during([3.0], [2.0,4.0]).

%test(interval_during10, [fail]) :-
  %interval_during([2.0,5.0], [2.0,4.0]).

:- end_tests(time_interval).
