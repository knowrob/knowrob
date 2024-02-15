:- module(functional,
    [ reduce/4,
      range/3,
      zip/3,
      zipm/3
    ]).
/** <module> Functional programming.

@author Bernhard Kirchlechner, Moritz Tenorth
@license BSD
*/

%% reduce(+Predicate, +List, +StartValue, -Result).
% The predicate is first called for the first element of the list, the start value and an intermediate result.
% Then it is repeatedly called for the next elements, the intermediate result and a new intermediate result.
% The final value is the result when the predicate has been called the last time.
% E.g. reduce(plus, [1,2,3,4], 0, 10) holds.
%
reduce(_, [], StartValue, StartValue).
reduce(Predicate, [Element|Rest], Accumulator, Result) :-
  call(Predicate, Element, Accumulator, NewAccumulator),
  reduce(Predicate, Rest, NewAccumulator, Result).

%% range(+M, +N, -Range)
% Generate a list of integers in a given range. E.g. range(5, 0, [5,4,3,2,1,0].
%
range(N,N,[N]) :- !.
range(M,N,[M|Ns]) :- nonvar(N), M < N, !, M1 is M+1, range(M1, N, Ns).
range(M,N,[M|Ns]) :- nonvar(N), M > N, !, M1 is M-1, range(M1, N, Ns).

%% zip(?List1, ?List2, ?List3)
% Zip the lists 1 and 2 together to get list3.
% It behaves like zip in python: zip([a,b],[1,2],[[a,1],[b,2]]).
%
zip([], [], []).
zip([A|ARest], [B|BRest], [[A,B]|CRest]) :-
  zip(ARest, BRest, CRest).

%% zipm(?List1, ?List2, ?List3)
% Zip the lists 1 and 2 together to get list3.
% It behaves similar to zip in python but uses - for concatenation: zip([a,b],[1,2],[a-1,b-2]).
%
zipm([], [], []).
zipm([A|ARest], [B|BRest], [A-B|CRest]) :-
  zipm(ARest, BRest, CRest).
