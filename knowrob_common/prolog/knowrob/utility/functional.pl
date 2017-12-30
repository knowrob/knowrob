/*
  Copyright (C) 2009 Bernhard Kirchlechner, Moritz Tenorth
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(knowrob_functional,
    [ reduce/4,
      range/3,
      zip/3,
      zipm/3
]).
/** <module> Prolog/OWL utility predicates

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
