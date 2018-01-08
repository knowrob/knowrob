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

:- module(knowrob_jpl,
    [ arrays_to_lists/2,
      lists_to_arrays/2
]).
/** <module> Prolog/OWL utility predicates

@author Bernhard Kirchlechner, Moritz Tenorth
@license BSD
*/

%% arrays_to_lists(+Object, -Result).
%
% Takes a java Object, and converts any arrays to lists.
% (i.e. double[][] becomes list of list of doubles).
% 
arrays_to_lists(Object, Result) :-
  jpl_object_to_type(Object,array(_)), !,
  jpl_array_to_list(Object, TempList),
  arrToListWork(TempList,Result).
arrays_to_lists(X,X).

arrToListWork([],[]).
arrToListWork([F | List] , [R | Result]) :-
  arrays_to_lists(F,R),       % convert the first entry if it is a list by itself
  arrToListWork(List,Result). % continue with the rest

%% lists_to_arrays(+Object, -Result).
%
% Takes a List, and converts any lists to arrays. (exacte opposite of arrays_to_lists
% (i.e. list of list of doubles becomes double[][]).
% 
lists_to_arrays([Head|Tail], Result) :-
      listToArrWork([Head|Tail], TempList),
      jpl_list_to_array(TempList,Result).
  lists_to_arrays(X,Y) :- number(X), string_to_atom(X,Y).
  lists_to_arrays(X,X) :- atom(X).

listToArrWork([F | List] , [R | Result]) :-
  lists_to_arrays(F,R), listToArrWork(List,Result).
  listToArrWork([],[]).


