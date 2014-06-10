%%
%% Copyright (C) 2009 by Bernhard Kirchlechner, Moritz Tenorth
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
:- module(util,
    [ reduce/4,
      range/3,
      ugraph_retain_transitive/3,
      zip/3,
      zipm/3,
      boxed_column/2,
      assoc_extract_value_list/3,
      jpl_call_for_each/4,
      jpl_serialize_to_file/2,
      unifill_list/3,
      has_to_work/2,
      list_to_fast_vector/2,
      fast_vector_add_list/2,
      arrays_to_lists/2,
      lists_to_arrays/2,
      strip_literal_type/2,
      key_to_index/3,
      extract_column/3,
      extract_values/3,
      extract_values_from_row/3,
      print_info/2,
      string_tokens/2,
      lists_equal/2
]).

%%
% reduce(+Predicate, +List, +StartValue, -Result).
% The predicate is first called for the first element of the list, the start value and an intermediate result.
% Then it is repeatedly called for the next elements, the intermediate result and a new intermediate result.
% The final value is the result when the predicate has been called the last time.
% E.g. reduce(plus, [1,2,3,4], 0, 10) holds.
%%
reduce(_, [], StartValue, StartValue).
reduce(Predicate, [Element|Rest], Accumulator, Result) :-
  call(Predicate, Element, Accumulator, NewAccumulator),
  reduce(Predicate, Rest, NewAccumulator, Result).

%%
% range(+M, +N, -Range)
% Generate a list of integers in a given range. E.g. range(5, 0, [5,4,3,2,1,0].
%%
range(N,N,[N]) :- !.
range(M,N,[M|Ns]) :- nonvar(N), M < N, !, M1 is M+1, range(M1, N, Ns).
range(M,N,[M|Ns]) :- nonvar(N), M > N, !, M1 is M-1, range(M1, N, Ns).

%%
% ugraph_retain_transitive(+Graph, +Vertices, -NewGraph)
% Retain only the given vertices and their transitive descendants of the graph.
%%
ugraph_retain_vertices(_, [], VertexSet, VertexSet).
ugraph_retain_vertices(Graph, [Vertex|VRest], VertexSet, Result) :-
  neighbors(Vertex, Graph, Vertices),
  union(VertexSet, Vertices, NewVertexSet),
  ugraph_retain_vertices(Graph, VRest, NewVertexSet, Result).

ugraph_retain_transitive(Graph, Vertices, NewGraph) :-
  transitive_closure(Graph, Closure),
  ugraph_retain_vertices(Closure, Vertices, Vertices, NewVertices),
  vertices(Graph, OriginalVertices),
  subtract(OriginalVertices, NewVertices, DeleteVertices),
  del_vertices(DeleteVertices, Graph, NewGraph).

%%
% zip(?List1, ?List2, ?List3)
% Zip the lists 1 and 2 together to get list3.
% It behaves like zip in python: zip([a,b],[1,2],[[a,1],[b,2]]).
%%
zip([], [], []).
zip([A|ARest], [B|BRest], [[A,B]|CRest]) :-
  zip(ARest, BRest, CRest).

%%
% zipm(?List1, ?List2, ?List3)
% Zip the lists 1 and 2 together to get list3.
% It behaves similar to zip in python but uses - for concatenation: zip([a,b],[1,2],[a-1,b-2]).
%%
zipm([], [], []).
zipm([A|ARest], [B|BRest], [A-B|CRest]) :-
  zipm(ARest, BRest, CRest).

%%
% assoc_extract_value_list(+Assoc, +Keys, -Values)
% Extract the values for the corresponding keys from the association.
%%
assoc_extract_value_list(_, [], []).
assoc_extract_value_list(Assoc, [Key|Keys], [Value|Values]) :-
  get_assoc(Key, Assoc, Value),
  assoc_extract_value_list(Assoc, Keys, Values).

%%
% jpl_call_for_each(+ObjList, +Method, +Attributes, -Values)
%
% Call a java method Method for all Instances in the ObjList with the given Attributes and return the Values.
%%
jpl_call_for_each(ObjList, Method, Attributes, Values) :-
  maplist(jpl_call_for_each_helper(Method), ObjList, Attributes, Values).
jpl_call_for_each_helper(Method, Object, Attributes, Value) :-
  jpl_call(Object, Method, Attributes, Value).

%%
% jpl_serialize_to_file(+Object, +FileName)
%
% Serialize the given Object to a file with the given FileName.
%%
jpl_serialize_to_file(Object, FileName) :-
  jpl_new('java.io.FileOutputStream', [FileName], FOS),
  jpl_new('java.io.ObjectOutputStream', [FOS], OOS),
  jpl_call(OOS, writeObject, [Object], _),
  jpl_call(OOS, 'close', [], _).

%%
% unifill_list(+Elem, +Num, -Result)
%
% Fill a list with Num elements Elem.
%%
unifill_list(_, 0, []) :- !.
unifill_list(_, Num, _) :- Num<0, !, fail.
unifill_list(Elem, Num, [Elem|Rest]) :-
  N1 is Num - 1,
  unifill_list(Elem, N1, Rest).

%%
% has_to_work(+Goal, +Error).
%
% Calls the Goal and throws the Error if it fails.
%%
has_to_work(Goal, Error) :-
  call(Goal)
  -> true
  ; prolog_current_frame(Frame),
  prolog_frame_attribute(Frame, parent, ParentFrame),
  prolog_frame_attribute(ParentFrame, goal, FrameGoal),
  functor(FrameGoal, Functor0, Arity0),
  % If the fuctor is only namespace prefixing go one further
  ( Functor0 = ':'
  -> FrameGoal =.. [':', FrameNS, FramePred],
    functor(FramePred, Functor1, Arity),
    Functor = FrameNS:Functor1
  ; Functor = Functor0, Arity = Arity0 ),
%  term_to_atom(FrameGoal, FG),
%  concat_atom([Error, ' in ', FG], ErrorMessage),
  throw(error(failure_error(Goal), context(Functor/Arity, Error))).

%%
% boxed_column(?Column, ?Rows)
%
% Rows is a table with the one column Column.
%%
boxed_column([],[]).
boxed_column([Column|Columns], [[Column]|Rows]) :-
  boxed_column(Columns, Rows).

%%
% extract_column(+Table, +ColumnNo, -Column)
%
% Extract a column from a table (list of rows).
%%
extract_column([],_,[]).
extract_column([Row|Rs], CNo, [V|Vs]):-
  nth0(CNo, Row, V),
  extract_column(Rs, CNo, Vs).

%%
% extract_values(+Indexes, +Rows, -Values)
% Get the indexed columns of a table given as list of rows.
%%
extract_values(_, [], []).
extract_values(Indexes, [Row|RRest], [Values|VRest]) :-
  extract_values_from_row(Indexes, Row, Values),
  extract_values(Indexes, RRest, VRest).
extract_values_from_row([], _, []).
extract_values_from_row([Index|IRest], Row, [Value|VRest]) :-
  nth0(Index, Row, Value),
  extract_values_from_row(IRest, Row, VRest).

%%
% list_to_fast_vector(+Values, -Vector)
%
% Create a weka.core.FastVector from a list.
%%
list_to_fast_vector(Values, Vector) :-
  jpl_new(class([weka, core], ['FastVector']), [], Vector),
  list_to_fast_vector_1(Values, Vector).
list_to_fast_vector_1([], _).
list_to_fast_vector_1([V|Vs], Vector) :-
  jpl_call(Vector, 'addElement', [V], _),
  list_to_fast_vector_1(Vs, Vector).
fast_vector_add_list(_, []).
fast_vector_add_list(Vector, [Elem|Rest]) :-
  jpl_call(Vector, 'addElement', [Elem], _),
  fast_vector_add_list(Vector, Rest).



% arrays_to_lists(+Object, -Result)
%
% Takes a java Object, and converts any arrays to lists.
% (i.e. double[][] becomes list of list of doubles).

arrays_to_lists(Object, Result) :-
  jpl_object_to_type(Object,array(_)), !,
  jpl_array_to_list(Object, TempList),
  arrToListWork(TempList,Result).
arrays_to_lists(X,X).

arrToListWork([],[]).
arrToListWork([F | List] , [R | Result]) :-
  arrays_to_lists(F,R),       % convert the first entry if it is a list by itself
  arrToListWork(List,Result). % continue with the rest



% lists_to_arrays(+Object, -Result)
%
% Takes a List, and converts any lists to arrays. (exacte opposite of arrays_to_lists
% (i.e. list of list of doubles becomes double[][]).
lists_to_arrays([Head|Tail], Result) :-
      listToArrWork([Head|Tail], TempList),
      jpl_list_to_array(TempList,Result).
  lists_to_arrays(X,Y) :- number(X), string_to_atom(X,Y).
  lists_to_arrays(X,X) :- atom(X).


  listToArrWork([F | List] , [R | Result]) :- lists_to_arrays(F,R), listToArrWork(List,Result).
  listToArrWork([],[]).




%%
% key_to_index(+Keys, +List, -Indexes)
% Get the indexes of the key elements in the list.
%%
key_to_index([Key|KRest], List, [Index|IRest]) :-
  nth0(Index, List, Key),
  key_to_index(KRest, List, IRest).
key_to_index([],_,[]).



%%
% print_info(+Term, +Level)
% Output information obeying the silent flag
%%

print_info(_, silent) :- !.
print_info(_, informational) :-
        current_prolog_flag(verbose, silent), !.
print_info(_, banner) :-
        current_prolog_flag(verbose, silent), !.
print_info(Term, _) :-
        flush_output(user_output),
        print(Term).


%% string_tokens(+Cs, -Ts)
%
% util predicate for splitting point and trajectory identifiers, split string at token '_'
%%

string_tokens(Cs, Ts) :- phrase(tokens(Cs, []), Ts).
tokens([], Ts) --> token(Ts).
tokens([C|Cs], Ts) -->
    ( { C == 95 } -> token(Ts), tokens(Cs, [])
    ; tokens(Cs, [C|Ts])
    ).
token([]) -->
    [].
token([T|Ts]) -->
    { reverse([T|Ts], Token) },
    [Token].


%% lists_equal(+A, +B)
%
% compare lists element-wise
%%
lists_equal([], []).
lists_equal([A|Arest],[B|Brest]) :-
  A=B,
  lists_equal(Arest, Brest).


%%
% min_list(+List, +Min)
%
% re-implementation of lists:min_list  that is not available in some versions
% not exported to avoid conflicts (use util:min_list to refer to this implementation)
%
min_list([], _).
min_list([A], A) :- !.
min_list([A|Arest],Min) :-
  min_list1(Arest, A, Min),!.

min_list1([], Min, Min).
min_list1([A|Arest], OldMin ,Min) :-
  (A<OldMin) -> (min_list1(Arest, A, Min));(min_list1(Arest, OldMin, Min)).

%%
% max_list(+List, +max)
%
% re-implementation of lists:max_list  that is not available in some versions
% not exported to avoid conflicts (use util:max_list to refer to this implementation)
%
max_list([], _).
max_list([A], A) :- !.
max_list([A|Arest], Max) :-
  max_list1(Arest, A, Max),!.

max_list1([], Max ,Max).
max_list1([A|Arest], OldMax ,Max) :-
  (A>OldMax) -> (max_list1(Arest, A, Max));(max_list1(Arest, OldMax, Max)).


%% strip_literal_type(+Input,-Output)
%
% Strip the literal(type(..., Value)) and return value if present, else return the original.
%
strip_literal_type(literal(type(_, Value)), Value) :- !.
strip_literal_type(Value, Value).