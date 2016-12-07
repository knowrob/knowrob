/** <module> Prolog/OWL utility predicates

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

@author Bernhard Kirchlechner, Moritz Tenorth
@license BSD

*/

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
      arrays_to_lists/2,
      lists_to_arrays/2,
      strip_literal_type/2,
      key_to_index/3,
      extract_column/3,
      extract_values/3,
      print_info/2,
      string_tokens/2,
      %prolog_pack_path/1,
      %prolog_pack_install/2,
      lists_equal/2,
      time_term/2,
      time_between/3,
      time_between/2,
      time_later_then/2,
      time_earlier_then/2,
      current_time/1,
      property_name/2,
      property_value/2,
      path_delimiter/1,
      path_concat/3,
      path_split/2,
      mkdir/1,
      matrix_rotation/2,
      matrix_translation/2,
      matrix_translate/3,
      matrix/3
]).

current_time(T) :-
  set_prolog_flag(float_format, '%.12g'),
  get_time(T).

% Use identity if first argument is a number
time_term(Timepoint, Timepoint) :-
  number(Timepoint), !.

time_term([Begin,End], [Begin,End]) :-
  number(Begin), number(End), !.

time_term([Begin], [Begin]) :-
  number(Begin), !.

time_term(Timeinterval, Interval) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  rdf_has(Timeinterval, knowrob:'startTime', Timepoint0),
  time_term(Timepoint0, Begin),
  (  rdf_has(Timeinterval, knowrob:'endTime', Timepoint1)
  -> (time_term(Timepoint1, End), Interval=[Begin,End])
  ;  Interval=[Begin]
  ), !.

time_term(Timepoint, Time) :-
  atom(Timepoint),
  (  rdf_split_url(_, TimePointLocal, Timepoint),
     atom_concat('timepoint_', TimeAtom, TimePointLocal)
  -> term_to_atom(Time, TimeAtom)
  ;  (  atom_concat('timepoint_', TimepointAtom, Timepoint)
     -> term_to_atom(Time, TimepointAtom)
     ;  term_to_atom(Time, Timepoint)
     )
  ).

%% time_between(+T, +T0, +T1)
% True iff T0 <= T <= T1
%
time_between(Timeinterval, T0, T1) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  time_term(Timeinterval , Interval),
  time_between(Interval, T0, T1), !.

time_between([T2,T3], T0, T1) :-
  time_between(T2, T0, T1),
  time_between(T3, T0, T1), !.

time_between(T, T0, T1) :-
  not(is_list(T)), 
  time_earlier_then(T0, T),
  time_earlier_then(T, T1).

time_between(T, Timeinterval) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  time_term(Timeinterval , Interval),
  time_between(T, Interval), !.

time_between(T, [Begin,End]) :-
  time_between(T, Begin, End).

time_between(T, [Begin]) :-
  time_later_then(T, [Begin]).


%% time_later_then(+T0, +T1)
% True iff T0 >= T1
%
time_later_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_later_then_(T0_term, T1_term), !.
time_later_then_([_], [_])     :- false.
time_later_then_([T0], [_,T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [_,T1]) :- T1 =< T0, !.
time_later_then_(T0, T1) :- number(T0), number(T1), T1 =< T0, !.

%% time_earlier_then(+T0, +T1)
% True iff T0 <= T1
%
time_earlier_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_earlier_then_(T0_term, T1_term), !.
time_earlier_then_([_], [_])     :- false.
time_earlier_then_([T0], [_,T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [_,T1]) :- T0 =< T1, !.
time_earlier_then_(T0, T1) :- number(T0), number(T1), T0 =< T1, !.

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

%% ugraph_retain_transitive(+Graph, +Vertices, -NewGraph)
% Retain only the given vertices and their transitive descendants of the graph.
%
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

%% assoc_extract_value_list(+Assoc, +Keys, -Values)
% Extract the values for the corresponding keys from the association.
%
assoc_extract_value_list(_, [], []).
assoc_extract_value_list(Assoc, [Key|Keys], [Value|Values]) :-
  get_assoc(Key, Assoc, Value),
  assoc_extract_value_list(Assoc, Keys, Values).

%% jpl_call_for_each(+ObjList, +Method, +Attributes, -Values)
%
% Call a java method Method for all Instances in the ObjList with the given Attributes and return the Values.
%
jpl_call_for_each(ObjList, Method, Attributes, Values) :-
  maplist(jpl_call_for_each_helper(Method), ObjList, Attributes, Values).
jpl_call_for_each_helper(Method, Object, Attributes, Value) :-
  jpl_call(Object, Method, Attributes, Value).

%% jpl_serialize_to_file(+Object, +FileName)
%
% Serialize the given Object to a file with the given FileName.
%
jpl_serialize_to_file(Object, FileName) :-
  jpl_new('java.io.FileOutputStream', [FileName], FOS),
  jpl_new('java.io.ObjectOutputStream', [FOS], OOS),
  jpl_call(OOS, writeObject, [Object], _),
  jpl_call(OOS, 'close', [], _).

%% unifill_list(+Elem, +Num, -Result)
%
% Fill a list with Num elements Elem.
%
unifill_list(_, 0, []) :- !.
unifill_list(_, Num, _) :- Num<0, !, fail.
unifill_list(Elem, Num, [Elem|Rest]) :-
  N1 is Num - 1,
  unifill_list(Elem, N1, Rest).

%% has_to_work(+Goal, +Error).
%
% Calls the Goal and throws the Error if it fails.
%
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

%% boxed_column(?Column, ?Rows)
%
% Rows is a table with the one column Column.
%
boxed_column([],[]).
boxed_column([Column|Columns], [[Column]|Rows]) :-
  boxed_column(Columns, Rows).

%% extract_column(+Table, +ColumnNo, -Column)
%
% Extract a column from a table (list of rows).
%
extract_column([],_,[]).
extract_column([Row|Rs], CNo, [V|Vs]):-
  nth0(CNo, Row, V),
  extract_column(Rs, CNo, Vs).

%% extract_values(+Indexes, +Rows, -Values)
% Get the indexed columns of a table given as list of rows.
%
extract_values(_, [], []).
extract_values(Indexes, [Row|RRest], [Values|VRest]) :-
  extract_values_from_row(Indexes, Row, Values),
  extract_values(Indexes, RRest, VRest).
extract_values_from_row([], _, []).
extract_values_from_row([Index|IRest], Row, [Value|VRest]) :-
  nth0(Index, Row, Value),
  extract_values_from_row(IRest, Row, VRest).

%% list_to_fast_vector(+Values, -Vector)
%
% Create a weka.core.FastVector from a list.
%
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


  listToArrWork([F | List] , [R | Result]) :- lists_to_arrays(F,R), listToArrWork(List,Result).
  listToArrWork([],[]).




%% key_to_index(+Keys, +List, -Indexes)
% Get the indexes of the key elements in the list.
%
key_to_index([Key|KRest], List, [Index|IRest]) :-
  nth0(Index, List, Key),
  key_to_index(KRest, List, IRest).
key_to_index([],_,[]).



%% print_info(+Term, +Level)
% Output information obeying the silent flag
%
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
%
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
%
lists_equal([], []).
lists_equal([A|Arest],[B|Brest]) :-
  A=B,
  lists_equal(Arest, Brest).


%% min_list(+List, +Min)
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

%% max_list(+List, +Max).
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
strip_literal_type(literal(Value), Value) :- !.
strip_literal_type(Value, Value).

%% property_name(+Relation,-RelationName)
%
% Strips rdf url prefix from relation identifier.
%
property_name(Relation, RelationName) :-
  atom(Relation), rdf_split_url(_,RelationName,Relation), !.
property_name(Relation, Relation).

%% property_value(+Related,-RelationValue)
%
% Strips literal or rdf url prefix from related identifier.
%
property_value(literal(type(_,RelationValue)), RelationValue) :- !.
property_value(literal(RelationValue), RelationValue) :- !.
property_value(Related, RelationValue) :-
  atom(Related), rdf_split_url(_,RelationValue,Related), !.
property_value(Related, Related).

%% path_delimiter(?Delimiter)
%
% Delimiter for filesystem paths.
%
path_delimiter('/').

%% path_concat(+Prefix, +Suffix, ?Path)
%
% Concatenate path prefix with path suffix.
% Makes sure that one filesytem delimiter is added time_between
% @Prefix and @Suffix.
%
path_concat(Prefix, Suffix, Path) :-
  path_delimiter(Delimiter),
  (( atomic_list_concat(PrefixList, Delimiter, Prefix), last(PrefixList,'') )
  -> PrefixDelimited = Prefix
  ;  atomic_list_concat([Prefix,''], Delimiter, PrefixDelimited)
  ),
  (  atomic_list_concat([''|_], Delimiter, Suffix)
  -> atomic_list_concat(['',Suffix], Delimiter, SuffixRelative)
  ;  SuffixRelative = Suffix
  ),
  atom_concat(PrefixDelimited, SuffixRelative, Path).

%% path_split(?Path, ?PathList)
%
% Splits @Path at delimiter characters and
% unifies with splitted path elements @PathList.
%
path_split(Path, PathList) :-
  path_delimiter(Delimiter),
  atomic_list_concat(PathList, Delimiter, Path).

%prolog_pack_path(Path) :-
  %getenv('USER', User),
  %atomic_list_concat(['/home',User,lib,swipl,pack], '/', Path).

%% prolog_pack_install(+PackName, +URL)
%
% Download and install a Prolog package.
% Do nothing if pack is allready installed.
%
%prolog_pack_install(PackName, URL) :-
  %prolog_pack_path(PackPath),
  %mkdir(PackPath),
  %( pack_property(delay, version(_));
    %pack_install(PackName, [interactive(false),
      %url(URL),
      %package_directory(PackPath),
      %upgrade(true), silent(true)
    %]) ), !.
  
%% mkdir(+Dir)
%
% Create directory at @Dir if it does not yet exist.
% Also creates all not-existing parent directories.
%
mkdir(Dir) :- exists_directory(Dir), !.
mkdir(Dir) :-
  path_split(Dir, [Head|Tail]),
  atom_concat('/', Head, Prefix),
  mkdir(Prefix, Tail),
  make_directory(Dir).

mkdir(Path, [Head|Tail]) :-
  (  exists_directory(Path)
  -> true
  ;  make_directory(Path)
  ),
  path_concat(Path, Head, ChildPath),
  mkdir(ChildPath, Tail).
mkdir(_, []).


matrix_rotation(Matrix, [QW,QX,QY,QZ]) :-
  jpl_list_to_array(Matrix, MatrixArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrixToQuaternion', [MatrixArr], QuaternionArr),
  jpl_array_to_list(QuaternionArr, [QW,QX,QY,QZ]).

matrix_translation(Matrix, [X,Y,Z]) :-
  nth0( 3, Matrix, X),
  nth0( 7, Matrix, Y),
  nth0(11, Matrix, Z).

matrix(Translation, Orientation, Matrix) :-
  jpl_list_to_array(Translation, TranslationArr),
  jpl_list_to_array(Orientation, OrientationArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrix', [TranslationArr,OrientationArr], MatrixArr),
  jpl_array_to_list(MatrixArr, Matrix).

matrix_translate([M00, M01, M02, MX,
                  M10, M11, M12, MY,
                  M20, M21, M22, MZ,
                  M30, M31, M32, M33],
                 [OX,OY,OZ],
                 [M00, M01, M02, MX_,
                  M10, M11, M12, MY_,
                  M20, M21, M22, MZ_,
                  M30, M31, M32, M33]) :-
  MX_ is MX + OX,
  MY_ is MY + OY,
  MZ_ is MZ + OZ.

