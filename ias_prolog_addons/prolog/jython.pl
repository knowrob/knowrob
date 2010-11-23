%%
%% Copyright (C) 2009 by Bernhard Kirchlechner
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
:- module(jython,
    [ jython_init/2,
      jython_call/4,	% Call a jython method
      jython_exec_script/4,
      jython_variable/3,
      jython_to_prolog/2
    ]).
:- use_module(library(jpl)).

%% jython_init(+Path, -Interpreter)
%%
jython_init(Path, Interpreter) :-
  nonvar(Path),
  var(Interpreter),
  jpl_new('java.util.Properties', [], Properties),
  jpl_call(Properties, setProperty, ['python.path',Path],_),
  jpl_call('java.lang.System', getProperties, [], SysProps),
  jpl_list_to_array([''], CommandLine),
  jpl_call('org.python.util.PythonInterpreter', initialize, [SysProps, Properties, CommandLine], _),
  jpl_new('org.python.util.PythonInterpreter', [], Interpreter).

%% jython_call(+Interpreter, +Command, +ResultNames, -Results).
%
% Execute a jython command.
%
% Interpreter: The jython interpreter (see jython_init).
% Command:     The jython command to execute.
% ResultNames: The names of the return variables.
% Results:     The variables values.
%%
jython_call(Interpreter, Command, ResultNames, Results) :-
  nonvar(Interpreter),
  nonvar(Command),
  jpl_call(Interpreter, exec, [Command], _),
  (nonvar(ResultNames)
  -> jython_getResults(Interpreter, ResultNames, Results)
  ; Results = 'No results requested!').


%% jython_variable(+Interpreter, ?Key, ?Value).
%
% Set/get a variable in the jython interpreter.
%%
jython_variable(Interpreter, Key, Value) :-
  nonvar(Interpreter),
  nonvar(Key),!,
  (nonvar(Value)
  -> (is_list(Value)
    -> jpl_datums_to_array(Value, ValArray),
      jpl_call(Interpreter, set, [Key, ValArray], _)
%     ; jpl_box_datum(Value, BValue),
%       jpl_call(Interpreter, set, [Key, BValue], _)
    )
  ;  jpl_call(Interpreter, get, [Key], Value)).
jython_variable(_,_,_) :-
  throw(error(instantiation_error, _)).

%% jython_to_prolog(JythonValue, PrologValue).
%
% Convert the value represented in a Jython class to prolog.
%%
jython_to_prolog(JythonValue, Value) :-
  jpl_is_ref(JythonValue),
  jpl_object_to_type(JythonValue, Type),
  jpl:jpl_type_fits_type(Type, class([org, python, core], ['PyObject'])),
  (
    jpl:jpl_type_fits_type(Type, class([org, python, core], ['PyString']))
    -> jpl_call(JythonValue, 'toString', [], V)
    ; jpl_call(JythonValue, 'isMappingType', [], @(true))
    -> jython_map_to_prolog(JythonValue, V)
    ; jpl_call(JythonValue, 'isNumberType', [], @(true))
    -> jython_number_to_prolog(JythonValue, V)
    ; jpl_call(JythonValue, 'isCallable', [], @(true))
    -> V = JythonValue
    ; jpl_call(JythonValue, 'isSequenceType', [], @(true))
    -> jpl_call(JythonValue, '__len__', [], SeqLength),
      jython_sequence_to_list(0, SeqLength, JythonValue, List),
      java_jpl_list_to_prolog(List, V)
    ; jpl_call(JythonValue, 'safeRep', [], V)
  ),
  Value = V, !
  ; java_to_prolog(JythonValue, Value).

jython_sequence_to_list(N, Max, _, []) :-
  N >= Max, !.
jython_sequence_to_list(N, Max, Sequence, [Elem|Rest]) :-
  jpl_call(Sequence, '__getitem__', [N], E),
  Elem = E,
  N1 is N+1,
  jython_sequence_to_list(N1, Max, Sequence, Rest).

jython_map_to_prolog(JythonMap, Map) :-
  jpl_call(JythonMap, 'keys', [], Keys),
  jpl_call(Keys, '__len__', [], Len),
  jython_sequence_to_list(0, Len, Keys, KeyList),
  jython_map_to_prolog_1(KeyList, JythonMap, Map).
jython_map_to_prolog_1([], _, []).
jython_map_to_prolog_1([K|KRest], JythonMap, [PK-PV|PRest]) :-
  jpl_call(JythonMap, '__getitem__', [K], V),
  jython_to_prolog(K, PK),
  jython_to_prolog(V, PV),
  jython_map_to_prolog_1(KRest, JythonMap, PRest).

% TODO: Handle PyComplex and others without getValue methods
jython_number_to_prolog(JythonNumber, Value) :-
  (
    catch(jpl_call(JythonNumber, getValue, [], V), _Error, fail) % If we have a getValue method, use it
    -> true
    ; jpl_object_to_type(JythonNumber, Type),
      print(['jython_number_to_prolog: ',Type,' does not have a getValue Method!']),
      jpl_call(JythonNumber, '__float__', [], Float),
      jpl_call(Float, getValue, [], V)
  ),
  V = Value.

java_to_prolog(JavaValue, _) :-
  var(JavaValue),
  throw(error(instantiation_error, _)).
java_to_prolog(JavaValue, PrologValue) :- % a ref
  jpl_is_ref(JavaValue), !,
  (
    (jpl_is_null(JavaValue)
        ; jpl_is_void(JavaValue))
    -> PValue = []
    ; jpl_object_to_type(JavaValue, Type),
    (Type = array(_)
    -> java_array_to_prolog(JavaValue, PValue)
    ; jpl:jpl_type_fits_type(Type, class([java,lang], ['Iterable']))
    -> jpl_call(JavaValue, iterator, [], Iterator),
      java_iterator_to_list(Iterator, IteratorList),
      java_jpl_list_to_prolog(IteratorList, PValue)
    ; jpl:jpl_type_fits_type(Type, class([java,util], ['Map']))
    -> java_map_to_prolog(JavaValue, PValue)
    ; jpl:jpl_type_fits_type(Type, class([java, lang], ['Number']))
    -> java_number_to_prolog(JavaValue, PValue)
    ; jpl:jpl_call(JavaValue, 'toString', [], PValue))
  ),
  PrologValue = PValue.
java_to_prolog(JavaValue, PrologValue) :- % a value
  PrologValue = JavaValue.  % will this work???

java_number_to_prolog(Number, Value) :-
  jpl_object_to_type(Number, Type),
  (
    Type = class([java,lang], ['Byte'])
    -> jpl:jpl_call(Number, 'byteValue', [], V)
    ; Type = class([java,lang], ['Short'])
    -> jpl:jpl_call(Number, 'shortValue', [], V)
    ; Type = class([java,lang], ['Integer'])
    -> jpl:jpl_call(Number, 'intValue', [], V)
    ; Type = class([java,lang], ['Long'])
    -> jpl:jpl_call(Number, 'longValue', [], V)
    ; Type = class([java,lang], ['Float'])
    -> jpl:jpl_call(Number, 'floatValue', [], V)
%    ; Type = class([java,lang], ['Double'])
%    -> jpl:jpl_call(Number, 'doubleValue', [], V)
    ; jpl:jpl_call(Number, 'doubleValue', [], V)  % For all others (also Double) we use the most variable one.
  ),
  Value = V.

java_jpl_list_to_prolog([],[]).
java_jpl_list_to_prolog([J|JRest],[L|LRest]) :-
  jython_to_prolog(J,L),
  java_jpl_list_to_prolog(JRest,LRest).
java_array_to_prolog(JavaArray, PrologList):-
  jpl_array_to_list(JavaArray, List),
  java_jpl_list_to_prolog(List, PrologList).

java_iterator_to_list(Iterator, [A|Rest]):-
  jpl_iterator_element(Iterator, A), !,
  java_iterator_to_list(Iterator, Rest).
java_iterator_to_list(_, []).

java_entries_to_prolog([],[]):- !.
java_entries_to_prolog([K-V|ERest], [PK-PV|PRest]) :-
  jython_to_prolog(K,PK),
  jython_to_prolog(V,PV),
  java_entries_to_prolog(ERest, PRest).
java_map_to_prolog(Map, PrologList) :-
  findall(Entry, jpl_map_element(Map, Entry), Entries),
  java_entries_to_prolog(Entries, PrologList).

jython_getResults(_, [], []).
jython_getResults(Interpreter, [VarName| RestNames], [Value| RestValues]) :-
  nonvar(VarName),
  jpl_call(Interpreter, get, [VarName], Value),
  jython_getResults(Interpreter, RestNames, RestValues).

%% jython_exec_script(+Interpreter, +FileName, +ResultNames, -Results)
%
% Execute a jython script.
%
% Interpreter: The jython interpreter (see jython_init).
% FileName:    The file name of the script. If it starts with http:// an URLConnection is performed.
% ResultNames: The names of the return variables.
% Results:     The variables values.
%%
jython_exec_script(Interpreter, FileName, ResultNames, Results) :-
  nonvar(Interpreter),
  nonvar(FileName),
  nonvar(ResultNames),
  (string_concat('http://',_,FileName)
  -> jpl_new('java.net.URL', [FileName], Url),
     jpl_call(Url, openStream, [], File)
  ; jpl_new('java.io.FileInputStream', [FileName], File)),
  jpl_call(Interpreter, execfile, [File], _),
  jython_getResults(Interpreter, ResultNames, Results).

jython_set_function_parameters(Interpreter, [Parameter| Parameters], ParameterStr, N) :-
  concat_atom(['myInnerParameter', N], ParameterName),
  jython_variable(Interpreter, ParameterName, Parameter),
  N1 is N + 1,
  jython_set_function_parameters(Interpreter, Parameters, ParameterNames, N1),
  concat_atom([ParameterName, ', ', ParameterNames], ParameterStr).

jython_set_function_parameters(_, [], '', _).


jython_call_function(Interpreter, ModuleName, FunctionName, Parameters, ResultNames, Results) :-
  jython_set_function_parameters(Interpreter, Parameters, ParameterStr, 0),
  (concat_atom([ParamStr, ', '], ParameterStr)
  -> ParameterString=ParamStr
  ; ParameterString=ParameterStr),
  concat_atom(['from ',ModuleName,' import ',FunctionName,'\n',FunctionName, '(', ParameterString,')'],Command),
  jython_call(Interpreter, Command, ResultNames, Results).