:- begin_tests('tripledb').

:- use_module(library('db/tripledb'),
    [ tripledb_load/2,
      tripledb_ask/3,
      tripledb_forget/3,
      tripledb_tell/3
    ]).

% load swrl owl file for tripledb testing
test('tripledb_load_local_owl_file(URL)') :-
    tripledb_load(
        'package://knowrob/owl/test/swrl.owl',
        [ graph(common),
          namespace(knowrob, 'http://knowrob.org/kb/swrl_test#')
        ]).

% check via tripledb_ask if individual triple exists
test('tripledb_ask_if_individual_triple_exists_in_triplestore(S,P,O)') :-
    tripledb_ask(
        knowrob:'Adult',
        rdfs:'subClassOf',
        knowrob:'TestThing').

% delete individual triple
test('tripledb_forget_from_triplestore(S,P,O)') :-
    tripledb_forget(
        knowrob:'Adult',
        rdfs:'subClassOf',
        knowrob:'TestThing').

% check again for that triple and it should not be in the tripledb
test('tripledb_ask_after_deletion', [fail]) :-
    tripledb_ask(
        knowrob:'Adult',
        rdfs:'subClassOf',
        knowrob:'TestThing').

% add triple
test('tripledb_tell_to_triplestore(S,P,O)') :-
    tripledb_tell(
        knowrob:'Adult',
        rdfs:'subClassOf',
        knowrob:'TestThing').


% check again for that triple and it should be in the tripledb
test('tripledb_ask_after_injection_of_new_triple') :-
    tripledb_ask(
        knowrob:'Adult',
        rdfs:'subClassOf',
        knowrob:'TestThing').

%%%%%%%%%%%%%%%%%%%%%%%% tests for xsd datatypes  %%%%%%%%%%%%%%%%%%%%%%%%
% load test owl file for tripledb testing for xsd datatypes
test('tripledb_load_local_owl_file_for_xsd_datatypes(URL)') :-
    tripledb_load(
        'package://knowrob/owl/test/datatype_test.owl',
        [ graph(common)
        ]).

% test for xsd:integer, check first if triple exists and then check if the datatype is prolog number?
% check for studentIds which should be integer datatype
test('tripledb_ask_for_each_studentId_if_integer') :-
    forall(tripledb_ask(
        X,
        'http://knowrob.org/kb/datatype_test#studentId',
        Y),
        integer(Y)).


% test for xsd:string, check first if triple exists and then check if the datatype is prolog atom?
% check for studentsFirstNames which should be atom/string datatype
test('tripledb_ask_for_each_studentFirstName_if_string') :-
    forall(tripledb_ask(
        X,
        'http://knowrob.org/kb/datatype_test#first_name',
        Y),
        atom(Y)).

:- end_tests('tripledb').
