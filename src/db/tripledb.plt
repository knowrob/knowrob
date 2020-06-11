:- begin_tests('tripledb').

:- use_module(library('db/tripledb'),
    [ tripledb_load/2,
      tripledb_ask/3,
      tripledb_forget/3,
      tripledb_tell/3
    ]).

:- use_module(library('semweb/rdf_db'),  [ rdf_register_ns/3 ]).

% register namespace for following tests
:- rdf_register_ns(swrl_tests,'http://knowrob.org/kb/swrl_test#',[keep(true)]).

% load swrl owl file for tripledb testing
test('tripledb_load_local_owl_file(URL)') :-
    tripledb_load(
        'package://knowrob/owl/test/swrl.owl',
        [ graph(common)
        ]).

% check via tripledb_ask if individual triple exists
test('tripledb_ask_if_individual_triple_exists_in_triplestore(S,P,O)') :-
    tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing').

% delete individual triple
test('tripledb_forget_from_triplestore(S,P,O)') :-
    tripledb_forget(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing').

% check again for that triple and it should not be in the tripledb
test('tripledb_ask_after_deletion', [fail]) :-
    tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing').

% add triple and check if it exists in db
test('tripledb_tell_to_triplestore_and_check_if_it_exists(S,P,O)') :-
    tripledb_tell(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing'),
    tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing').


%%%%%%%%%%%%%%%%%%%%%%%% tests for xsd datatypes  %%%%%%%%%%%%%%%%%%%%%%%%
% register namespace for following tests
:- rdf_register_ns(test_datatype, 'http://knowrob.org/kb/datatype_test#',[keep(true)]).

% load test owl file for tripledb testing for xsd datatypes
test('tripledb_load_local_owl_file_for_xsd_datatypes(URL)') :-
    tripledb_load(
        'package://knowrob/owl/test/datatype_test.owl',
        [ graph(user)
        ]).

% test for xsd:integer, check first if triple exists and then check if the datatype is prolog number?
% check for studentIds which should be integer datatype
test('tripledb_ask_for_each_studentId_if_integer') :-
    forall(tripledb_ask(X, test_datatype:'studentId', Y), integer(Y)).

% test for xsd:string, check first if triple exists and then check if the datatype is prolog atom?
% check for studentsFirstNames which should be atom/string datatype
test('tripledb_ask_for_each_studentFirstName_if_string') :-
    forall(tripledb_ask(X, test_datatype:'first_name', Y), atom(Y)).

% test for xsd:string, add triple with lecturer first name
% check for lecturerFirstNames which should be atom/string datatype
% remove the triple at the end of test
test('tripledb_tell_for_lecurerFirstName_as_string') :-
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'first_name', 'Johana'),
    tripledb_ask(test_datatype:'Lecturer3', test_datatype:'first_name', Y),
    atom(Y),
    tripledb_forget(test_datatype:'Lecturer3', test_datatype:'first_name', 'Johana').

:- end_tests('tripledb').
