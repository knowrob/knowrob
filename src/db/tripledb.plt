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

% test for xsd:integer, Str, float
test('tripledb_tell_for_XSD_DataTypes') :-
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'first_name', 'Johana$'),
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'last_name', 'Muller'),
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'studentId', 212123),
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'height', 5.10).

% test for xsd:integer, Str, float
test('tripledb_ask_for_XSD_DataTypes') :-
    forall(tripledb_ask(_, test_datatype:'studentId', X), integer(X)),
    forall(tripledb_ask(_, test_datatype:'first_name', Y), atom(Y)),
    forall(tripledb_ask(_, test_datatype:'last_name', Z), atom(Z)),
    forall(tripledb_ask(_, test_datatype:'height', H), float(H)).

% remove the triple at the end of test
test('tripledb_forget_for_XSD_DataTypes') :-
    tripledb_forget(test_datatype:'Lecturer3', _, _).

:- end_tests('tripledb').

