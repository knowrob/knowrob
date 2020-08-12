
:- use_module(library('db/tripledb_tests')).
:- begin_tests(
		'tripledb',
		[   setup(tripledb:set_default_graph(test)),
		    cleanup(tripledb_tests:tripledb_cleanup)
		]).

:- use_module(library('db/tripledb')).

:- use_module(library('lang/scopes/temporal')).

:- use_module(library('lang/terms/holds')).

:- use_module(library('rostest.pl')).

:- use_module(library('semweb/rdf_db'),  [ rdf_register_ns/3 ]).

% register namespace for following tests
:- rdf_register_ns(swrl_tests,'http://knowrob.org/kb/swrl_test#',[keep(true)]).

% load swrl owl file for tripledb testing
test('tripledb tests: load local owl file') :-
    tripledb_load(
        'package://knowrob/owl/test/swrl.owl',
        [ graph(test)
        ]).

% check via tripledb_ask if individual triple exists
test('tripledb tests: ask triple exists in triplestore') :-
    assert_true(tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing')).

% delete individual triple
test('tripledb tests: forget triple') :-
    assert_true(tripledb_forget(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing')).

% check again for that triple and it should not be in the tripledb
test('tripledb tests: ask after deletion', [fail]) :-
    tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing').

% add triple and check if it exists in db
test('tripledb tests: tell to triplestore and check if triple exists') :-
    assert_true(tripledb_tell(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing')),
    assert_true(tripledb_ask(swrl_tests:'Adult', rdfs:'subClassOf', swrl_tests:'TestThing')).


%%%%%%%%%%%%%%%%%%%%%%%% tests for xsd datatypes  %%%%%%%%%%%%%%%%%%%%%%%%
% register namespace for following tests
:- rdf_register_ns(test_datatype, 'http://knowrob.org/kb/datatype_test#',[keep(true)]).

% load test owl file for tripledb testing for xsd datatypes
test('tripledb tests: load local owl file for xsd datatypes') :-
    tripledb_load(
        'package://knowrob/owl/test/datatype_test.owl',
        [ graph(test)
        ]).

% test for xsd:integer, Str, float
test('tripledb tests: tell triples with various XSD DataTypes') :-
    assert_true(tripledb_tell(test_datatype:'Lecturer3', test_datatype:'first_name', 'Johana$')),
    assert_true(tripledb_tell(test_datatype:'Lecturer3', test_datatype:'last_name', 'Muller')),
    assert_true(tripledb_tell(test_datatype:'Lecturer3', test_datatype:'studentId', 212123)),
    assert_true(tripledb_tell(test_datatype:'Lecturer3', test_datatype:'height', 5.10)).

% test for xsd:integer, Str, float
test('tripledb tests: ask triples with various XSD DataTypes') :-
    assert_true(forall(tripledb_ask(_, test_datatype:'studentId', X), number(X))),
    assert_true(forall(tripledb_ask(_, test_datatype:'first_name', Y), atom(Y))),
    assert_true(forall(tripledb_ask(_, test_datatype:'last_name', Z), atom(Z))),
    assert_true(forall(tripledb_ask(_, test_datatype:'height', H), float(H))).


% test for special characters
%test('tripledb_tell_special_character_umlaut', [ fixme('fix encoding for special characters e.g. umlaut') ]):-
%	tripledb_tell(test_datatype:'Lecturer3', test_datatype:'last_name', 'Müller').

% test for list as an argument
test('tripledb_tell_list_as_an_argument') :-
	DataTerm=[255,99,71],
	assert_true(tripledb_tell(test_datatype:'Lecturer3',
		test_datatype:'hasHairColor', term(DataTerm))),
	assert_true(tripledb_ask(test_datatype:'Lecturer3',
		test_datatype:'hasHairColor', term(DataTerm))),
	tripledb_ask(test_datatype:'Lecturer3',
		test_datatype:'hasHairColor', term(Actual)),
	assert_true(is_list(Actual)),
	assert_equals(Actual,DataTerm).

% remove the triple at the end of test
test('tripledb tests: forget triple with various XSD DataTypes') :-
    assert_true(tripledb_forget(test_datatype:'Lecturer3', _, _)).

% test for time scope
test('tripledb tell triple with time scope'):-
    time_scope(=(5), =(10), T_S1),
    tripledb_tell(test_datatype:'Lecturer4', test_datatype:'last_name', 'Spiendler', T_S1),
    assert_true(tripledb_ask(test_datatype:'Lecturer4', test_datatype:'last_name', 'Spiendler', T_S1, _, _)),
    time_scope(=(5), =(20), T_S2),
    assert_false(tripledb_ask(test_datatype:'Lecturer4', test_datatype:'last_name', 'Spiendler', T_S2, _, _)).

% test for time scope extension
test('tripledb tell triple with time scope extension'):-
    time_scope(=(5), =(20), T_S1),
    tripledb_tell(test_datatype:'Lecturer4', test_datatype:'last_name', 'Spiendler', T_S1),
    assert_true(tripledb_ask(test_datatype:'Lecturer4', test_datatype:'last_name', 'Spiendler', T_S1, _, _)).

% test for unit
test('tripledb tell triple with Unit'):-
    assert_false(tripledb_ask(test_datatype:'Lecturer4', test_datatype:'height', unit(double(2.1),'meter'))),
    assert_true(tripledb_tell(test_datatype:'Lecturer4', test_datatype:'height', unit(double(2.1),'meter'))),
    assert_true(tripledb_ask(test_datatype:'Lecturer4', test_datatype:'height', unit(double(2.1),'meter'))).

% test for special characters in iri: @*~!#?
test('tripledb_tell_special_characters_@*~!#?'):-
    assert_true(tripledb_tell(test_datatype:'normal_user_test_new', test_datatype:'last@*~!#?_name', 'umlaut')),
    assert_true(tripledb_ask(test_datatype:'normal_user_test_new', test_datatype:'last@*~!#?_name', 'umlaut')).

% it is observed that once these special characters are stored in db(with wrong format), they can not be retrieved normally in variables or any other way.
% Hence, in the code we throw a warning at triple_query_unify_o1
test('tripledb ask for triple with special character already stored in db(wrong/corrupt format)',
    [ fixme('fix encoding for special characters e.g. umlaut') ]):-
    tripledb_tell(test_datatype:'Lecturer3', test_datatype:'last_name', 'Müller'),
    assert_true(tripledb_ask(test_datatype:'Lecturer3', test_datatype:'last_name', _)).

% test for non existent triples
test('tripledb_ask_for_non_existant_triples'):-
    assert_false(tripledb_ask(test_datatype:'xyz', test_datatype:'last_name', _)).

:- end_tests('tripledb').

