
:- use_module(library('unittest')).
:- use_module('mongolog_test').

:- begin_tests(
		'mongolog_triple',
		[   setup(sw_set_default_graph(test)),
		    cleanup(mongolog_test:cleanup)
		]).

:- use_module(library('semweb/rdf_db'),
		[ rdf_global_term/2 ]).

:- use_module(library('mongodb/client')).
:- use_module(library('scope')).
:- use_module(library('semweb')).
:- use_module('mongolog').
:- use_module('triple').

% register namespaces for following tests
:- sw_register_prefix(swrl_tests,    'http://knowrob.org/kb/swrl_test#').
:- sw_register_prefix(test_datatype, 'http://knowrob.org/kb/datatype_test#').

test('assert triple(a,b,c)') :-
	assert_true(mongolog_assert(triple(a,b,c))).

test('assert triple(a,b,_)', [throws(error(instantiation_error,assert(triple(a,b,_))))]) :-
	mongolog_assert(triple(a,b,_)).

test('triple(a,b,c)') :-
	assert_true(mongolog_call(triple(a,b,c))),
	assert_false(mongolog_call(triple(x,b,c))),
	assert_false(mongolog_call(triple(a,x,c))),
	assert_false(mongolog_call(triple(a,b,x))).

test('triple(A,b,c)') :-
	mongolog_call(triple(A,b,c)),
	assert_equals(A,a),
	assert_false(mongolog_call(triple(_,x,c))).

test('triple(a,B,c)') :-
	mongolog_call(triple(a,B,c)),
	assert_equals(B,b),
	assert_false(mongolog_call(triple(x,_,c))).

test('triple(a,b,C)') :-
	mongolog_call(triple(a,b,C)),
	assert_equals(C,c),
	assert_false(mongolog_call(triple(a,x,_))).

test('triple(A,b,C)') :-
	mongolog_call(triple(A,b,C)),
	assert_equals(A,a),
	assert_equals(C,c),
	assert_false(mongolog_call(triple(_,x,_))).

% load swrl owl file for tripledb testing
test('load local owl file') :-
	assert_true(mongolog_call(load_rdf_xml('owl/test/swrl.owl', test))),
	assert_true(mongolog_call(load_rdf_xml('owl/test/datatype_test.owl',test))).

% check via tripledb_ask if individual triple exists
test('query triple') :-
	assert_true(mongolog_call(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))).

% delete individual triple
test('retract triple') :-
	assert_true(mongolog_retract(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))),
	assert_false(mongolog_call(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))).

% add triple and check if it exists in db
test('assert to triplestore and check if triple exists') :-
	assert_true(mongolog_assert(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))),
	assert_true(mongolog_call(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))),
	assert_false(mongolog_call(triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'Car'
	))).

% tests for xsd:integer, Str, float
test('assert XSD') :-
	rdf_global_term(test_datatype:'Lecturer3',S),
	assert_true(mongolog_assert(triple(S, test_datatype:'first_name', 'Johana$'))),
	assert_true(mongolog_assert(triple(S, test_datatype:'last_name',  'Muller'))),
	assert_true(mongolog_assert(triple(S, test_datatype:'studentId',  212123))),
	assert_true(mongolog_assert(triple(S, test_datatype:'height',     5.10))).

% tests for xsd:integer, Str, float
test('query XSD') :-
	assert_true(forall(mongolog_call(triple(_, test_datatype:'studentId',  X)), number(X))),
	assert_true(forall(mongolog_call(triple(_, test_datatype:'first_name', Y)), atom(Y))),
	assert_true(forall(mongolog_call(triple(_, test_datatype:'last_name',  Z)), atom(Z))),
	assert_true(forall(mongolog_call(triple(_, test_datatype:'height',     H)), float(H))).

% tests for list as an argument
test('assert list') :-
	rdf_global_term(test_datatype:'Lecturer3',S),
	DataTerm=[255,99,71],
	% tests asserting list value
	assert_true(mongolog_assert(triple(S, test_datatype:'hasHairColor', term(DataTerm)))),
	% tests with ground value
	assert_true(mongolog_call(triple(S, test_datatype:'hasHairColor', term(DataTerm)))),
	% tests with var value
	(	mongolog_call(triple(S, test_datatype:'hasHairColor', term(Actual)))
	->	assert_equals(Actual,DataTerm)
	;	true
	).

% tests for time scope
test('assert with scope'):-
	rdf_global_term(test_datatype:'Lecturer4',S),
	rdf_global_term(test_datatype:'last_name',P),
	time_scope(=(double(5)), =(double(10)), T_S1),
	time_scope(=(double(5)), =(double(20)), T_S2),
	mongolog_assert(triple(S, P, 'Spiendler'), [query_scope(T_S1)]),
	assert_true(mongolog_call(triple(S, P, 'Spiendler'), [query_scope(T_S1)])),
	assert_false(mongolog_call(triple(S, P, 'Spiendler'), [query_scope(T_S2)])).

% tests for time scope extension
test('extend time scope'):-
	rdf_global_term(test_datatype:'Lecturer4',S),
	rdf_global_term(test_datatype:'last_name',P),
	time_scope(=(double(10)), =(double(20)), T_S1),
	time_scope(=(double(5)),  =(double(20)), T_S2),
	mongolog_assert(triple(S, P, 'Spiendler'), [query_scope(T_S1)]),
	assert_true(mongolog_call(triple(S, P, 'Spiendler'), [query_scope(T_S2)])).

test('query value operators') :-
	assert_true(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', =(6)))),
	assert_true(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', =<(9)))),
	assert_true(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', <(7)))),
	assert_true(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', >=(5)))),
	assert_true(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', >(3.5)))),
	assert_false(mongolog_call(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', <(3)))).

test('query operator in'):-
	findall(LastName,
		mongolog_call(triple(
			in(array([
				string(test_datatype:'Lecturer3'),
				string(test_datatype:'Lecturer4')
			])),
			test_datatype:'last_name',
			LastName)),
		Names
	),
	assert_equals(Names,['Muller']).

test('query operator in + ->'):-
	findall([Lecturer,LastName],
		mongolog_call(triple(
			in(array([
				string(test_datatype:'Lecturer3'),
				string(test_datatype:'Lecturer4')
			])) -> Lecturer,
			test_datatype:'last_name',
			LastName)),
		LecturerList
	),
	assert_equals(LecturerList,
		[[test_datatype:'Lecturer3', 'Muller']]).

% tests for special characters in iri: @*~!#?
test('non alphabetic character'):-
	assert_true(mongolog_assert(triple(
		test_datatype:'normal_user_test_new',
		test_datatype:'last@*~!#?_name',
		'umlaut'
	))),
	assert_true(mongolog_call(triple(
		test_datatype:'normal_user_test_new',
		test_datatype:'last@*~!#?_name',
		'umlaut'
	))).

test('non utf8 character', fixme('bson_pl has issues reading non-utf8')):-
	mongolog_assert(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		'Müller'
	)),
	assert_true(mongolog_call(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		'Müller'
	))).

% tests for non existent triples
test('non existant'):-
	assert_false(mongolog_call(
		triple(test_datatype:'xyz', test_datatype:'last_name', _)
	)).

test('triple(+,transitive(+),+') :-
	assert_true(mongolog_call(triple(
		swrl_tests:'Rex',
		transitive(swrl_tests:isParentOf),
		swrl_tests:'Ernest'))),
	assert_true(mongolog_call(triple(
		swrl_tests:'Rex',
		transitive(swrl_tests:isParentOf),
		swrl_tests:'Lea'))),
	assert_false(mongolog_call(triple(
		swrl_tests:'Rex',
		transitive(swrl_tests:isParentOf),
		swrl_tests:'Person'))).

test('triple(-,transitive(+),+') :-
	findall(X,
		mongolog_call(triple(X,
			transitive(swrl_tests:isParentOf),
			swrl_tests:'Lea')),
		Ancestors),
	assert_unifies(Ancestors,[_,_]),
	assert_true(member(swrl_tests:'Fred', Ancestors)),
	assert_true(member(swrl_tests:'Rex', Ancestors)),
	%%
	assert_false(member(swrl_tests:'Ernest', Ancestors)).

test('triple(+,reflexive(transitive(+)),-)') :-
	findall(X,
		mongolog_call(triple(
			swrl_tests:'Rex',
			transitive(reflexive(swrl_tests:isParentOf)),
			X)),
		Ancestors),
	length(Ancestors,NumAncestors),
	assert_equals(NumAncestors, 4),
	assert_true(member(swrl_tests:'Rex', Ancestors)),
	assert_true(member(swrl_tests:'Ernest', Ancestors)),
	assert_true(member(swrl_tests:'Fred', Ancestors)),
	assert_true(member(swrl_tests:'Lea', Ancestors)).

test('call(+Triple)') :-
	assert_true(mongolog_call(call(triple(
		swrl_tests:'Rex',
		swrl_tests:isParentOf,
		swrl_tests:'Ernest')))),
	assert_false(mongolog_call(call(triple(
		swrl_tests:'Rex',
		swrl_tests:isParentOf,
		test_datatype:'Lecturer3')))).

test('call_with_context(+Triple,+Context)') :-
	assert_true(mongolog:test_call(
		call_with_context(
			triple(swrl_tests:'Rex', swrl_tests:isParentOf, swrl_tests:'Ernest'),
			[ query_scope(dict{ time: dict{ since: =<(Time), until: >=(Time) } }) ]
		), Time, 999)),
	%
	assert_false(mongolog:test_call(
		call_with_context(
			triple(swrl_tests:'Rex', swrl_tests:isParentOf, swrl_tests:'Rex'),
			[ query_scope(dict{ time: dict{ since: =<(Time), until: >=(Time) } }) ]
		), Time, 999)).

:- end_tests('mongolog_triple').
