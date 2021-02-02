:- use_module('rdf_tests').
:- begin_tests(
		'lang_query',
		[   setup(lang_query:set_default_graph(test)),
		    cleanup(rdf_tests:cleanup)
		]).

:- use_module(library('semweb/rdf_db'),
		[ rdf_global_term/2 ]).
:- use_module(library('rostest.pl')).
:- use_module(library('semweb/rdf_db'),
		[ rdf_register_ns/3 ]).
:- use_module('query').
:- use_module('db').
:- use_module('scope').

% register namespaces for following tests
:- rdf_register_ns(swrl_tests,
		'http://knowrob.org/kb/swrl_test#',
		[keep(true)]).
:- rdf_register_ns(test_datatype,
		'http://knowrob.org/kb/datatype_test#',
		[keep(true)]).

test('tell triple(a,b,c)') :-
	assert_true(lang_query:tell(triple(a,b,c))).

test('tell triple(a,b,_)') :-
	assert_false(lang_query:tell(triple(a,b,_))).

test('ask triple(a,b,c)') :-
	assert_true(lang_query:ask(triple(a,b,c))).

test('ask triple(A,b,c)') :-
	lang_query:ask(triple(A,b,c)),
	assert_equals(A,a).

test('ask triple(a,B,c)') :-
	lang_query:ask(triple(a,B,c)),
	assert_equals(B,b).

test('ask triple(a,b,C)') :-
	lang_query:ask(triple(a,b,C)),
	assert_equals(C,c).

test('ask triple(A,b,C)') :-
	lang_query:ask(triple(A,b,C)),
	assert_equals(A,a),
	assert_equals(C,c).

% load swrl owl file for tripledb testing
test('load local owl file') :-
	load_owl('package://knowrob/owl/test/swrl.owl',
		[ graph(test) ]),
	load_owl('package://knowrob/owl/test/datatype_test.owl',
		[ graph(test) ]).

% check via tripledb_ask if individual triple exists
test('ask triple') :-
	assert_true( lang_query:ask( triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))).

% delete individual triple
test('forget triple') :-
	assert_true( forget( triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))),
	assert_false( ask( triple(
		swrl_tests:'Adult',
		rdfs:'subClassOf',
		swrl_tests:'TestThing'
	))).

% add triple and check if it exists in db
test('tell to triplestore and check if triple exists') :-
    assert_true( tell( triple(
    	swrl_tests:'Adult',
    	rdfs:'subClassOf',
    	swrl_tests:'TestThing'
    ))),
    assert_true( ask( triple(
    	swrl_tests:'Adult',
    	rdfs:'subClassOf',
    	swrl_tests:'TestThing'
    ))).

% test for xsd:integer, Str, float
test('tell XSD') :-
	rdf_global_term(test_datatype:'Lecturer3',S),
	assert_true(tell(triple(S, test_datatype:'first_name', 'Johana$'))),
	assert_true(tell(triple(S, test_datatype:'last_name',  'Muller'))),
	assert_true(tell(triple(S, test_datatype:'studentId',  212123))),
	assert_true(tell(triple(S, test_datatype:'height',     5.10))).

% test for xsd:integer, Str, float
test('ask XSD') :-
	assert_true(forall(ask(triple(_, test_datatype:'studentId',  X)), number(X))),
	assert_true(forall(ask(triple(_, test_datatype:'first_name', Y)), atom(Y))),
	assert_true(forall(ask(triple(_, test_datatype:'last_name',  Z)), atom(Z))),
	assert_true(forall(ask(triple(_, test_datatype:'height',     H)), float(H))).

% test for list as an argument
test('tell list') :-
	rdf_global_term(test_datatype:'Lecturer3',S),
	DataTerm=[255,99,71],
	% test asserting list value
	assert_true(tell(triple(S, test_datatype:'hasHairColor', term(DataTerm)))),
	% test ask with ground value
	assert_true( ask(triple(S, test_datatype:'hasHairColor', term(DataTerm)))),
	% test ask with var value
	ask(triple(S, test_datatype:'hasHairColor', term(Actual))),
	assert_true(is_list(Actual)),
	assert_equals(Actual,DataTerm).

% test for time scope
test('tell with scope'):-
	rdf_global_term(test_datatype:'Lecturer4',S),
	rdf_global_term(test_datatype:'last_name',P),
	time_scope(=(double(5)), =(double(10)), T_S1),
	time_scope(=(double(5)), =(double(20)), T_S2),
	%%
	tell(triple(S, P, 'Spiendler'), T_S1),
	assert_true( ask(triple(S, P, 'Spiendler'), T_S1, _)),
	assert_false(ask(triple(S, P, 'Spiendler'), T_S2, _)).

% test for time scope extension
test('extend time scope'):-
	rdf_global_term(test_datatype:'Lecturer4',S),
	rdf_global_term(test_datatype:'last_name',P),
	time_scope(=(double(10)), =(double(20)), T_S1),
	time_scope(=(double(5)),  =(double(20)), T_S2),
	%%
	tell(triple(S, P, 'Spiendler'), T_S1),
	assert_true(ask(triple(S, P, 'Spiendler'), T_S2, _)).

test('query value operators') :-
	assert_true(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', =(6)))),
	assert_true(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', =<(9)))),
	assert_true(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', <(7)))),
	assert_true(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', >=(5)))),
	assert_true(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', >(3.5)))),
	assert_false(ask(triple(swrl_tests:'RectangleSmall',swrl_tests:'hasHeightInMeters', <(3)))).

test('query operator in'):-
	findall(LastName,
		ask(triple(
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
		ask(triple(
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

% test for special characters in iri: @*~!#?
test('non alphabetic character'):-
	assert_true(tell(triple(
		test_datatype:'normal_user_test_new',
		test_datatype:'last@*~!#?_name',
		'umlaut'
	))),
	assert_true(ask(triple(
		test_datatype:'normal_user_test_new',
		test_datatype:'last@*~!#?_name',
		'umlaut'
	))).

test('non utf8 character'):-
	tell(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		'Müller'
	)),
	assert_true(ask(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		'Müller'
	))).

% test for non existent triples
test('ask non existant'):-
	assert_false( ask(
		triple(test_datatype:'xyz', test_datatype:'last_name', _)
	)).

test('transitive') :-
	findall(X,
		ask(triple(
			swrl_tests:'Rex',
			transitive(swrl_tests:isParentOf),
			X
		)),
		Ancestors),
	assert_unifies(Ancestors,[_,_,_]),
	assert_true(member(swrl_tests:'Ernest', Ancestors)),
	assert_true(member(swrl_tests:'Fred', Ancestors)),
	assert_true(member(swrl_tests:'Lea', Ancestors)).

test('transitive+reflexive') :-
	findall(X,
		ask(triple(
			swrl_tests:'Rex',
			transitive(reflexive(swrl_tests:isParentOf)),
			X)),
		Ancestors),
	%% FIXME: reflexive may create some redundant results
	%assert_unifies(Ancestors,[_,_,_,_]),
	list_to_set(Ancestors,Ancestors0),
	assert_unifies(Ancestors0,[_,_,_,_]),
	assert_true(member(swrl_tests:'Rex', Ancestors)),
	assert_true(member(swrl_tests:'Ernest', Ancestors)),
	assert_true(member(swrl_tests:'Fred', Ancestors)),
	assert_true(member(swrl_tests:'Lea', Ancestors)).

:- end_tests('lang_query').
