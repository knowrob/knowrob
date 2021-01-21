:- use_module('rdf_tests').
:- begin_tests(
		'lang_query',
		[   setup(lang_query:set_default_graph(test)),
		    cleanup(rdf_tests:cleanup)
		]).

:- use_module(library('db/tripledb')).
:- use_module(library('rostest.pl')).
:- use_module(library('semweb/rdf_db'),  [ rdf_register_ns/3 ]).

% register namespaces for following tests
:- rdf_register_ns(swrl_tests,
		'http://knowrob.org/kb/swrl_test#',
		[keep(true)]).
:- rdf_register_ns(test_datatype,
		'http://knowrob.org/kb/datatype_test#',
		[keep(true)]).

% load swrl owl file for tripledb testing
test('load local owl file') :-
	tripledb_load(
		'package://knowrob/owl/test/swrl.owl',
		[ graph(test)
		]),
	tripledb_load(
		'package://knowrob/owl/test/datatype_test.owl',
		[ graph(test)
		]).

% check via tripledb_ask if individual triple exists
test('ask triple') :-
	assert_true( ask( triple(
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
	))).

% check again for that triple and it should not be in the tripledb
test('ask after deletion', [fail]) :-
	assert_true( ask( triple(
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
	rdf_equal(S, test_datatype:'Lecturer3'),
	assert_true(tell(triple(S, test_datatype:'first_name', 'Johana$'))),
	assert_true(tell(triple(S, test_datatype:'last_name',  'Muller'))),
	assert_true(tell(triple(S, test_datatype:'studentId',  212123))),
	assert_true(tell(triple(S, test_datatype:'height',     5.10))).

% test for xsd:integer, Str, float
test('ask XSD') :-
	assert_true(forall(ask(triple(_, test_datatype:'studentId',  X), number(X)))),
	assert_true(forall(ask(triple(_, test_datatype:'first_name', Y), atom(Y)))),
	assert_true(forall(ask(triple(_, test_datatype:'last_name',  Z), atom(Z)))),
	assert_true(forall(ask(triple(_, test_datatype:'height',     H), float(H)))).

% test for list as an argument
test('tell list') :-
	rdf_equal(S, test_datatype:'Lecturer3'),
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
	T_S1=_{ time: _{ since: =(double(5)), until: =(double(10)) }},
	T_S2=_{ time: _{ since: =(double(5)), until: =(double(20)) }},
	rdf_equal(S, test_datatype:'Lecturer4'),
	rdf_equal(P, test_datatype:'last_name'),
	%%
	tell(triple(S, P, 'Spiendler'), T_S1),
	assert_true( ask(triple(S, P, 'Spiendler'), T_S1, _)),
	assert_false(ask(triple(S, P, 'Spiendler'), T_S2, _)).

% test for time scope extension
test('extend time scope'):-
	T_S1=_{ time: _{ since: =(double(10)), until: =(double(20)) }},
	T_S2=_{ time: _{ since: =(double(5)),  until: =(double(20)) }},
	rdf_equal(S, test_datatype:'Lecturer4'),
	rdf_equal(P, test_datatype:'last_name'),
	%%
	tell(triple(S, P, 'Spiendler'), T_S1),
	assert_true(ask(triple(S, P, 'Spiendler'), T_S2, _)).

% test for unit
test('query units'):-
	rdf_equal(S, test_datatype:'Lecturer4'),
	rdf_equal(P, test_datatype:'height'),
	% value is not there initially 
	assert_false(ask(triple(S, P, unit(double(2.1),'meter')))),
	% them assert the value with unit
	assert_true(tell(triple(S, P, unit(double(2.1),'meter')))),
	% and test if the value and unit can be retrieved again
	assert_true( ask(triple(S, P, unit(double(2.1),'meter')))).

test('query operator >='):-
	ask(triple(
		test_datatype:'Lecturer4',
		test_datatype:'height',
		>=(unit(double(2.0),'meter'))->V
	)),
	assert_equals(V,2.1).

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
	assert_unifies(Names,[_,_]),
	assert_true(member('Muller',Names)),
	assert_true(member('Spiendler',Names)).

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
	assert_unifies(LecturerList,[_,_]),
	assert_true(member(
		[test_datatype:'Lecturer3','Muller'],
		LecturerList)),
	assert_true(member(
		[test_datatype:'Lecturer4','Spiendler'],
		LecturerList)).

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

% it is observed that once these special characters are stored in db(with wrong format),
% they can not be retrieved normally in variables or any other way.
% Hence, in the code we throw a warning
test('non utf8 character',
	[ fixme('fix encoding for special characters e.g. umlaut') ]):-
	tell(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		'Müller'
	)),
	assert_true(ask(triple(
		test_datatype:'Lecturer3',
		test_datatype:'last_name',
		_
	))).

% test for non existent triples
test('ask non existant'):-
	assert_false( ask(
		triple(test_datatype:'xyz', test_datatype:'last_name', _)
	)).

test('transitive') :-
	findall(X,
		ask( transitive(
			triple(swrl_tests:'Rex',swrl_tests:isParentOf,X)
		)),
		Ancestors),
	assert_unifies(Ancestors,[_,_,_]),
	assert_true(member(swrl_tests:'Ernest', Ancestors)),
	assert_true(member(swrl_tests:'Fred', Ancestors)),
	assert_true(member(swrl_tests:'Lea', Ancestors)).

test('transitive+reflexive') :-
	findall(X,
		ask( transitive( reflexive(
				triple(swrl_tests:'Rex',swrl_tests:isParentOf,X)
		))),
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
