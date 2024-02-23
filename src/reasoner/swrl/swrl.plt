:- use_module(library('rdf_test')).
:- begin_rdf_tests('swrl', 'owl/test/swrl.owl').

:- use_module(library('semweb/rdf_db'), [ rdf_equal/2, rdf_has/3 ]).
:- use_module(library('semweb/rdfs'), [ rdfs_individual_of/2 ]).
:- use_module(library('semweb'), [ sw_register_prefix/2 ]).
:- use_module(library('logging')).
:- use_module(library('unittest')).
:- use_module('swrl').
:- use_module('parser').

:- sw_register_prefix(test, 'http://knowrob.org/kb/swrl_test#').

swrl_test_file(test('swrl/test.swrl')).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Parsing
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_parse_rules) :-
  swrl_test_file(Filepath),
  forall(
    swrl_file_parse(Filepath,Rule,_),
    ( Rule=(':-'(Head,Body)), Head \= [], Body \= [] )
  ).

:- rdf_meta(test_swrl_parse(?,t)).

test_swrl_parse(ExprList, Term) :-
  atomic_list_concat(ExprList, ' ', Expr),
  swrl_phrase(Term_gen, Expr, 'http://knowrob.org/kb/swrl_test#'),
  Term_gen = Term,
  swrl_phrase(Term_gen, Expr_gen, 'http://knowrob.org/kb/swrl_test#'), % tests in both directions!
  Expr_gen = Expr.

test(swrl_parse_Driver, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasCar(?p, true)', '->', 'Driver(?p)'],
    [ class(class(test:'Driver'),var(p)) ] :-
      [ class(class(test:'Person'),var(p)),
      property(var(p), test:'hasCar', true) ]
  ).

test(swrl_parse_DriverFred, [nondet]) :-
  test_swrl_parse(
    ['Person(Fred), hasCar(Fred, true)', '->', 'Driver(Fred)'],
    [ class(class(test:'Driver'),test:'Fred') ] :-
      [ class(class(test:'Person'),test:'Fred'),
        property(test:'Fred', test:'hasCar', true) ]
  ).

test(swrl_parse_brother, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasSibling(?p, ?s), Man(?s)', '->', 'hasBrother(?p, ?s)'],
    [ property(var(p), test:'hasBrother', var(s)) ] :-
      [ class(class(test:'Person'), var(p)),
        property(var(p), test:'hasSibling', var(s)),
        class(class(test:'Man'),var(s)) ]
  ).

test(swrl_parse_startsWith, [nondet]) :-
  test_swrl_parse(
    [ 'Person(?p), hasNumber(?p, ?number), startsWith(?number, "+")',
      '->',
      'hasInternationalNumber(?p, true)' ],
    [ property(var(p), test:'hasInternationalNumber', true) ] :-
      [ class(class(test:'Person'), var(p)),
        property(var(p), test:'hasNumber', var(number)),
        startsWith(var(number),+) ]
  ).

test(swrl_parse_exactly, [nondet]) :-
  test_swrl_parse(
    ['Person(?x), (hasSibling exactly 0 Person)(?x)', '->', 'Singleton(?x)'],
    [ class(class(test:'Singleton'), var(x)) ] :-
      [ class(class(test:'Person'), var(x)),
        class(exactly(0, test:'hasSibling', class(test:'Person')), var(x)) ]
  ).

test(swrl_parse_Person1, [nondet]) :-
  test_swrl_parse(
    ['(Man or Woman)(?x)', '->', 'Person(?x)'],
    [ class(class(test:'Person'), var(x)) ] :-
      [ class(union_of([class(test:'Man'),class(test:'Woman')]), var(x)) ]
  ).

test(swrl_parse_Person2, [nondet]) :-
  test_swrl_parse(
    ['(Man and Woman and Person)(?x)', '->', 'Hermaphrodite(?x)'],
    [ class(class(test:'Hermaphrodite'), var(x)) ] :-
      [ class(intersection_of([class(test:'Man'),class(test:'Woman'),class(test:'Person')]), var(x)) ]
  ).

test(swrl_parse_NonHuman, [nondet]) :-
  test_swrl_parse(
    ['(not Person)(?x)', '->', 'NonHuman(?x)'],
    [ class(class(test:'NonHuman'), var(x)) ] :-
      [ class(complement_of(class(test:'Person')), var(x)) ]
  ).

test(swrl_parse_Adult1, [nondet]) :-
  test_swrl_parse(
    ['greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(class(test:'Adult'), var(p)) ] :-
      [ greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult2, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasAge(?p, ?age), greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(class(test:'Adult'), var(p)) ] :-
      [ class(class(test:'Person'), var(p)),
        property(var(p), test:'hasAge', var(age)),
        greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult3, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (hasChild value true))(?x)', '->', 'Adult(?x)'],
    [ class(class(test:'Adult'), var(x)) ] :-
      [ class(union_of([
           class(test:'Driver'),
           value(test:'hasChild',true)
        ]), var(x)) ]
  ).

test(swrl_parse_nested, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (not (Car and NonHuman)))(?x)', '->', 'Person(?x)'],
    [ class(class(test:'Person'), var(x)) ] :-
      [ class(union_of([class(test:'Driver'), complement_of(intersection_of(
          [class(test:'Car'),class(test:'NonHuman')])) ]), var(x)) ]
  ).

test(swrl_parse_area, [nondet]) :-
  test_swrl_parse(
    [ 'Rectangle(?r), hasWidthInMeters(?r, ?w), hasHeightInMeters(?r, ?h), multiply(?areaInSquareMeters, ?w, ?h)',
      '->',
      'hasAreaInSquareMeters(?r, ?areaInSquareMeters)' ],
    [ property(var(r), test:'hasAreaInSquareMeters', var(areaInSquareMeters)) ] :-
      [ class(class(test:'Rectangle'), var(r)),
        property(var(r), test:'hasWidthInMeters', var(w)),
        property(var(r), test:'hasHeightInMeters', var(h)),
        multiply(var(areaInSquareMeters),var(w),var(h)) ]
  ).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Asserting RDF/XML SWRL rules as Prolog rules
% % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Driver) :-
	assert_false(rdfs_individual_of(test:'Fred', test:'Driver')),
	assert_true(rdfs_individual_of(test:'Fred', test:'Person')),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'Driver'),
	assert_true(rdfs_individual_of(test:'Fred', test:'Driver')).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person) :-
	assert_false(rdfs_individual_of(test:'Alex', test:'Person')),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'Person'),
	assert_true(rdfs_individual_of(test:'Alex', test:'Person')).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Hermaphrodite) :-
	assert_false(rdfs_individual_of(test:'Lea', test:'Hermaphrodite')),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'Hermaphrodite'),
	assert_true(rdfs_individual_of(test:'Lea', test:'Hermaphrodite')),
	assert_false(rdfs_individual_of(test:'Fred', test:'Hermaphrodite')).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_area) :-
	assert_false(rdf_has(test:'RectangleBig', test:'hasAreaInSquareMeters', _)),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'area'),
	assert_true(rdf_has(test:'RectangleBig', test:'hasAreaInSquareMeters', _)).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_startsWith) :-
	assert_true(rdfs_individual_of(test:'Fred', test:'Person')),
	assert_true(rdf_has(test:'Fred', test:'hasNumber', _)),
	assert_false(rdf_has(test:'Fred', test:'hasInternationalNumber', _)),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'startsWith'),
	assert_true(rdf_has(test:'Fred', test:'hasInternationalNumber', _)).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_hasBrother) :-
	assert_false(rdf_has(test:'Fred', test:'hasBrother', _)),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'brother'),
	assert_true(rdf_has(test:'Fred', test:'hasBrother', test:'Ernest')).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_BigRectangle1) :-
	assert_false(rdfs_individual_of(test:'RectangleBig', test:'BigRectangle')),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'BigRectangle'),
	assert_true(rdfs_individual_of(test:'RectangleBig', test:'BigRectangle')).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_greaterThen) :-
	assert_false(rdfs_individual_of(test:'Ernest', test:'Adult')),
	swrl_test_file(Filepath),
	swrl_file_fire(Filepath,'greaterThen'),
	assert_true(rdfs_individual_of(test:'Ernest', test:'Adult')).


% % % % % % % % % % % % % % % % % % % % % % % %
% % % % SWRL rules asserted from human readable expressions
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_phrase_hasUncle) :-
	assert_false(rdf_has(test:'Lea', test:'hasUncle', test:'Ernest')),
	assert_true(swrl_parser:swrl_phrase_fire(
		'hasParent(?x, ?y), hasBrother(?y, ?z) -> hasUncle(?x, ?z)',
		'http://knowrob.org/kb/swrl_test#')),
	assert_true(rdf_has(test:'Lea', test:'hasUncle', test:'Ernest')).

:- end_rdf_tests('swrl').
