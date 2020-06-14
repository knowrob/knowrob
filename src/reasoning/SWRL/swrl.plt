:- use_module(library('db/tripledb_tests')).
:- begin_tripledb_tests(
		'swrl',
		'package://knowrob/owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#')
		]).

:- use_module(library('semweb/rdf_db'),    [ rdf_equal/2 ]).
:- use_module(library('db/tripledb'),      [ tripledb_load/2 ]).
:- use_module(library('model/RDFS'),       [ has_type/2 ]).
:- use_module(library('lang/terms/is_a'),  [ instance_of/2 ]).
:- use_module(library('lang/terms/holds'), [ holds/3 ]).
:- use_module(library('reasoning/pool'),   [ reasoning_module/1 ]).

:- use_module('./swrl.pl').
:- use_module('./parser.pl').

:- reasoning_module(swrl).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Parsing
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_parse_rules) :-
  swrl_file_path(knowrob,'test.swrl',Filepath),
  forall(
    swrl_file_parse(Filepath,Rule,_),
    ( Rule=(Head:-Body), Head \= [], Body \= [] )
  ).

:- rdf_meta(test_swrl_parse(?,t)).

test_swrl_parse(ExprList, Term) :-
  atomic_list_concat(ExprList, ' ', Expr),
  swrl_phrase(Term_gen, Expr, 'http://knowrob.org/kb/swrl_test#'),
  Term_gen = Term,
  swrl_phrase(Term_gen, Expr_gen, 'http://knowrob.org/kb/swrl_test#'), % test in both directions!
  Expr_gen = Expr.

test(swrl_parse_Driver, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasCar(?p, true)', '->', 'Driver(?p)'],
    [ class(test:'Driver',var(p)) ] :-
      [ class(dul:'Person',var(p)),
      property(var(p), test:'hasCar', true) ]
  ).

test(swrl_parse_DriverFred, [nondet]) :-
  test_swrl_parse(
    ['Person(Fred), hasCar(Fred, true)', '->', 'Driver(Fred)'],
    [ class(test:'Driver',test:'Fred') ] :-
      [ class(dul:'Person',test:'Fred'),
        property(test:'Fred', test:'hasCar', true) ]
  ).

test(swrl_parse_brother, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasSibling(?p, ?s), Man(?s)', '->', 'hasBrother(?p, ?s)'],
    [ property(var(p), test:'hasBrother', var(s)) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test:'hasSibling', var(s)),
        class(test:'Man',var(s)) ]
  ).

test(swrl_parse_startsWith, [nondet]) :-
  test_swrl_parse(
    [ 'Person(?p), hasNumber(?p, ?number), startsWith(?number, "+")',
      '->',
      'hasInternationalNumber(?p, true)' ],
    [ property(var(p), test:'hasInternationalNumber', true) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test:'hasNumber', var(number)),
        startsWith(var(number),+) ]
  ).

test(swrl_parse_exactly, [nondet]) :-
  test_swrl_parse(
    ['Person(?x), (hasSibling exactly 0 Person)(?x)', '->', 'Singleton(?x)'],
    [ class(test:'Singleton', var(x)) ] :-
      [ class(dul:'Person', var(x)),
        class(exactly(0, test:'hasSibling', dul:'Person'), var(x)) ]
  ).

test(swrl_parse_Person1, [nondet]) :-
  test_swrl_parse(
    ['(Man or Woman)(?x)', '->', 'Person(?x)'],
    [ class(dul:'Person', var(x)) ] :-
      [ class(union_of([test:'Man',test:'Woman']), var(x)) ]
  ).

test(swrl_parse_Person2, [nondet]) :-
  test_swrl_parse(
    ['(Man and Woman and Person)(?x)', '->', 'Hermaphrodite(?x)'],
    [ class(test:'Hermaphrodite', var(x)) ] :-
      [ class(intersection_of([test:'Man',test:'Woman',dul:'Person']), var(x)) ]
  ).

test(swrl_parse_NonHuman, [nondet]) :-
  test_swrl_parse(
    ['(not Person)(?x)', '->', 'NonHuman(?x)'],
    [ class(test:'NonHuman', var(x)) ] :-
      [ class(not(dul:'Person'), var(x)) ]
  ).

test(swrl_parse_Adult1, [nondet]) :-
  test_swrl_parse(
    ['greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(test:'Adult', var(p)) ] :-
      [ greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult2, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasAge(?p, ?age), greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(test:'Adult', var(p)) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test:'hasAge', var(age)),
        greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult3, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (hasChild value true))(?x)', '->', 'Adult(?x)'],
    [ class(test:'Adult', var(x)) ] :-
      [ class(union_of([
           test:'Driver',
           value(test:'hasChild',true)
        ]), var(x)) ]
  ).

test(swrl_parse_nested, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (not (Car and NonHuman)))(?x)', '->', 'Person(?x)'],
    [ class(dul:'Person', var(x)) ] :-
      [ class(union_of([test:'Driver', not(intersection_of(
          [test:'Car',test:'NonHuman'])) ]), var(x)) ]
  ).

test(swrl_parse_area, [nondet]) :-
  test_swrl_parse(
    [ 'Rectangle(?r), hasWidthInMeters(?r, ?w), hasHeightInMeters(?r, ?h), multiply(?areaInSquareMeters, ?w, ?h)',
      '->',
      'hasAreaInSquareMeters(?r, ?areaInSquareMeters)' ],
    [ property(var(r), test:'hasAreaInSquareMeters', var(areaInSquareMeters)) ] :-
      [ class(test:'Rectangle', var(r)),
        property(var(r), test:'hasWidthInMeters', var(w)),
        property(var(r), test:'hasHeightInMeters', var(h)),
        multiply(var(areaInSquareMeters),var(w),var(h)) ]
  ).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Asserting RDF/XML SWRL rules as Prolog rules
% % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Driver_load) :-
  \+ instance_of(test:'Fred', test:'Driver'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Driver').

test(swrl_Driver) :-
  instance_of(test:'Fred', test:'Driver').

test(swrl_Driver_class_unbound, [nondet]) :-
  instance_of(test:'Fred', X),
  rdf_equal(X, test:'Driver').

test(swrl_Driver_subject_unbound, [nondet]) :-
  instance_of(X, test:'Driver'),
  rdf_equal(X, test:'Fred').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person1) :-
  \+ instance_of(test:'Alex', dul:'Person'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Person'),
  instance_of(test:'Alex', dul:'Person').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person2) :-
  \+ instance_of(test:'Lea', test:'Hermaphrodite'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Hermaphrodite'),
  instance_of(test:'Lea', test:'Hermaphrodite').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_NonHuman) :-
  \+ instance_of(test:'RedCar', test:'NonHuman'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'NonHuman').

test(swrl_NonHuman_RedCar) :-
  instance_of(test:'RedCar', test:'NonHuman').

test(swrl_NonHuman_Fred) :-
  \+ instance_of(test:'Fred', test:'NonHuman').

test(swrl_NonHuman_class_unbound, [nondet]) :-
  instance_of(test:'RedCar', X),
  rdf_equal(X, test:'NonHuman').

test(swrl_NonHuman_subject_unbound, [nondet]) :-
  instance_of(X, test:'NonHuman'),
  rdf_equal(X, test:'RedCar').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_area, [nondet]) :-
  \+ holds(test:'RectangleBig', test:'hasAreaInSquareMeters', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'area'),
  holds(test:'RectangleBig', test:'hasAreaInSquareMeters', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_startsWith_load) :-
  \+ holds(test:'Fred', test:'hasInternationalNumber', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'startsWith').

test(swrl_startsWith_holds1, [nondet]) :-
  holds(test:'Fred', test:'hasInternationalNumber', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_hasBrother_load) :-
  \+ holds(test:'Fred', test:'hasBrother', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'brother').

test(swrl_hasBrother_holds) :-
  holds(test:'Fred', test:'hasBrother', test:'Ernest').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_BigRectangle1, [nondet]) :-
  \+ instance_of(test:'RectangleBig', test:'BigRectangle'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'BigRectangle'),
  instance_of(test:'RectangleBig', test:'BigRectangle').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_greaterThen) :-
  \+ instance_of(test:'Ernest', test:'Adult'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'greaterThen'),
  instance_of(test:'Ernest', test:'Adult').

% % % % % % % % % % % % % % % % % % % % % % % %
test(test_rdf_swrl_unload) :-
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_unload(Filepath).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % SWRL rules asserted from human readable expressions
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_phrase_hasUncle, [nondet]) :-
  \+ holds(test:'Lea', test:'hasUncle', test:'Ernest'),
  swrl_phrase_assert('hasParent(?x, ?y), hasBrother(?y, ?z) -> hasUncle(?x, ?z)',
                     'http://knowrob.org/kb/swrl_test#'),
  holds(test:'Lea', test:'hasUncle', test:'Ernest').

:- end_tripledb_tests('swrl').

