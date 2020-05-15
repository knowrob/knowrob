
:- begin_tests('reasoning/SWRL/swrl').

:- use_module(library('semweb/rdf_db'),    [ rdf_equal/2 ]).
:- use_module(library('db/tripledb'),      [ tripledb_load/2 ]).
:- use_module(library('model/RDFS'),       [ has_type/2 ]).
:- use_module(library('lang/terms/is_a'),  [ instance_of/2 ]).
:- use_module(library('lang/terms/holds'), [ holds/3 ]).
:- use_module(library('reasoning/pool'),   [ reasoning_module/1 ]).

:- use_module('./swrl.pl').
:- use_module('./parser.pl').

:- tripledb_load(
        'package://knowrob/owl/test/swrl.owl',
        [ graph(user),
          namespace(test_swrl,'http://knowrob.org/kb/swrl_test#')
        ]).

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
    [ class(test_swrl:'Driver',var(p)) ] :-
      [ class(dul:'Person',var(p)),
      property(var(p), test_swrl:'hasCar', true) ]
  ).

test(swrl_parse_DriverFred, [nondet]) :-
  test_swrl_parse(
    ['Person(Fred), hasCar(Fred, true)', '->', 'Driver(Fred)'],
    [ class(test_swrl:'Driver',test_swrl:'Fred') ] :-
      [ class(dul:'Person',test_swrl:'Fred'),
        property(test_swrl:'Fred', test_swrl:'hasCar', true) ]
  ).

test(swrl_parse_brother, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasSibling(?p, ?s), Man(?s)', '->', 'hasBrother(?p, ?s)'],
    [ property(var(p), test_swrl:'hasBrother', var(s)) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test_swrl:'hasSibling', var(s)),
        class(test_swrl:'Man',var(s)) ]
  ).

test(swrl_parse_startsWith, [nondet]) :-
  test_swrl_parse(
    [ 'Person(?p), hasNumber(?p, ?number), startsWith(?number, "+")',
      '->',
      'hasInternationalNumber(?p, true)' ],
    [ property(var(p), test_swrl:'hasInternationalNumber', true) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test_swrl:'hasNumber', var(number)),
        startsWith(var(number),+) ]
  ).

test(swrl_parse_exactly, [nondet]) :-
  test_swrl_parse(
    ['Person(?x), (hasSibling exactly 0 Person)(?x)', '->', 'Singleton(?x)'],
    [ class(test_swrl:'Singleton', var(x)) ] :-
      [ class(dul:'Person', var(x)),
        class(exactly(0, test_swrl:'hasSibling', dul:'Person'), var(x)) ]
  ).

test(swrl_parse_Person1, [nondet]) :-
  test_swrl_parse(
    ['(Man or Woman)(?x)', '->', 'Person(?x)'],
    [ class(dul:'Person', var(x)) ] :-
      [ class(union_of([test_swrl:'Man',test_swrl:'Woman']), var(x)) ]
  ).

test(swrl_parse_Person2, [nondet]) :-
  test_swrl_parse(
    ['(Man and Woman and Person)(?x)', '->', 'Hermaphrodite(?x)'],
    [ class(test_swrl:'Hermaphrodite', var(x)) ] :-
      [ class(intersection_of([test_swrl:'Man',test_swrl:'Woman',dul:'Person']), var(x)) ]
  ).

test(swrl_parse_NonHuman, [nondet]) :-
  test_swrl_parse(
    ['(not Person)(?x)', '->', 'NonHuman(?x)'],
    [ class(test_swrl:'NonHuman', var(x)) ] :-
      [ class(not(dul:'Person'), var(x)) ]
  ).

test(swrl_parse_Adult1, [nondet]) :-
  test_swrl_parse(
    ['greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(test_swrl:'Adult', var(p)) ] :-
      [ greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult2, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasAge(?p, ?age), greaterThan(?age, 17)', '->', 'Adult(?p)'],
    [ class(test_swrl:'Adult', var(p)) ] :-
      [ class(dul:'Person', var(p)),
        property(var(p), test_swrl:'hasAge', var(age)),
        greaterThan(var(age),17) ]
  ).

test(swrl_parse_Adult3, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (hasChild value true))(?x)', '->', 'Adult(?x)'],
    [ class(test_swrl:'Adult', var(x)) ] :-
      [ class(union_of([
           test_swrl:'Driver',
           value(test_swrl:'hasChild',true)
        ]), var(x)) ]
  ).

test(swrl_parse_nested, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (not (Car and NonHuman)))(?x)', '->', 'Person(?x)'],
    [ class(dul:'Person', var(x)) ] :-
      [ class(union_of([test_swrl:'Driver', not(intersection_of(
          [test_swrl:'Car',test_swrl:'NonHuman'])) ]), var(x)) ]
  ).

test(swrl_parse_area, [nondet]) :-
  test_swrl_parse(
    [ 'Rectangle(?r), hasWidthInMeters(?r, ?w), hasHeightInMeters(?r, ?h), multiply(?areaInSquareMeters, ?w, ?h)',
      '->',
      'hasAreaInSquareMeters(?r, ?areaInSquareMeters)' ],
    [ property(var(r), test_swrl:'hasAreaInSquareMeters', var(areaInSquareMeters)) ] :-
      [ class(test_swrl:'Rectangle', var(r)),
        property(var(r), test_swrl:'hasWidthInMeters', var(w)),
        property(var(r), test_swrl:'hasHeightInMeters', var(h)),
        multiply(var(areaInSquareMeters),var(w),var(h)) ]
  ).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Asserting RDF/XML SWRL rules as Prolog rules
% % % % % % % % % % % % % % % % % % % % % % % %

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Driver_load) :-
  \+ instance_of(test_swrl:'Fred', test_swrl:'Driver'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Driver').

test(swrl_Driver) :-
  instance_of(test_swrl:'Fred', test_swrl:'Driver').

test(swrl_Driver_class_unbound, [nondet]) :-
  instance_of(test_swrl:'Fred', X),
  rdf_equal(X, test_swrl:'Driver').

test(swrl_Driver_subject_unbound, [nondet]) :-
  instance_of(X, test_swrl:'Driver'),
  rdf_equal(X, test_swrl:'Fred').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person1) :-
  \+ instance_of(test_swrl:'Alex', dul:'Person'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Person'),
  instance_of(test_swrl:'Alex', dul:'Person').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person2) :-
  \+ instance_of(test_swrl:'Lea', test_swrl:'Hermaphrodite'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Hermaphrodite'),
  instance_of(test_swrl:'Lea', test_swrl:'Hermaphrodite').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_NonHuman) :-
  \+ instance_of(test_swrl:'RedCar', test_swrl:'NonHuman'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'NonHuman').

test(swrl_NonHuman_RedCar) :-
  instance_of(test_swrl:'RedCar', test_swrl:'NonHuman').

test(swrl_NonHuman_Fred) :-
  \+ instance_of(test_swrl:'Fred', test_swrl:'NonHuman').

test(swrl_NonHuman_class_unbound, [nondet]) :-
  instance_of(test_swrl:'RedCar', X),
  rdf_equal(X, test_swrl:'NonHuman').

test(swrl_NonHuman_subject_unbound, [nondet]) :-
  instance_of(X, test_swrl:'NonHuman'),
  rdf_equal(X, test_swrl:'RedCar').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_area, [nondet]) :-
  \+ holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'area'),
  holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_startsWith_load) :-
  \+ holds(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'startsWith').

test(swrl_startsWith_holds1, [nondet]) :-
  holds(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_hasBrother_load) :-
  \+ holds(test_swrl:'Fred', test_swrl:'hasBrother', _),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'brother').

test(swrl_hasBrother_holds) :-
  holds(test_swrl:'Fred', test_swrl:'hasBrother', test_swrl:'Ernest').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_BigRectangle1, [nondet]) :-
  \+ instance_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'BigRectangle'),
  instance_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_greaterThen) :-
  \+ instance_of(test_swrl:'Ernest', test_swrl:'Adult'),
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_load(Filepath,'greaterThen'),
  instance_of(test_swrl:'Ernest', test_swrl:'Adult').

% % % % % % % % % % % % % % % % % % % % % % % %
test(test_rdf_swrl_unload) :-
  swrl_file_path(knowrob,'test.swrl',Filepath),
  swrl_file_unload(Filepath).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % SWRL rules asserted from human readable expressions
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_phrase_hasUncle, [nondet]) :-
  \+ holds(test_swrl:'Lea', test_swrl:'hasUncle', test_swrl:'Ernest'),
  swrl_phrase_assert('hasParent(?x, ?y), hasBrother(?y, ?z) -> hasUncle(?x, ?z)',
                     'http://knowrob.org/kb/swrl_test#'),
  holds(test_swrl:'Lea', test_swrl:'hasUncle', test_swrl:'Ernest').

:- end_tests('reasoning/SWRL/swrl').

