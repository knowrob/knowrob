
:- begin_tests(swrl).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
%%
:- use_module(library('knowrob/lang/ask')).
:- use_module(library('knowrob/lang/tell')).

:- use_module(library('swrl')).
:- use_module(library('swrl_parser')).
:- use_module(library('rdf_swrl')).

:- owl_parser:owl_parse('package://swrlprolog/owl/test.owl').

:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Parsing
% % % % % % % % % % % % % % % % % % % % % % % %

test(swrl_parse_rules) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  forall(
    swrl_file_parse(Filepath,Rule,_),
    ( Rule=(Head:-Body), Head \= [], Body \= [] )
  ).
:- rdf_meta test_swrl_parse(?,t).

test_swrl_parse(ExprList, Term) :-
  atomic_list_concat(ExprList, ' ', Expr),
  swrl_phrase(Term_gen, Expr),
  Term_gen = Term,
  swrl_phrase(Term_gen, Expr_gen), % test in both directions!
  Expr_gen = Expr.

test(swrl_parse_Driver, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasCar(?p, true)', '->', 'Driver(?p)'],
    [ class(test_swrl:'Driver',var(p)) ] :-
      [ class(dul:'Person',var(p)),
      property(var(p), test_swrl:'hasCar', literal(type(xsd:'boolean',true))) ]
  ).

test(swrl_parse_DriverFred, [nondet]) :-
  test_swrl_parse(
    ['Person(Fred), hasCar(Fred, true)', '->', 'Driver(Fred)'],
    [ class(test_swrl:'Driver',test_swrl:'Fred') ] :-
      [ class(dul:'Person',test_swrl:'Fred'),
        property(test_swrl:'Fred', test_swrl:'hasCar', literal(type(xsd:'boolean',true))) ]
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
    [ property(var(p), test_swrl:'hasInternationalNumber', literal(type(xsd:boolean,true))) ] :-
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
      [ class(oneOf([test_swrl:'Man',test_swrl:'Woman']), var(x)) ]
  ).

test(swrl_parse_Person2, [nondet]) :-
  test_swrl_parse(
    ['(Man and Woman and Person)(?x)', '->', 'Hermaphrodite(?x)'],
    [ class(test_swrl:'Hermaphrodite', var(x)) ] :-
      [ class(allOf([test_swrl:'Man',test_swrl:'Woman',dul:'Person']), var(x)) ]
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
      [ class(oneOf([
           test_swrl:'Driver',
           value(test_swrl:'hasChild', literal(type(_, true)))
        ]), var(x)) ]
  ).

test(swrl_parse_nested, [nondet]) :-
  test_swrl_parse(
    ['(Driver or (not (Car and NonHuman)))(?x)', '->', 'Person(?x)'],
    [ class(dul:'Person', var(x)) ] :-
      [ class(oneOf([test_swrl:'Driver', not(allOf(
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
  \+ kb_type_of(test_swrl:'Fred', test_swrl:'Driver'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Driver').

test(swrl_Driver) :-
  kb_type_of(test_swrl:'Fred', test_swrl:'Driver').

test(swrl_Driver_class_unbound, [nondet]) :-
  kb_type_of(test_swrl:'Fred', X),
  rdf_equal(X, test_swrl:'Driver').

test(swrl_Driver_subject_unbound, [nondet]) :-
  kb_type_of(X, test_swrl:'Driver'),
  rdf_equal(X, test_swrl:'Fred').

%% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person1) :-
  \+ kb_type_of(test_swrl:'Alex', dul:'Person'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Person'),
  kb_type_of(test_swrl:'Alex', dul:'Person').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person2) :-
  \+ kb_type_of(test_swrl:'Lea', test_swrl:'Hermaphrodite'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'Hermaphrodite'),
  kb_type_of(test_swrl:'Lea', test_swrl:'Hermaphrodite').

%% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_NonHuman) :-
  \+ kb_type_of(test_swrl:'RedCar', test_swrl:'NonHuman'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'NonHuman'),
  kb_type_of(test_swrl:'RedCar', test_swrl:'NonHuman'),
  \+ kb_type_of(test_swrl:'Fred', test_swrl:'NonHuman').

test(swrl_NonHuman_class_unbound, [nondet]) :-
  kb_type_of(test_swrl:'RedCar', X),
  rdf_equal(X, test_swrl:'NonHuman').

test(swrl_NonHuman_subject_unbound, [nondet]) :-
  kb_type_of(X, test_swrl:'NonHuman'),
  rdf_equal(X, test_swrl:'RedCar').

test(swrl_NonHuman_holds) :-
  kb_triple(test_swrl:'RedCar', rdf:type, test_swrl:'NonHuman').

%% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_area, [nondet]) :-
  \+ kb_triple(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'area'),
  kb_triple(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

%% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_startsWith_load) :-
  \+ kb_triple(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'startsWith').

test(swrl_startsWith_holds1, [nondet]) :-
  kb_triple(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_hasBrother_load) :-
  \+ kb_triple(test_swrl:'Fred', test_swrl:'hasBrother', _),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'brother').

test(swrl_hasBrother_holds) :-
  kb_triple(test_swrl:'Fred', test_swrl:'hasBrother', test_swrl:'Ernest').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_BigRectangle1, [nondet]) :-
  \+ kb_type_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'BigRectangle'),
  kb_type_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle'),
  kb_triple(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_greaterThen) :-
  \+ kb_type_of(test_swrl:'Ernest', test_swrl:'Adult'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'greaterThen'),
  kb_type_of(test_swrl:'Ernest', test_swrl:'Adult').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_exactly) :-
  \+ kb_type_of(test_swrl:'Lea', test_swrl:'Singleton'),
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_load(Filepath,'exactly'),
  kb_type_of(test_swrl:'Lea', test_swrl:'Singleton'),
  \+ kb_type_of(test_swrl:'Fred', test_swrl:'Singleton').

% % % % % % % % % % % % % % % % % % % % % % % %
test(test_rdf_swrl_unload) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  swrl_file_unload(Filepath).

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % Projection of SWRL rule implications into the RDF triple store
% % % % % % % % % % % % % % % % % % % % % % % %

%
swrl_file_project(Filepath,Label) :-
  swrl_file_project(Filepath,Label, []).
swrl_file_project(Filepath,Label,Vars) :-
  swrl_file_parse(Filepath,Rule,Args),
  get_dict(label,Args,Label),
  swrl_project(Rule,Vars).

test(swrl_projection_Driver, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Fred', rdf:type, test_swrl:'Driver'),
  swrl_file_project(Filepath,'Driver'),
  rdf_has(test_swrl:'Fred', rdf:type, test_swrl:'Driver').

test(swrl_projection_Person1, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Alex', rdf:type, dul:'Person'),
  swrl_file_project(Filepath,'Person'),
  rdf_has(test_swrl:'Alex', rdf:type, dul:'Person').

test(swrl_projection_Person2, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Lea', rdf:type, test_swrl:'Hermaphrodite'),
  swrl_file_project(Filepath,'Hermaphrodite'),
  rdf_has(test_swrl:'Lea', rdf:type, test_swrl:'Hermaphrodite').

test(swrl_projection_area, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  swrl_file_project(Filepath,'area'),
  rdf_has(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

test(swrl_projection_startsWith, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _),
  swrl_file_project(Filepath,'startsWith'),
  rdf_has(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _).

test(swrl_projection_BigRectangle1, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'RectangleBig', rdf:type, test_swrl:'BigRectangle'),
  swrl_file_project(Filepath,'BigRectangle'),
  rdf_has(test_swrl:'RectangleBig', rdf:type, test_swrl:'BigRectangle').

test(swrl_projection_greaterThen, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Ernest', rdf:type, test_swrl:'Adult'),
  swrl_file_project(Filepath,'greaterThen'),
  rdf_has(test_swrl:'Ernest', rdf:type, test_swrl:'Adult').

test(swrl_projection_hasBrother, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Fred', test_swrl:'hasBrother', _),
  swrl_file_project(Filepath,'brother'),
  rdf_has(test_swrl:'Fred', test_swrl:'hasBrother', test_swrl:'Ernest').

test(swrl_projection_exactly, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'Lea', rdf:type, test_swrl:'Singleton'),
  swrl_file_project(Filepath,'exactly'),
  rdf_has(test_swrl:'Lea', rdf:type, test_swrl:'Singleton').

test(swrl_projection_NonHuman1, [nondet]) :-
  swrl_file_path(swrlprolog,'test.swrl',Filepath),
  \+ rdf_has(test_swrl:'RedCar', rdf:type, test_swrl:'NonHuman'),
  swrl_file_project(Filepath,'NonHuman'),
  rdf_has(test_swrl:'RedCar', rdf:type, test_swrl:'NonHuman').

% % % % % % % % % % % % % % % % % % % % % % % %
% % % % SWRL rules asserted from human readable expressions
% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_phrase_hasUncle, [nondet]) :-
  \+ kb_triple(test_swrl:'Lea', test_swrl:'hasUncle', test_swrl:'Ernest'),
  swrl_phrase_assert('hasParent(?x, ?y), hasBrother(?y, ?z) -> hasUncle(?x, ?z)'),
  kb_triple(test_swrl:'Lea', test_swrl:'hasUncle', test_swrl:'Ernest').

:- end_tests(swrl).

