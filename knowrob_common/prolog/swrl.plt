%%
%% Copyright (C) 2017 by Daniel Beßler
%%
%% This file contains tests for the swrl module
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%

:- begin_tests(swrl).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('swrl')).

:- owl_parser:owl_parse('package://knowrob_common/owl/swrl_test.owl').

:- rdf_db:rdf_register_prefix(test_swrl, 'http://knowrob.org/kb/swrl_test#', [keep(true)]).
:- rdf_db:rdf_register_prefix(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).
:- rdf_db:rdf_register_prefix(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).

test_rule_id(Id, Descr) :-
  once(( rdf_has(Descr, rdfs:label, literal(type(_,Id))) ;
         rdf_has(Descr, rdfs:label, literal(Id)) )),
  rdf_has(Descr, rdf:type, swrl:'Imp').

test(swrl_parse_rules) :-
  forall( rdf_has(Descr, rdf:type, swrl:'Imp'), (
    rdf_swrl_rule(Descr, Head :- Body),
    Head \= [], Body \= []
  )).

rdf_swrl_load_named_rule(Id) :-
  test_rule_id(Id, Descr),
  rdf_swrl_load(Descr).

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Driver_load, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'Fred', test_swrl:'Driver'),
  rdf_swrl_load_named_rule('Driver').

test(swrl_Driver, [nondet]) :-
  swrl_individual_of(test_swrl:'Fred', test_swrl:'Driver').

test(swrl_Driver_class_unbound, [nondet]) :-
  swrl_individual_of(test_swrl:'Fred', X),
  rdf_equal(X, test_swrl:'Driver').

test(swrl_Driver_subject_unbound, [nondet]) :-
  swrl_individual_of(X, test_swrl:'Driver'),
  rdf_equal(X, test_swrl:'Fred').

test(swrl_Driver_during, [nondet]) :-
  current_time(Now),
  owl_individual_of_during(test_swrl:'Fred', test_swrl:'Driver', Now).

test(swrl_Driver_holds, [nondet]) :-
  holds(test_swrl:'Fred', rdf:type, test_swrl:'Driver').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_Person1, [nondet]) :-
  \+ swrl_holds(test_swrl:'Alex', rdf:type, test_swrl:'Person'),
  rdf_swrl_load_named_rule('Person'),
  swrl_holds(test_swrl:'Alex', rdf:type, test_swrl:'Person').

test(swrl_Person2, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'Lea', test_swrl:'Hermaphrodite'),
  rdf_swrl_load_named_rule('Person2'),
  swrl_individual_of(test_swrl:'Lea', test_swrl:'Hermaphrodite').

test(swrl_NonHuman1, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'RedCar', test_swrl:'NonHuman'),
  rdf_swrl_load_named_rule('NonHuman'),
  swrl_individual_of(test_swrl:'RedCar', test_swrl:'NonHuman'),
  \+ swrl_individual_of(test_swrl:'Fred', test_swrl:'NonHuman').

test(swrl_area, [nondet]) :-
  \+ swrl_holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  rdf_swrl_load_named_rule('area'),
  swrl_holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

test(swrl_startsWith, [nondet]) :-
  \+ swrl_holds(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _),
  rdf_swrl_load_named_rule('startsWith'),
  swrl_holds(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _).

test(swrl_hasBrother, [nondet]) :-
  \+ swrl_holds(test_swrl:'Fred', test_swrl:'hasBrother', _),
  rdf_swrl_load_named_rule('brother'),
  swrl_holds(test_swrl:'Fred', test_swrl:'hasBrother', test_swrl:'Ernest').

test(swrl_BigRectangle1, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle'),
  rdf_swrl_load_named_rule('BigRectangle'),
  swrl_individual_of(test_swrl:'RectangleBig', test_swrl:'BigRectangle'),
  swrl_holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _).

test(swrl_greaterThen, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'Ernest', test_swrl:'Adult'),
  rdf_swrl_load_named_rule('greaterThen'),
  swrl_individual_of(test_swrl:'Ernest', test_swrl:'Adult').

test(swrl_exactly, [nondet]) :-
  \+ swrl_individual_of(test_swrl:'Lea', test_swrl:'Singleton'),
  rdf_swrl_load_named_rule('exactly'),
  swrl_individual_of(test_swrl:'Lea', test_swrl:'Singleton'),
  \+ swrl_individual_of(test_swrl:'Fred', test_swrl:'Singleton').

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_assert_rules) :- rdf_swrl_load.

% % % % % % % % % % % % % % % % % % % % % % % %
test(swrl_holds, [nondet]) :-
  holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  holds(test_swrl:'RectangleBig', test_swrl:'hasAreaInSquareMeters', _),
  holds(test_swrl:'Fred', test_swrl:'hasInternationalNumber', _),
  holds(test_swrl:'Fred', test_swrl:'hasBrother', test_swrl:'Ernest').

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
      [ class(test_swrl:'Person',var(p)),
      property(var(p), test_swrl:'hasCar', literal(type(xsd:'boolean',true))) ]
  ).

test(swrl_parse_DriverFred, [nondet]) :-
  test_swrl_parse(
    ['Person(Fred), hasCar(Fred, true)', '->', 'Driver(Fred)'],
    [ class(test_swrl:'Driver',test_swrl:'Fred') ] :-
      [ class(test_swrl:'Person',test_swrl:'Fred'),
        property(test_swrl:'Fred', test_swrl:'hasCar', literal(type(xsd:'boolean',true))) ]
  ).

test(swrl_parse_brother, [nondet]) :-
  test_swrl_parse(
    ['Person(?p), hasSibling(?p, ?s), Man(?s)', '->', 'hasBrother(?p, ?s)'],
    [ property(var(p), test_swrl:'hasBrother', var(s)) ] :-
      [ class(test_swrl:'Person', var(p)),
        property(var(p), test_swrl:'hasSibling', var(s)),
        class(test_swrl:'Man',var(s)) ]
  ).

test(swrl_parse_startsWith, [nondet]) :-
  test_swrl_parse(
    [ 'Person(?p), hasNumber(?p, ?number), startsWith(?number, "+")',
      '->',
      'hasInternationalNumber(?p, true)' ],
    [ property(var(p), test_swrl:'hasInternationalNumber', literal(type(xsd:boolean,true))) ] :-
      [ class(test_swrl:'Person', var(p)),
        property(var(p), test_swrl:'hasNumber', var(number)),
        startsWith(var(number),+) ]
  ).

test(swrl_parse_exactly, [nondet]) :-
  test_swrl_parse(
    ['Person(?x), (hasSibling exactly 0 Person)(?x)', '->', 'Singleton(?x)'],
    [ class(test_swrl:'Singleton', var(x)) ] :-
      [ class(test_swrl:'Person', var(x)),
        class(exactly(0, test_swrl:'hasSibling', test_swrl:'Person'), var(x)) ]
  ).

test(swrl_parse_Person1, [nondet]) :-
  test_swrl_parse(
    ['(Man or Woman)(?x)', '->', 'Person(?x)'],
    [ class(test_swrl:'Person', var(x)) ] :-
      [ class(oneOf([test_swrl:'Man',test_swrl:'Woman']), var(x)) ]
  ).

test(swrl_parse_Person2, [nondet]) :-
  test_swrl_parse(
    ['(Man and Woman and Person)(?x)', '->', 'Hermaphrodite(?x)'],
    [ class(test_swrl:'Hermaphrodite', var(x)) ] :-
      [ class(allOf([test_swrl:'Man',test_swrl:'Woman',test_swrl:'Person']), var(x)) ]
  ).

test(swrl_parse_NonHuman, [nondet]) :-
  test_swrl_parse(
    ['(not Person)(?x)', '->', 'NonHuman(?x)'],
    [ class(test_swrl:'NonHuman', var(x)) ] :-
      [ class(not(test_swrl:'Person'), var(x)) ]
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
      [ class(test_swrl:'Person', var(p)),
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
    [ class(test_swrl:'Person', var(x)) ] :-
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

:- end_tests(swrl).

