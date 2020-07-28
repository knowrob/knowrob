
:- use_module('./class.pl').

:- use_module(library('model/OWL'),
    [ has_description/2,
      is_class/1,
      is_object_property/1,
      is_functional_property/1,
      is_intersection_of/2,
      has_equivalent_class/2,
      has_inverse_property/2
    ]).

:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2,
      subproperty_of/2
    ]).

:- begin_tripledb_tests(
        'plowl_class',
        'package://knowrob/owl/test/test_owl.owl',
        [ namespace('http://knowrob.org/kb/test_owl#')]
  ).

:- use_module(library('rostest')).

test('owl_subclass_of') :-
  % Equivalent classes
  % Direct equivalent classes
  rostest:assert_true(plowl_class:owl_subclass_of(test:'E',test:'F')),
  rostest:assert_true(plowl_class:owl_subclass_of(test:'F',test:'E')),
  % Indirect equivalent classes
  rostest:assert_true(plowl_class:owl_subclass_of(test:'EqClsChain1',test:'EqClsChain3')),
  rostest:assert_false(plowl_class:owl_subclass_of(test:'A',test:'E')),
  % Intersection as super class
  subclass_of(test:'CInter',Intersection), is_intersection_of(Intersection,_),
  rostest:assert_true(plowl_class:owl_subclass_of(Intersection,test:'C1')),
  rostest:assert_true(plowl_class:owl_subclass_of(Intersection,test:'C2')),
  rostest:assert_true(plowl_class:owl_subclass_of(Intersection,test:'C')),
  rostest:assert_false(plowl_class:owl_subclass_of(Intersection,test:'B')),!.

test('owl_property_range') :-
  % Unbound first variable
  rostest:assert_true(plowl_class:owl_property_range(_,test:'r',test:'Range1')),  
  % Unbound second variable
  rostest:assert_true(plowl_class:owl_property_range(test:'A',_,test:'Range1')),  
  % Unbound thrid variable
  rostest:assert_true(plowl_class:owl_property_range(test:'A',test:'r',_)),  
  % General range restriction on role
  rostest:assert_true(plowl_class:owl_property_range(owl:'Thing',test:'r',test:'Range1')),
  % Correct Class
  rostest:assert_true(plowl_class:owl_property_range(test:'A',test:'s',test:'Range2')),
  % Subclasses of correct Class
  rostest:assert_true(plowl_class:owl_property_range(test:'ASubRange',test:'s',test:'Range2')),
  rostest:assert_true(plowl_class:owl_property_range(test:'ASubSubRange',test:'s',test:'Range2')),
  % Incorrect classes
  rostest:assert_false(plowl_class:owl_property_range(test:'B',test:'s',test:'Range2')),
  rostest:assert_false(plowl_class:owl_property_range(owl:'Thing',test:'s',test:'Range2')),
  % Range of class in the intersection
  rostest:assert_true(plowl_class:owl_property_range(intersection_of([test:'C1',test:'C2',test:'B']),test:'p',test:'RangeC2')),
  % General range
  rostest:assert_true(plowl_class:owl_property_range(intersection_of([test:'C1',test:'C2',test:'B']),test:'r',test:'Range1')),
  % Negative Case for intersectionWo
  rostest:assert_false(plowl_class:owl_property_range(intersection_of([test:'C1',test:'C2',test:'B']),test:'s',test:'Range2')).

test('owl_property_cardinality with class in range axiom') :-
  % Min restraint with class in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'C',test:'t1',test:'Range1',1,_)),
  % Max Restraint with class in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'E2',test:'t2',test:'Range2',_,4)),
  % Min Restraint with class and max restraint with class in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'G1',test:'p',test:'Range3',3,8)),
  % Exactly tests.
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'D',test:'s',test:'Range2',2,2)).

test('owl_property_cardinality with owl:Thing in range axiom', 
    [ blocked('owl_property_cardinality is not handled in the correct way with owl:Thing as Range') ]) :-
  % Min restraint with owl:Thing in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'E1',test:'t1',test:'Range1',2,_)),
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'E1',test:'t1',owl:'Thing',2,_)),
  % Max Restraint with owl:Thing in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'E1',test:'p',test:'Range2',_,4)),
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'E1',test:'p',test:'Range2',_,4)),
  % Min Restraint with class and max restraint with owl:Thing in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'G2',test:'t',test:'Range1',8,15)),
  % Min Restraint with owl:Thing and max restraint with owl:Thing in range axiom
  rostest:assert_true(plowl_class:owl_property_cardinality(test:'B',test:'r',test:'Range1',2,5)).

:- end_tests(plowl_class).
