:- begin_tripledb_tests(
        'model_OWL',
        'package://knowrob/owl/test/test_owl.owl',
        [ namespace('http://knowrob.org/kb/test_owl#')]
  ).

:- use_module('./OWL.pl').

:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2,
      subproperty_of/2
    ]).

:- use_module(library('db/tripledb'),
    [ tripledb_load/2,
      tripledb_ask/3
    ]).

:- use_module(library('rostest')).

test('is_class') :-
  assert_true(is_class(test:'E')),
  assert_true(is_class(owl:'Thing')),
  assert_true(is_class(owl:'Nothing')),
  assert_false(is_class(test:'DoesNotExists')),
  assert_false(is_class(test:'p')),
  assert_false(is_class(test:'bIndiv')),
  assert_false(is_class(test:'P')).

test('is_restriction/1') :-
  assert_true((
    is_restriction(Entity), 
    tripledb_ask(Entity,owl:'onProperty',test:'s'),
    tripledb_ask(Entity,owl:'allValuesFrom',test:'Range2')
  )),
  assert_true((
    is_restriction(Entity), 
    tripledb_ask(Entity,owl:'onProperty',test:'p'),
    tripledb_ask(Entity,owl:'someValuesFrom',test:'B'),
    % Additionally test if the restriction is the superclass of the correct class
    subclass_of(test:'A2',Entity)
  )),
  % false cases
  assert_false((
    is_restriction(Entity), 
    tripledb_ask(Entity,owl:'onProperty',test:'s'),
    tripledb_ask(Entity,owl:'allValuesFrom',test:'Range1')
  )),
  assert_false((
    is_restriction(Entity), 
    tripledb_ask(Entity,owl:'onProperty',test:'t'),
    tripledb_ask(Entity,owl:'allValuesFrom',test:'Range2')
  )).

test('is_restriction/2') :-
  assert_true((subclass_of(test:'A',R), is_restriction(R,only(test:'s', test:'Range2')))),
  assert_true((subclass_of(test:'A2',R), is_restriction(R,some(test:'p', test:'B')))),
  assert_true((subclass_of(test:'B',R), is_restriction(R,min(test:'r',2,test:'Range1')))),
  assert_true((subclass_of(test:'B',R), is_restriction(R,max(test:'r',5)))),
  assert_true((subclass_of(test:'B',R), is_restriction(R,exactly(test:'s',3)))),
  assert_true((subclass_of(test:'D',R), is_restriction(R,min(test:'s',2)))),
  assert_true((subclass_of(test:'D',R), is_restriction(R,max(test:'s',5,test:'Range1')))),
  assert_true((subclass_of(test:'D',R), is_restriction(R,exactly(test:'s',2,test:'Range2')))),
  % False cases
  assert_false((subclass_of(test:'A',R), is_restriction(R,only(test:'s', test:'Range1')))),
  assert_false((subclass_of(test:'A2',R), is_restriction(R,some(test:'t', test:'B')))),
  assert_false((subclass_of(test:'C',R), is_restriction(R,min(test:'r',2,test:'Range1')))).

test('is_union_of') :-
  assert_true((subclass_of(test:'EUnion',Union),
  %has_description(Union, Desc),
  is_union_of(Union,union_of([test:'E1',test:'E2'])))),
  assert_false(is_complement_of(test:'A',test:'B')).

test('is_intersection_of') :-
  assert_true((subclass_of(test:'CInter',Inter),
  %has_description(Inter, Desc),
  is_intersection_of(Inter,intersection_of([test:'C1',test:'C2'])))),
  assert_false(is_complement_of(test:'A',test:'B')).

test('is_complement_of') :-
  assert_true((subclass_of(test:'DCompl',Compl),
  has_description(Compl, Desc),
  is_complement_of(Compl,Desc))),
  assert_false(is_complement_of(test:'A',test:'B')).

test('is_individual') :-
  assert_false(is_individual(test:'E')),
  assert_false(is_individual(owl:'Thing')),
  assert_false(is_individual(owl:'Nothing')),
  assert_false(is_individual(test:'DoesNotExists')),
  assert_false(is_individual(test:'p')),
  assert_true(is_individual(test:'bIndiv')),
  assert_false(is_individual(test:'P')).

test('is_object_property') :-
  assert_false(is_object_property(test:'E')),
  assert_false(is_object_property(owl:'Thing')),
  assert_false(is_object_property(owl:'Nothing')),
  assert_false(is_object_property(test:'DoesNotExists')),
  assert_true(is_object_property(test:'p')),
  assert_false(is_object_property(test:'bIndiv')),
  assert_false(is_object_property(test:'P')).

test('is_data_property') :-
  assert_false(is_data_property(test:'E')),
  assert_false(is_data_property(owl:'Thing')),
  assert_false(is_data_property(owl:'Nothing')),
  assert_false(is_data_property(test:'DoesNotExists')),
  assert_false(is_data_property(test:'p')),
  assert_false(is_data_property(test:'bIndiv')),
  assert_true(is_data_property(test:'P')).

test('is_functional_property') :-
  assert_true(is_functional_property(test:'p')),
  assert_false(is_functional_property(test:'s')),
  assert_false(is_functional_property(test:'r')),
  assert_false(is_functional_property(test:'DoesNotExists')).

test('is_transitive_property') :-
  assert_false(is_transitive_property(test:'p')),
  assert_false(is_transitive_property(test:'s')),
  assert_true(is_transitive_property(test:'r')),
  assert_false(is_transitive_property(test:'DoesNotExists')).

test('is_symmetric_property') :-
  assert_false(is_symmetric_property(test:'p')),
  assert_true(is_symmetric_property(test:'s')),
  assert_false(is_symmetric_property(test:'r')),
  assert_false(is_symmetric_property(test:'DoesNotExists')).

test('has_inverse_property') :-
  assert_true(has_inverse_property(test:'r',test:'rInv')),
  assert_true(has_inverse_property(test:'rInv2',test:'r')),
  assert_false(has_inverse_property(test:'p',test:'r')).

test('has_property_chain') :-
  assert_true(has_property_chain(test:'t',[test:'t1',test:'t2',test:'t3'])),
  assert_false(has_property_chain(test:'t',[test:'t1',test:'t2'])),
  assert_false(has_property_chain(test:'t',[test:'p',test:'r'])),
  assert_false(has_property_chain(test:'p',[test:'t1',test:'t2',test:'t3'])).

test('has_disjoint_class1') :-
  assert_true(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain2')),
  assert_true(has_disjoint_class(test:'DisjClsChain2',test:'DisjClsChain3')),
  assert_false(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain3')),
  assert_false(has_disjoint_class(test:'A',test:'B')),
  assert_true(has_disjoint_class(test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')).

% Test OWL2 AllDisjointClasses
test('has_disjoint_class2') :-
  assert_true(has_disjoint_class(test:'DisjCls1',test:'DisjCls2')),
  assert_true(has_disjoint_class(test:'DisjCls3',test:'DisjCls2')),
  assert_true(has_disjoint_class(test:'DisjCls1',test:'DisjCls3')),
  assert_true(has_disjoint_class(test:'DisjCls1Sub',test:'DisjCls2Sub')).

test('has_equivalent_class') :-
  assert_true(has_equivalent_class(test:'ASub',test:'ASubEq')),
  assert_true(has_equivalent_class(test:'ASubEq',test:'ASub')),
  assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain2')),
  assert_true(has_equivalent_class(test:'EqClsChain2',test:'EqClsChain3')),
  assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain3')),
  assert_false(has_equivalent_class(test:'A',test:'B')).

test('same_as') :-
  assert_true(same_as(test:'bIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'aIndiv')),
  assert_false(same_as(test:'aIndiv',test:'indiv')),
  assert_false(same_as(test:'aIndiv',test:'doesNotExists')).

:- end_tests(model_OWL).