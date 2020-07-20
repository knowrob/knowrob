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
:- use_module(library('semweb/rdf_db')).

:- begin_tripledb_tests(
        'model_OWL',
        'package://knowrob/owl/test/test_owl.owl',
        [ namespace('http://knowrob.org/kb/test_owl#')]
  ).

:- rdf_meta check_is_restriction_true(t,t,t,t),
            check_is_restriction_false(t,t,t,t).

check_is_restriction_true(Entity, OWLRestriction, TestProperty, TestRange) :-
  assert_true(is_restriction(Entity)), 
  assert_true(tripledb_ask(Entity,owl:'onProperty',TestProperty)),
  assert_true(tripledb_ask(Entity,OWLRestriction,TestRange)).

check_is_restriction_false(Entity, OWLRestriction, TestProperty, TestRange) :-
  assert_true(is_restriction(Entity)), 
  assert_true(tripledb_ask(Entity,owl:'onProperty',TestProperty)),
  assert_false(tripledb_ask(Entity,OWLRestriction,TestRange)).

test('is_class') :-
  assert_true(is_class(test:'E')),
  assert_true(is_class(owl:'Thing')),
  assert_true(is_class(owl:'Nothing')),
  assert_false(is_class(test:'DoesNotExists')),
  assert_false(is_class(test:'p')),
  assert_false(is_class(test:'bIndiv')),
  assert_false(is_class(test:'P')),
  % Argument is unbound
  assert_true(is_class(_)).

test('is_restriction/1') :-
  check_is_restriction_true(_,owl:'allValuesFrom',test:'s',test:'Range2'),
  check_is_restriction_true(Entity,owl:'someValuesFrom',test:'p',test:'B'),
  assert_true(subclass_of(test:'A2',Entity)),
  check_is_restriction_false(_,owl:'allValuesFrom',test:'s',test:'Range1'),
  check_is_restriction_false(_,owl:'someValuesFrom',test:'p',test:'Range2').

test('is_restriction/2') :-
  assert_true((subclass_of(test:'A',R))),
  assert_true(is_restriction(R,only(test:'s', test:'Range2'))),
  assert_true((subclass_of(test:'A2',R))), 
  assert_true(is_restriction(R,some(test:'p', test:'B'))),
  assert_true((subclass_of(test:'B',R))), 
  assert_true(is_restriction(R,min(test:'r',2,test:'Range1'))),
  assert_true((subclass_of(test:'B',R))), 
  assert_true(is_restriction(R,max(test:'r',5))),
  assert_true((subclass_of(test:'B',R))), 
  assert_true(is_restriction(R,exactly(test:'s',3))),
  assert_true((subclass_of(test:'D',R))), 
  assert_true(is_restriction(R,min(test:'s',2))),
  assert_true((subclass_of(test:'D',R))), 
  assert_true(is_restriction(R,max(test:'s',5,test:'Range1'))),
  assert_true((subclass_of(test:'D',R))),
  assert_true(is_restriction(R,exactly(test:'s',2,test:'Range2'))),
  % Negative Case
  assert_true((subclass_of(test:'A',R))), 
  assert_false(is_restriction(R,only(test:'s', test:'Range1'))),
  assert_true((subclass_of(test:'A2',R))), 
  assert_false(is_restriction(R,some(test:'t', test:'B'))).

test('is_union_of') :-
  assert_true(subclass_of(test:'EUnion',Union)),
  %has_description(Union, Desc),
  assert_true(is_union_of(Union,is_union_of([test:'E1',test:'E2']))),
  assert_false(is_union_of(test:'A',test:'B')).

test('is_intersection_of') :-
  assert_true(subclass_of(test:'CInter',CInter)),
  %has_description(Inter, Desc),
  assert_true(is_intersection_of(CInter,intersection_of([test:'C1',test:'C2']))),
    % Left argument is unbound
  %assert_true(is_complement_of(_,not(test:'D'))),
  %((is_intersection_of(test:'CInter',Compl), is_complement_of(A,not(test:'D'))) -> assert_unifies(Compl,A) ; true),
  % Right argument is unbound
  %assert_true(is_complement_of(Compl,_)),
  %((subclass_of(test:'DCompl',Compl), is_complement_of(Compl,B)) -> assert_unifies(not(test:'D'),B) ; true),
  % Both arguments are unbound
  %assert_true(is_intersection_of(_,_)),
  % Negative Case
  assert_false(is_intersection_of(test:'A',test:'B')).

test('is_complement_of') :-
  assert_true(subclass_of(test:'DCompl',Compl)),
  assert_true(is_complement_of(Compl,not(test:'D'))),
  % Left argument is unbound
  assert_true(is_complement_of(_,not(test:'D'))),
  ((subclass_of(test:'DCompl',Compl), is_complement_of(A,not(test:'D'))) -> assert_unifies(Compl,A) ; true),
  % Right argument is unbound
  assert_true(is_complement_of(Compl,_)),
  ((subclass_of(test:'DCompl',Compl), is_complement_of(Compl,B)) -> assert_unifies(not(test:'D'),B) ; true),
  % Both arguments are unbound
  assert_true(is_complement_of(_,_)),
  % Negative Case
  assert_false(is_complement_of(test:'A',test:'B')).

test('is_individual') :-
  assert_false(is_individual(test:'E')),
  assert_false(is_individual(owl:'Thing')),
  assert_false(is_individual(owl:'Nothing')),
  assert_false(is_individual(test:'DoesNotExists')),
  assert_false(is_individual(test:'p')),
  assert_true(is_individual(test:'bIndiv')),
  assert_false(is_individual(test:'P')),
  % Argument is unbound
  assert_true(is_individual(_)).

test('is_object_property') :-
  assert_false(is_object_property(test:'E')),
  assert_false(is_object_property(owl:'Thing')),
  assert_false(is_object_property(owl:'Nothing')),
  assert_false(is_object_property(test:'DoesNotExists')),
  assert_true(is_object_property(test:'p')),
  assert_false(is_object_property(test:'bIndiv')),
  assert_false(is_object_property(test:'P')),
  % Argument is unbound
  assert_true(is_object_property(_)).

test('is_data_property') :-
  assert_false(is_data_property(test:'E')),
  assert_false(is_data_property(owl:'Thing')),
  assert_false(is_data_property(owl:'Nothing')),
  assert_false(is_data_property(test:'DoesNotExists')),
  assert_false(is_data_property(test:'p')),
  assert_false(is_data_property(test:'bIndiv')),
  assert_true(is_data_property(test:'P')),
  % Argument is unbound
  assert_true(is_data_property(_)).

test('is_functional_property') :-
  assert_true(is_functional_property(test:'p')),
  assert_false(is_functional_property(test:'s')),
  assert_false(is_functional_property(test:'r')),
  assert_false(is_functional_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_functional_property(_)).

test('is_transitive_property') :-
  assert_false(is_transitive_property(test:'p')),
  assert_false(is_transitive_property(test:'s')),
  assert_true(is_transitive_property(test:'r')),
  assert_false(is_transitive_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_transitive_property(_)).

test('is_symmetric_property') :-
  assert_false(is_symmetric_property(test:'p')),
  assert_true(is_symmetric_property(test:'s')),
  assert_false(is_symmetric_property(test:'r')),
  assert_false(is_symmetric_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_symmetric_property(_)).

test('has_inverse_property') :-
  assert_true(has_inverse_property(test:'r',test:'rInv')),
  assert_true(has_inverse_property(test:'rInv2',test:'r')),
  % Left argument is unbound
  assert_true(has_inverse_property(test:'r',_)),
  % Right argument is unbound
  assert_true(has_inverse_property(_,test:'rInv')),
  % Both arguments are unbound
  assert_true(has_inverse_property(_,_)),
  % Negative Case
  assert_false(has_inverse_property(test:'p',test:'r')).

test('has_property_chain') :-
  assert_true(has_property_chain(test:'t',[test:'t1',test:'t2',test:'t3'])),
  assert_false(has_property_chain(test:'t',[test:'t1',test:'t2'])),
  assert_false(has_property_chain(test:'t',[test:'p',test:'r'])),
  assert_false(has_property_chain(test:'p',[test:'t1',test:'t2',test:'t3'])),  
  % Left argument is unbound
  assert_true(has_property_chain(_,[test:'t1',test:'t2',test:'t3'])),
  (has_property_chain(A,[test:'t1',test:'t2',test:'t3']) *-> assert_unifies(A,test:'t') ; true), 
  % Right argument is unbound
  assert_true(has_property_chain(test:'t',_)),
  (has_property_chain(test:'t',B) *-> assert_unifies([test:'t1',test:'t2',test:'t3'],B) ; true),
  % Both arguments are unbound
  assert_true(has_property_chain(_,_)),!.

test('has_disjoint_class1') :-
  assert_true(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain2')),
  assert_true(has_disjoint_class(test:'DisjClsChain2',test:'DisjClsChain1')),
  assert_true(has_disjoint_class(test:'DisjClsChain2',test:'DisjClsChain3')),
  assert_false(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain3')),
  assert_true(has_disjoint_class(test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')),  
  % Left argument is unbound
  assert_true(has_disjoint_class(_,test:'DisjClsChain2')),
  (findall(A, has_disjoint_class(A,test:'DisjClsChain2'), AList) *-> assert_true(member(test:'DisjClsChain1', AList)) ; true), 
  (findall(B, has_disjoint_class(B,test:'DisjClsChain2'), BList) *-> assert_true(member(test:'DisjClsChain3', BList)) ; true), 
  % Right argument is unbound
  assert_true(has_disjoint_class(test:'DisjClsChain2',_)),
  (findall(C, has_disjoint_class(test:'DisjClsChain2',C), CList) *-> assert_true(member(test:'DisjClsChain1', CList)) ; true), 
  (findall(D, has_disjoint_class(test:'DisjClsChain2',D), DList) *-> assert_true(member(test:'DisjClsChain3', DList)) ; true),
  % Both arguments are unbound
  assert_true(has_disjoint_class(_,_)),  
  % Negative Case
  assert_false(has_disjoint_class(test:'A',test:'B')).


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
  % Make sure that there is only one class that is equivalent to the tested class
  findall(X, has_equivalent_class(X,test:'ASubEq'), Xs),
  assert_unifies(Xs,[_]),
  % Left argument is unbound
  assert_true(has_equivalent_class(_,test:'ASubEq')),
  %(has_equivalent_class(A,test:'EqClsChain2') *-> assert_unifies(A,test:'EqClsChain1') ; true), 
  %(has_equivalent_class(A,test:'EqClsChain2') *-> assert_unifies(A,test:'EqClsChain3') ; true), 
  % Right argument is unbound
  assert_true(has_equivalent_class(test:'ASub',_)),
  %(has_equivalent_class(test:'EqClsChain2',B) *-> assert_unifies(test:'EqClsChain1',B) ; true),
  %(has_equivalent_class(test:'EqClsChain2',B) *-> assert_unifies(test:'EqClsChain3',B) ; true),
  % Both arguments are unbound
  assert_true(has_equivalent_class(_,_)),
  % Negative Case
  assert_false(has_equivalent_class(test:'A',test:'B')),!.

test('same_as') :-
  assert_true(same_as(test:'bIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'aIndiv')),
  assert_false(same_as(test:'aIndiv',test:'indiv')),
  assert_false(same_as(test:'aIndiv',test:'doesNotExists')).

:- end_tests(model_OWL).
