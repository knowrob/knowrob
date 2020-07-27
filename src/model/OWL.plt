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
  assert_true(is_class(_)),
  assert_true(once((is_class(A), test:'E'=A))).

test('is_restriction/1') :-
  check_is_restriction_true(_,owl:'allValuesFrom',test:'s',test:'Range2'),
  check_is_restriction_false(_,owl:'allValuesFrom',test:'s',test:'Range1'),
  check_is_restriction_false(_,owl:'someValuesFrom',test:'p',test:'Range2'),
  % Argument is unbound
  once((assert_true(subclass_of(test:'A2',A)), check_is_restriction_true(A,owl:'someValuesFrom',test:'p',test:'B'))),
  assert_true(is_restriction(_)).

test('is_restriction/2') :-
  assert_true((subclass_of(test:'A',R),is_restriction(R,only(test:'s', test:'Range2')))),
  assert_true((subclass_of(test:'A2',R2),is_restriction(R2,some(test:'p', test:'B')))),
  assert_true((subclass_of(test:'B',R3),is_restriction(R3,min(test:'r',2,test:'Range1')))),
  assert_true((subclass_of(test:'B',R4),is_restriction(R4,max(test:'r',5)))),
  assert_true((subclass_of(test:'B',R5),is_restriction(R5,exactly(test:'s',3)))),
  assert_true((subclass_of(test:'D',R6),is_restriction(R6,min(test:'s',2)))),
  assert_true((subclass_of(test:'D',R7),is_restriction(R7,max(test:'s',5,test:'Range1')))),
  assert_true((subclass_of(test:'D',R8),is_restriction(R8,exactly(test:'s',2,test:'Range2')))),
  % Left argument is unbound
  assert_true(is_restriction(_,only(test:'s', test:'Range2'))),
  assert_true(once((is_restriction(ARestr,only(test:'s', test:'Range2')), subclass_of(test:'A',ARestr)))),
  % Right argument is unbound
  assert_true((subclass_of(test:'A',R),is_restriction(R,_))),
  assert_true(once((subclass_of(test:'A',A),is_restriction(A,only(test:'s', test:'Range2'))))),
  % Both arguments are unbound
  assert_true(is_union_of(_,_)),
  % Negative Case
  assert_false((subclass_of(test:'A',R), is_restriction(R,only(test:'s', test:'Range1')))),
  assert_false((subclass_of(test:'A2',R),is_restriction(R,some(test:'t', test:'B')))).

test('is_union_of') :-
  assert_true((subclass_of(test:'EUnion',Union1),is_union_of(Union1,union_of([test:'E1',test:'E2'])))),
  assert_false((subclass_of(test:'EUnion',Union2),is_union_of(Union2,union_of([test:'E1'])))),
  assert_false((subclass_of(test:'EUnion',Union3),is_union_of(Union3,union_of([test:'E1',test:'E2',test:'E3'])))),
  % Left argument is unbound
  assert_true(is_union_of(_,union_of([test:'E1',test:'E2']))),
  assert_true(once((is_union_of(AUnion,union_of([test:'E1',test:'E2'])),subclass_of(test:'EUnion',AUnion)))),
  % Right argument is unbound
  assert_true((subclass_of(test:'EUnion',Union4),is_union_of(Union4,_))),
  assert_true(once((subclass_of(test:'EUnion',BUnion),is_union_of(BUnion,Union),Union=union_of([test:'E1',test:'E2'])))),
  % Both arguments are unbound
  assert_true(is_union_of(_,_)),
  % Negative Case
  assert_false(is_union_of(test:'A',test:'B')).

test('is_intersection_of') :-
  assert_true((subclass_of(test:'CInter',CInter1),is_intersection_of(CInter1,intersection_of([test:'C1',test:'C2'])))),
  assert_false((subclass_of(test:'CInter',CInter2),is_intersection_of(CInter2,intersection_of([test:'C1'])))),
  assert_false((subclass_of(test:'CInter',CInter3),is_intersection_of(CInter3,intersection_of([test:'C1',test:'C2',test:'C3'])))),
  % Left argument is unbound
  assert_true(is_intersection_of(_,intersection_of([test:'C1',test:'C2']))),
  ((subclass_of(test:'CInter',AInter),is_intersection_of(A,intersection_of([test:'C1',test:'C2']))) -> assert_unifies(AInter,A) ; true),
  % Right argument is unbound
  assert_true((subclass_of(test:'CInter',CInter4),is_intersection_of(CInter4,_))),
  ((subclass_of(test:'CInter',CInter5),is_intersection_of(CInter5,B)) -> assert_unifies(intersection_of([test:'C1',test:'C2']),B) ; true),
  % Both arguments are unbound
  assert_true(is_intersection_of(_,_)),
  % Negative Case
  assert_false(is_intersection_of(test:'A',test:'B')).

test('is_complement_of') :-
  assert_true((subclass_of(test:'DCompl',Compl1),is_complement_of(Compl1,not(test:'D')))),
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
  assert_true(is_individual(_)),
  assert_true(once((is_individual(A), test:'bIndiv'=A))).

test('is_object_property') :-
  assert_false(is_object_property(test:'E')),
  assert_false(is_object_property(owl:'Thing')),
  assert_false(is_object_property(owl:'Nothing')),
  assert_false(is_object_property(test:'DoesNotExists')),
  assert_true(is_object_property(test:'p')),
  assert_false(is_object_property(test:'bIndiv')),
  assert_false(is_object_property(test:'P')),
  % Argument is unbound
  assert_true(is_object_property(_)),
  assert_true(once((is_object_property(A), test:'p'=A))).

test('is_data_property') :-
  assert_false(is_data_property(test:'E')),
  assert_false(is_data_property(owl:'Thing')),
  assert_false(is_data_property(owl:'Nothing')),
  assert_false(is_data_property(test:'DoesNotExists')),
  assert_false(is_data_property(test:'p')),
  assert_false(is_data_property(test:'bIndiv')),
  assert_true(is_data_property(test:'P')),
  % Argument is unbound
  assert_true(is_data_property(_)),
  assert_true(once((is_data_property(A),test:'P'=A))).

test('is_functional_property') :-
  assert_true(is_functional_property(test:'p')),
  assert_false(is_functional_property(test:'s')),
  assert_false(is_functional_property(test:'r')),
  assert_false(is_functional_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_functional_property(_)),
  assert_true(once((is_functional_property(A),test:'p'=A))).

test('is_transitive_property') :-
  assert_false(is_transitive_property(test:'p')),
  assert_false(is_transitive_property(test:'s')),
  assert_true(is_transitive_property(test:'r')),
  assert_false(is_transitive_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_transitive_property(_)),
  assert_true(once((is_transitive_property(A), test:'r'=A))).

test('is_symmetric_property') :-
  assert_false(is_symmetric_property(test:'p')),
  assert_true(is_symmetric_property(test:'s')),
  assert_false(is_symmetric_property(test:'r')),
  assert_false(is_symmetric_property(test:'DoesNotExists')),
  % Argument is unbound
  assert_true(is_symmetric_property(_)),
  assert_true(once((is_symmetric_property(A), test:'s'=A))).

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
  assert_true(once((has_property_chain(A,[test:'t1',test:'t2',test:'t3']), A=test:'t'))), 
  % Right argument is unbound
  assert_true(has_property_chain(test:'t',_)),
  assert_true(once((has_property_chain(test:'t',B), [test:'t1',test:'t2',test:'t3']=B))),
  % Both arguments are unbound
  assert_true(has_property_chain(_,_)),!.

test('has_disjoint_class1', 
    [ blocked('The transitive(subclass_of(B,Disjoint) call in unify_disjoint of OWL.pl seems to not return the transitive closure of all subclasses') ]) :-
  assert_true(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain2')),
  assert_true(has_disjoint_class(test:'DisjClsChain2',test:'DisjClsChain1')),
  assert_true(has_disjoint_class(test:'DisjClsChain2',test:'DisjClsChain3')),
  assert_false(has_disjoint_class(test:'DisjClsChain1',test:'DisjClsChain3')),
  assert_true(has_disjoint_class(test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')),  
  assert_true(has_disjoint_class(test:'DisjClsChain1SubSub',test:'DisjClsChain2Sub')),
  % Left argument is unbound
  assert_true(has_disjoint_class(_,test:'DisjClsChain2')),
  assert_true((findall(A, has_disjoint_class(A,test:'DisjClsChain2'), AList), member(test:'DisjClsChain1', AList))), 
  assert_true((findall(B, has_disjoint_class(B,test:'DisjClsChain2'), BList), member(test:'DisjClsChain3', BList))), 
  (findall(C, has_disjoint_class(C,test:'DisjClsChain1Sub'), CList), assert_true(member(test:'DisjClsChain2Sub', CList))),
  (findall(C, has_disjoint_class(C,test:'DisjClsChain1SubSub'), CList), assert_true(member(test:'DisjClsChain2SubSub', CList))),
  % Right argument is unbound
  assert_true(has_disjoint_class(test:'DisjClsChain2',_)),
  assert_true((findall(D, has_disjoint_class(test:'DisjClsChain2',D), DList), member(test:'DisjClsChain1', DList))), 
  assert_true((findall(E, has_disjoint_class(test:'DisjClsChain2',E), EList), member(test:'DisjClsChain3', EList))),
  assert_true((findall(F, has_disjoint_class(test:'DisjClsChain1Sub',F), FList), member(test:'DisjClsChain2Sub', FList))),
  assert_true((findall(F, has_disjoint_class(test:'DisjClsChain1SubSub',F), FList), member(test:'DisjClsChain2Sub', FList))),
  % Both arguments are unbound
  assert_true(has_disjoint_class(_,_)),  
  % Negative Case
  assert_false(has_disjoint_class(test:'A',test:'B')).


% Test OWL2 AllDisjointClasses
test('has_disjoint_class2') :-
  assert_true(has_disjoint_class(test:'DisjCls1',test:'DisjCls2')),
  assert_true(has_disjoint_class(test:'DisjCls3',test:'DisjCls2')),
  assert_true(has_disjoint_class(test:'DisjCls1',test:'DisjCls3')),
  assert_true(has_disjoint_class(test:'DisjCls1Sub',test:'DisjCls2Sub')),
  assert_true(has_disjoint_class(test:'DisjCls1Sub',test:'DisjCls3')),
  assert_true(has_disjoint_class(test:'DisjCls1SubSub',test:'DisjCls2Sub')),
  assert_true(has_disjoint_class(test:'DisjCls1SubSub',test:'DisjCls3')),
  % Left argument is unbound
  assert_true(has_disjoint_class(_,test:'DisjCls2')),
  (findall(A, has_disjoint_class(A,test:'DisjCls2'), AList), assert_true(member(test:'DisjCls1', AList))), 
  (findall(B, has_disjoint_class(B,test:'DisjCls2'), BList), assert_true(member(test:'DisjCls3', BList))),
  (findall(C, has_disjoint_class(C,test:'DisjCls1Sub'), CList), assert_true(member(test:'DisjCls2Sub', CList))),
  (findall(D, has_disjoint_class(D,test:'DisjCls1Sub'), DList), assert_true(member(test:'DisjCls3', DList))),
  % Right argument is unbound
  assert_true(has_disjoint_class(test:'DisjCls2',_)),
  (findall(E, has_disjoint_class(test:'DisjCls2',E), EList), assert_true(member(test:'DisjCls1', EList))),
  (findall(F, has_disjoint_class(test:'DisjCls2',F), FList), assert_true(member(test:'DisjCls3', FList))),
  (findall(E, has_disjoint_class(test:'DisjCls1Sub',E), EList), assert_true(member(test:'DisjCls2Sub', EList))),
  (findall(F, has_disjoint_class(test:'DisjCls1Sub',F), FList), assert_true(member(test:'DisjCls3', FList))).

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
  assert_true(once((has_equivalent_class(A,test:'EqClsChain2'), A=test:'EqClsChain1'))), 
  assert_true(once((has_equivalent_class(A,test:'EqClsChain2'), A=test:'EqClsChain3'))), 
  % Right argument is unbound
  assert_true(has_equivalent_class(test:'ASub',_)),
  assert_true(once((has_equivalent_class(test:'EqClsChain2',B), test:'EqClsChain1'=B))),
  assert_true(once((has_equivalent_class(test:'EqClsChain2',B), test:'EqClsChain3'=B))),
  % Both arguments are unbound
  assert_true(has_equivalent_class(_,_)),
  % Negative Case
  assert_false(has_equivalent_class(test:'A',test:'B')),!.

test('same_as') :-
  assert_true(same_as(test:'bIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'aIndiv')),
  assert_false(same_as(test:'aIndiv',test:'indiv')),
  % Make sure that there is only one individual (except the individual itself) that is the same as the tested individual
  findall(X, (same_as(X,test:'aIndiv')), Xs),
  assert_unifies(Xs,[_,_]),
  % Left argument is unbound
  assert_true(same_as(_,test:'bIndiv')),
  assert_true(once((same_as(A,test:'aIndiv'),A=test:'bIndiv'))), 
  % Right argument is unbound
  assert_true(same_as(test:'aIndiv',_)),
  assert_true(once((same_as(test:'aIndiv',B), test:'bIndiv'=B))),
  % Both arguments are unbound
  assert_true(same_as(_,_)),
  % Negative Case
  assert_false(same_as(test:'aIndiv',test:'doesNotExists')).

:- end_tests(model_OWL).
