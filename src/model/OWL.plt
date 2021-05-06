:- use_module('./OWL.pl').
:- use_module(library('model/RDFS'),
    [ subclass_of/2,
      subproperty_of/2
    ]).
:- use_module(library('rostest')).
:- use_module(library('semweb/rdf_db')).

:- begin_rdf_tests(
        'model_OWL',
        'package://knowrob/owl/test/test_owl.owl',
        [ namespace('http://knowrob.org/kb/test_owl#')]
  ).

:- rdf_meta check_is_restriction_true(t,t,t,t),
            check_is_restriction_false(t,t,t,t).

check_is_restriction_true(Entity, OWLRestriction, TestProperty, TestRange) :-
  assert_true(is_restriction(Entity)), 
  assert_true(kb_call(triple(Entity,owl:'onProperty',TestProperty))),
  assert_true(kb_call(triple(Entity,OWLRestriction,TestRange))).

check_is_restriction_false(Entity, OWLRestriction, TestProperty, TestRange) :-
  assert_true(is_restriction(Entity)), 
  assert_true(kb_call(triple(Entity,owl:'onProperty',TestProperty))),
  assert_false(kb_call(triple(Entity,OWLRestriction,TestRange))).

test('is_class(?)') :-
  assert_true(is_class(test:'E')),
  assert_true(is_class(owl:'Thing')),
  assert_true(is_class(owl:'Nothing')),
  assert_false(is_class(test:'DoesNotExists')),
  assert_false(is_class(test:'p')),
  assert_false(is_class(test:'bIndiv')),
  assert_false(is_class(test:'P')),
  % Argument is unbound
  assert_true(is_class(_)).

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

test('has_property_chain(+,+)') :-
  	assert_true(has_property_chain(test:'t',
  		[test:'t1',test:'t2',test:'t3'])),
  	assert_false(has_property_chain(test:'t',
  		[test:'t2',test:'t3'])).

test('has_property_chain(-,+)') :-
  assert_true(has_property_chain(_,[test:'t1',test:'t2',test:'t3'])),
  %
  (  has_property_chain(A,[test:'t1',test:'t2',test:'t3'])
  -> assert_equals(A, test:'t')
  ;  true
  ).

test('has_property_chain(+,-)') :-
  assert_true(has_property_chain(test:'t',_)),
  findall(X, has_property_chain(test:'t',X), List),
  length(List,NumChains),
  assert_equals(NumChains,1),
  %
  (  has_property_chain(test:'t',B)
  -> assert_equals(B,[test:'t1',test:'t2',test:'t3'])
  ;  true
  ).

test('has_property_chain(-,-)') :-
  assert_true(once(has_property_chain(_,_))).

test('is_complement_of(+,+)') :-
  assert_true(kb_call((
    subclass_of(test:'DCompl',Compl1),
    is_complement_of(Compl1,complement_of(test:'D'))
  ))),
  % Negative Case
  assert_false(is_complement_of(test:'A',test:'B')).

test('is_complement_of(-,+)') :-
  % Left argument is unbound
  assert_true(is_complement_of(_,complement_of(test:'D'))),
  (
    kb_call((
      subclass_of(test:'DCompl',Compl), 
      is_complement_of(A,complement_of(test:'D'))
    ))
    -> assert_unifies(Compl,A) 
    ;  true
  ).
 
test('is_complement_of(+,-)') :-
  % Right argument is unbound
  assert_true(is_complement_of(Compl,_)),
  (
    kb_call((
      subclass_of(test:'DCompl',Compl), 
      is_complement_of(Compl,B)
    ))
    -> assert_unifies(complement_of(test:'D'),B) 
    ;  true
  ).

test('is_complement_of(-,-)') :-
  assert_true(once(is_complement_of(_,_))).

test('is_restriction(?)') :-
  check_is_restriction_true(_,owl:'allValuesFrom',test:'s',test:'Range2'),
  check_is_restriction_false(_,owl:'allValuesFrom',test:'s',test:'Range1'),
  check_is_restriction_false(_,owl:'someValuesFrom',test:'p',test:'Range2'),
  % Argument is unbound
  once((
    assert_true(subclass_of(test:'A2',A)), 
    check_is_restriction_true(A,owl:'someValuesFrom',test:'p',test:'B')
  )),
  assert_true(is_restriction(_)).

test('is_restriction(+,+)') :-
  assert_true(kb_call((
    subclass_of(test:'A',R),
    is_restriction(R,only(test:'s', test:'Range2'))
  ))),
  assert_true(kb_call((
    subclass_of(test:'A2',R2),
    is_restriction(R2,some(test:'p', test:'B'))
  ))),
  assert_true(kb_call((
    subclass_of(test:'B',R3),
    is_restriction(R3,min(test:'r',2,test:'Range1'))
  ))),
  assert_true(kb_call((
    subclass_of(test:'B',R4),
    is_restriction(R4,max(test:'r',5))
  ))),
  assert_true(kb_call((
    subclass_of(test:'B',R5),
    is_restriction(R5,exactly(test:'s',3))
  ))),
  assert_true(kb_call((
    subclass_of(test:'D',R6),
    is_restriction(R6,min(test:'s',2))
  ))),
  assert_true(kb_call((
    subclass_of(test:'D',R7),
    is_restriction(R7,max(test:'s',5,test:'Range1'))
  ))),
  assert_true(kb_call((
    subclass_of(test:'D',R8),
    is_restriction(R8,exactly(test:'s',2,test:'Range2'))
  ))),
  % Negative Case
  assert_false(kb_call((
    subclass_of(test:'A',R), 
    is_restriction(R,only(test:'s', test:'Range1'))
  ))),
  assert_false(kb_call((
    subclass_of(test:'A2',R),
    is_restriction(R,some(test:'t', test:'B'))
  ))).

test('is_restriction(-,+)') :-
  % Left argument is unbound
  assert_true(is_restriction(_,only(test:'s', test:'Range2'))),
  assert_true(kb_call(once((
      is_restriction(R,only(test:'s', test:'Range2')),
      subclass_of(test:'A',R)
  )))).

test('is_restriction(+,-)') :-
  % Right argument is unbound
  assert_true((subclass_of(test:'A',R),is_restriction(R,_))),
  assert_true(kb_call(once((
      subclass_of(test:'A',A),
      is_restriction(A,only(test:'s', test:'Range2'))
  )))).

% TODO
%test('is_restriction(-,-)', throws(instantiation_error(_))) :-
%  assert_true(is_restriction(_,_)).

test('is_union_of(+,+)') :-
  assert_true(kb_call((
    subclass_of(test:'EUnion',Union1),
    is_union_of(Union1,union_of([test:'E1',test:'E2']))
  ))),
  assert_false(kb_call((
    subclass_of(test:'EUnion',Union2),
    is_union_of(Union2,union_of([test:'E1']))
  ))),
  assert_false(kb_call((
    subclass_of(test:'EUnion',Union3),
    is_union_of(Union3,union_of([test:'E1',test:'E2',test:'E3']))
  ))),
  assert_false(is_union_of(test:'A',test:'B')).

test('is_union_of(-,+)') :-
  assert_true(is_union_of(_,union_of([test:'E1',test:'E2']))),
  (  is_union_of(Union, union_of([test:'E1',test:'E2']))
  -> assert_true(subclass_of(test:'EUnion', Union))
  ;  true
  ).

test('is_union_of(+,-)') :-
  assert_true(kb_call((subclass_of(test:'EUnion',Union4),is_union_of(Union4,_)))),
  assert_true(
    kb_call(once((
      subclass_of(test:'EUnion',BUnion),
      is_union_of(BUnion,Union),
      Union=union_of([test:'E1',test:'E2'])
    )))
  ).

test('is_union_of(-,-)') :-
  assert_true(once(is_union_of(_,_))).

test('is_intersection_of(+,+)') :-
  assert_true(kb_call((
    subclass_of(test:'CInter',CInter1),
    is_intersection_of(CInter1,intersection_of([test:'C1',test:'C2']))
  ))),
  assert_false(kb_call((
    subclass_of(test:'CInter',CInter2),
    is_intersection_of(CInter2,intersection_of([test:'C1']))
  ))),
  assert_false(kb_call((
    subclass_of(test:'CInter',CInter3),
    is_intersection_of(CInter3,intersection_of([test:'C1',test:'C2',test:'C3']))
  ))),
  assert_false(is_intersection_of(test:'A',test:'B')).

test('is_intersection_of(-,+)') :-
  assert_true(is_intersection_of(_,intersection_of([test:'C1',test:'C2']))),
  (
    kb_call((
      subclass_of(test:'CInter',AInter),
      is_intersection_of(A,intersection_of([test:'C1',test:'C2']))
    ))
    -> assert_unifies(AInter,A) 
    ;  true
  ).

test('is_intersection_of(+,-)') :-
  assert_true(kb_call((
    subclass_of(test:'CInter',CInter4),
    is_intersection_of(CInter4,_)
  ))),
  (
    kb_call((
      subclass_of(test:'CInter',CInter5), 
      is_intersection_of(CInter5,B)
    ))
    -> assert_unifies(intersection_of([test:'C1',test:'C2']),B) 
    ;  true
  ).

test('is_intersection_of(-,-)') :-
  assert_true(is_intersection_of(_,_)).

test('has_equivalent_class(+,+)') :-
  assert_true(has_equivalent_class(test:'ASub',test:'ASubEq')),
  assert_true(has_equivalent_class(test:'ASubEq',test:'ASub')),
  assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain2')),
  assert_true(has_equivalent_class(test:'EqClsChain2',test:'EqClsChain3')),
  assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain3')),
  %%
  assert_false(has_equivalent_class(test:'A',test:'B')).

test('has_equivalent_class(-,+)') :-
  % Make sure that there is only one class that is equivalent to the tested class
  findall(X, has_equivalent_class(X,test:'ASubEq'), Xs),
  assert_unifies(Xs,[_]),
  %
  assert_true(has_equivalent_class(_,test:'ASubEq')),
  findall(Y, has_equivalent_class(Y, test:'EqClsChain2'), Ys),
  assert_true(memberchk(test:'EqClsChain1', Ys)),
  assert_true(memberchk(test:'EqClsChain3', Ys)).

test('has_equivalent_class(+,-)') :-
  assert_true(has_equivalent_class(test:'ASub',_)),
  findall(X, has_equivalent_class(test:'EqClsChain2',X), Xs),
  assert_true(memberchk(test:'EqClsChain1', Xs)),
  assert_true(memberchk(test:'EqClsChain3', Xs)).

test('has_equivalent_class(-,-)', fail) :-
  has_equivalent_class(_,_).

test('same_as(+,+)') :-
  assert_true(same_as(test:'bIndiv',test:'bIndiv')),
  assert_true(same_as(test:'aIndiv',test:'bIndiv')),
  %%
  assert_false(same_as(test:'aIndiv',test:'indiv')),
  assert_false(same_as(test:'aIndiv',test:'doesNotExists')).

test('same_as(-,+)') :-
  % Make sure that there is only one individual (except the individual itself) that
  % is the same as the tested individual
  findall(X, (same_as(X,test:'aIndiv')), Xs),
  assert_unifies(Xs,[_,_]),
  %
  assert_true(same_as(_,test:'bIndiv')),
  assert_true(once((same_as(A,test:'aIndiv'),A=test:'bIndiv'))).

test('same_as(+,-)') :-
  assert_true(same_as(test:'aIndiv',_)),
  assert_true(once((same_as(test:'aIndiv',B), test:'bIndiv'=B))).

test('same_as(-,-)') :-
  assert_true(once(same_as(_,_))).

test('disjoint_with_direct(+,+)::OWL1') :-
  assert_true(model_OWL:disjoint_with_direct(test:'DisjClsChain1',test:'DisjClsChain2')),
  assert_true(model_OWL:disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain1')),
  assert_true(model_OWL:disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain3')),
  %
  assert_false(model_OWL:disjoint_with_direct(test:'DisjClsChain1',test:'DisjClsChain3')),
  assert_false(model_OWL:disjoint_with_direct(test:'A',test:'B')).

test('disjoint_with_direct(+,-)::OWL1') :-
  findall(A, model_OWL:disjoint_with_direct(test:'DisjClsChain1',A), List),
  assert_equals(List, [test:'DisjClsChain2']).

test('disjoint_with_direct(-,+)::OWL1') :-
  findall(A, model_OWL:disjoint_with_direct(A,test:'DisjClsChain1'), List),
  assert_equals(List, [test:'DisjClsChain2']).

test('disjoint_with_direct(+,+)::OWL1::direct') :-
  assert_true(model_OWL:disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain1')),
  assert_true(model_OWL:disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain3')),
  %%
  assert_false(model_OWL:disjoint_with_direct(test:'DisjClsChain1',test:'DisjClsChain3')).

test('disjoint_with_indirect(+,+)::OWL1::indirect') :-
  assert_true(model_OWL:disjoint_with_indirect(
      test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')),
  assert_true(model_OWL:disjoint_with_indirect(
      test:'DisjClsChain1SubSub',test:'DisjClsChain2Sub')).

test('disjoint_with(+,+)::OWL1::indirect') :-
  assert_true(disjoint_with(test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')),
  assert_true(disjoint_with(test:'DisjClsChain1SubSub',test:'DisjClsChain2Sub')).

test('disjoint_with(-,+)::OWL1') :-
  % NOTE: it is intended that only the direct disjoint classes
  %       are yielded for unbound vars to avoid redundancy.
  findall(A, disjoint_with(A,test:'DisjClsChain1Sub'),    AList),
  findall(B, disjoint_with(B,test:'DisjClsChain1SubSub'), BList),
  assert_equals(AList, [test:'DisjClsChain2']),
  assert_equals(BList, [test:'DisjClsChain2']),
  findall(C, disjoint_with(C,test:'DisjClsChain2'), CList),
  assert_equals(CList, [test:'DisjClsChain1',test:'DisjClsChain3']).

test('disjoint_with(+,-)::OWL1') :-
  findall(C, disjoint_with(test:'DisjClsChain1Sub',C),    AList),
  findall(D, disjoint_with(test:'DisjClsChain1SubSub',D), BList),
  assert_equals(AList, [test:'DisjClsChain2']),
  assert_equals(BList, [test:'DisjClsChain2']),
  findall(A, disjoint_with(test:'DisjClsChain2',A), CList),
  assert_equals(CList, [test:'DisjClsChain3',test:'DisjClsChain1']).

test('disjoint_with_direct(+,+)::OWL2') :-
  assert_true(model_OWL:disjoint_with_direct(test:'DisjCls1',test:'DisjCls2')),
  assert_true(model_OWL:disjoint_with_direct(test:'DisjCls3',test:'DisjCls2')),
  assert_true(model_OWL:disjoint_with_direct(test:'DisjCls1',test:'DisjCls3')).

test('disjoint_with_direct(-,+)::OWL2', fail) :-
  model_OWL:disjoint_with_direct(_,test:'DisjCls2').

test('disjoint_with_direct(+,-)::OWL2') :-
  findall(A, model_OWL:disjoint_with_direct(test:'DisjCls2',A), List),
  assert_equals(List, [test:'DisjCls1', test:'DisjCls3']).

test('disjoint_with(+,+)::OWL2') :-
  assert_true(disjoint_with(test:'DisjCls1Sub',test:'DisjCls2Sub')),
  assert_true(disjoint_with(test:'DisjCls1Sub',test:'DisjCls3')),
  assert_true(disjoint_with(test:'DisjCls1SubSub',test:'DisjCls2Sub')),
  assert_true(disjoint_with(test:'DisjCls1SubSub',test:'DisjCls3')).

test('disjoint_with(-,+)::OWL2') :-
  findall(A, disjoint_with(A,test:'DisjCls1Sub'), List),
  assert_equals(List, [test:'DisjCls2', test:'DisjCls3']).

test('disjoint_with(+,-)::OWL2') :-
  findall(A, disjoint_with(test:'DisjCls1Sub',A), List),
  assert_equals(List, [test:'DisjCls2', test:'DisjCls3']).

test('disjoint_with(-,-)::OWL1') :-
  assert_true(once(disjoint_with(_,_))).

test('subclass_of(+,+Descr)') :-
    assert_true(kb_call(subclass_of_description(test:'D', min(test:'s',2)))),
    assert_true(subclass_of(test:'D', min(test:'s',2))),
    assert_true(subclass_of(test:'CInter', intersection_of([test:'C1',test:'C2']))),
    assert_true(subclass_of(test:'A2',     some(test:'p', test:'B'))),
    % Negative Case
    assert_false(subclass_of(test:'A', only(test:'s', test:'Range1'))).

:- end_tests(model_OWL).
