
:- use_module('mongolog_test').
:- begin_mongolog_tests(mongolog_semweb, 'owl/test/test_owl.owl').

:- use_module('mongolog', [ mongolog_call/1 ]).
:- use_module(library('semweb'), [ sw_register_prefix/2 ]).
:- use_module('semweb').

:- sw_register_prefix(test, 'http://knowrob.org/kb/test_owl#').

test('is_resource(+Resource)') :-
	assert_true(is_resource(test:'E')),
	assert_false(is_resource(test:'bIndiv')),
	assert_false(is_resource(test:'p')),
	assert_false(is_resource(test:'NotExisting')).

test('is_property(+Property)') :-
	assert_true(is_property(test:'p')),
	assert_false(is_property(test:'bIndiv')),
	assert_false(is_property(test:'NotExisting')).

test('instance_of(+,+)') :-
	assert_true(instance_of(test:'bIndiv', test:'B')),
	assert_false(instance_of(test:'bIndiv', test:'A')).

test('subproperty_of(+Sub,+Sup)') :-
	assert_true(subproperty_of(test:'sProp', test:'rProp')),
	assert_false(subproperty_of(test:'sProp', test:'p')).

test_list(RDF_list) :-
	mongolog_call(triple(test:t, owl:propertyChainAxiom, RDF_list)).

test('rdf_list(+,+)') :-
	test_list(RDF_list),
	assert_true(rdf_list(RDF_list,  [test:t1,test:t2,test:t3])),
	assert_false(rdf_list(RDF_list, [test:t2,test:t1,test:t3])),
	assert_false(rdf_list(RDF_list, [test:t1])).

test('rdf_list(+,-)') :-
	test_list(RDF_list),
	assert_true(rdf_list(RDF_list, _)),
	(	rdf_list(RDF_list, Pl_List)
	->	assert_equals(Pl_List, [test:t1,test:t2,test:t3])
	;	true
	).

test('rdf_list_head(+,+)') :-
	test_list(RDF_list1),
	mongolog_call(triple(SubList, rdf:first, test:t2)),
	assert_true(mongolog_call(rdf_list_head(SubList, RDF_list1))).

test('rdf_list_head(+,-)') :-
	test_list(RDF_list1),
	mongolog_call(triple(SubList, rdf:first, test:t2)),
	assert_true(mongolog_call(rdf_list_head(SubList, _))),
	(	mongolog_call(rdf_list_head(SubList, RDF_list2))
	->	assert_equals(RDF_list2, RDF_list1)
	;	true
	).

test('is_class(?)') :-
    assert_true(is_class(_)),
    assert_true(is_class(test:'E')),
    assert_true(is_class(owl:'Thing')),
    assert_false(is_class(test:'DoesNotExists')),
    assert_false(is_class(test:'p')).

test('is_individual') :-
    assert_true(is_individual(_)),
    assert_true(is_individual(test:'bIndiv')),
    assert_false(is_individual(owl:'Thing')),
    assert_false(is_individual(test:'DoesNotExists')),
    assert_false(is_individual(test:'p')).

test('is_object_property') :-
    assert_true(is_object_property(_)),
    assert_true(is_object_property(test:'p')),
    assert_false(is_object_property(test:'DoesNotExists')),
    assert_false(is_object_property(test:'bIndiv')).

test('is_data_property') :-
    assert_true(is_data_property(_)),
    assert_true(is_data_property(test:'P')),
    assert_false(is_data_property(owl:'Nothing')),
    assert_false(is_data_property(test:'DoesNotExists')).

test('is_functional_property') :-
    assert_true(is_functional_property(_)),
    assert_true(is_functional_property(test:'p')),
    assert_false(is_functional_property(test:'s')),
    assert_false(is_functional_property(test:'DoesNotExists')).

test('is_transitive_property') :-
    assert_true(is_transitive_property(_)),
    assert_true(is_transitive_property(test:'r')),
    assert_false(is_transitive_property(test:'p')).

test('is_symmetric_property') :-
    assert_true(is_symmetric_property(_)),
    assert_true(is_symmetric_property(test:'s')),
    assert_false(is_symmetric_property(test:'p')).

test('has_inverse_property') :-
    assert_true(has_inverse_property(test:'r',test:'rInv')),
    assert_true(has_inverse_property(test:'rInv2',test:'r')),
    assert_true(has_inverse_property(test:'r',_)),
    assert_true(has_inverse_property(_,test:'rInv')),
    assert_true(has_inverse_property(_,_)),
    assert_false(has_inverse_property(test:'p',test:'r')).

test('has_property_chain(+,+)') :-
  	assert_true(has_property_chain(test:'t',  [test:'t1',test:'t2',test:'t3'])),
  	assert_false(has_property_chain(test:'t', [test:'t2',test:'t3'])).

test('has_property_chain(+,-)') :-
    assert_true(has_property_chain(test:'t',_)),
    findall(X, has_property_chain(test:'t',X), List),
    length(List,NumChains),
    assert_equals(NumChains,1),
    %
    (   has_property_chain(test:'t',B)
    ->  assert_equals(B,[test:'t1',test:'t2',test:'t3'])
    ;   true
    ).

test('has_equivalent_class(+,+)') :-
    assert_true(has_equivalent_class(test:'ASub',test:'ASubEq')),
    assert_true(has_equivalent_class(test:'ASubEq',test:'ASub')),
    assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain2')),
    assert_true(has_equivalent_class(test:'EqClsChain2',test:'EqClsChain3')),
    assert_true(has_equivalent_class(test:'EqClsChain1',test:'EqClsChain3')),
    assert_false(has_equivalent_class(test:'A',test:'B')).

test('has_equivalent_class(-,+)') :-
    findall(X, has_equivalent_class(X, test:'ASubEq'), Xs),
    findall(Y, has_equivalent_class(Y, test:'EqClsChain2'), Ys),
    assert_unifies(Xs,[_]),
    assert_true(has_equivalent_class(_,test:'ASubEq')),
    assert_true(memberchk(test:'EqClsChain1', Ys)),
    assert_true(memberchk(test:'EqClsChain3', Ys)).

test('has_equivalent_class(+,-)') :-
    findall(X, has_equivalent_class(test:'EqClsChain2',X), Xs),
    assert_true(has_equivalent_class(test:'ASub',_)),
    assert_true(memberchk(test:'EqClsChain1', Xs)),
    assert_true(memberchk(test:'EqClsChain3', Xs)).

test('has_equivalent_class(-,-)', fail) :-
    has_equivalent_class(_,_).

test('same_as(+,+)') :-
    assert_true(same_as(test:'bIndiv',test:'bIndiv')),
    assert_true(same_as(test:'aIndiv',test:'bIndiv')),
    assert_false(same_as(test:'aIndiv',test:'indiv')),
    assert_false(same_as(test:'aIndiv',test:'doesNotExists')).

test('same_as(-,+)') :-
    findall(X, (same_as(X,test:'aIndiv')), Xs),
    assert_unifies(Xs,[_,_]),
    assert_true(same_as(_,test:'bIndiv')),
    assert_true(once((same_as(A,test:'aIndiv'),A=test:'bIndiv'))).

test('same_as(+,-)') :-
    assert_true(same_as(test:'aIndiv',_)),
    assert_true(once((same_as(test:'aIndiv',B), test:'bIndiv'=B))).

test('same_as(-,-)') :-
    assert_true(once(same_as(_,_))).

test('disjoint_with_direct::OWL1') :-
    assert_true(disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain1')),
    assert_true(disjoint_with_direct(test:'DisjClsChain2',test:'DisjClsChain3')),
    assert_true(disjoint_with_direct(test:'DisjClsChain1',test:'DisjClsChain2')),
    findall(A, disjoint_with_direct(test:'DisjClsChain1',A), List),
    assert_equals(List, [test:'DisjClsChain2']),
    %
    assert_false(disjoint_with_direct(test:'DisjClsChain1',test:'DisjClsChain3')),
    assert_false(disjoint_with_direct(test:'A',test:'B')).

test('disjoint_with_direct::OWL2') :-
    assert_true(disjoint_with_direct(test:'DisjCls1',test:'DisjCls2')),
    assert_true(disjoint_with_direct(test:'DisjCls3',test:'DisjCls2')),
    assert_true(disjoint_with_direct(test:'DisjCls1',test:'DisjCls3')),
    findall(A, disjoint_with_direct(test:'DisjCls2',A), List),
    assert_equals(List, [test:'DisjCls1', test:'DisjCls3']).

test('disjoint_with(+,+)::OWL1') :-
    assert_true(disjoint_with(test:'DisjClsChain1Sub',test:'DisjClsChain2Sub')),
    assert_true(disjoint_with(test:'DisjClsChain1SubSub',test:'DisjClsChain2Sub')).

test('disjoint_with(+,+)::OWL2') :-
    assert_true(disjoint_with(test:'DisjCls1Sub',test:'DisjCls2Sub')),
    assert_true(disjoint_with(test:'DisjCls1Sub',test:'DisjCls3')),
    assert_true(disjoint_with(test:'DisjCls1SubSub',test:'DisjCls2Sub')),
    assert_true(disjoint_with(test:'DisjCls1SubSub',test:'DisjCls3')).

test('disjoint_with(+,-)::OWL1') :-
    findall(C, disjoint_with(test:'DisjClsChain1Sub',C),    AList),
    findall(D, disjoint_with(test:'DisjClsChain1SubSub',D), BList),
    findall(A, disjoint_with(test:'DisjClsChain2',A),       CList),
    assert_equals(AList, [test:'DisjClsChain2']),
    assert_equals(BList, [test:'DisjClsChain2']),
    assert_equals(CList, [test:'DisjClsChain3',test:'DisjClsChain1']).

test('disjoint_with(+,-)::OWL2') :-
    findall(A, disjoint_with(test:'DisjCls1Sub',A), List),
    assert_equals(List, [test:'DisjCls2', test:'DisjCls3']).

test('class_expr(+,restriction(+))') :-
    assert_true(mongolog_call((subclass_of(test:'A',R),   class_expr(R,  only(test:'s', test:'Range2'))))),
    assert_true(mongolog_call((subclass_of(test:'A2',R2), class_expr(R2, some(test:'p', test:'B'))))),
    assert_true(mongolog_call((subclass_of(test:'B',R3),  class_expr(R3, min(test:'r',2,test:'Range1'))))),
    assert_true(mongolog_call((subclass_of(test:'B',R4),  class_expr(R4, max(test:'r',5))))),
    assert_true(mongolog_call((subclass_of(test:'B',R5),  class_expr(R5, exactly(test:'s',3))))),
    % Negative Cases
    assert_false(mongolog_call((subclass_of(test:'A',R),  class_expr(R,  only(test:'s', test:'Range1'))))),
    assert_false(mongolog_call((subclass_of(test:'A2',R), class_expr(R,  some(test:'t', test:'B'))))).

test('class_expr(+,restriction(-))') :-
    assert_true((subclass_of(test:'A',R),class_expr(R,_))),
    assert_true(mongolog_call(once((
        subclass_of(test:'A',A),
        class_expr(A,only(test:'s', test:'Range2')))))).

test('class_expr(+,union_of(+))') :-
    assert_true(mongolog_call((subclass_of(test:'EUnion',Union1),
                               class_expr(Union1,union_of([test:'E1',test:'E2']))))),
    assert_false(mongolog_call((subclass_of(test:'EUnion',Union2),
                                class_expr(Union2,union_of([test:'E1']))))),
    assert_false(class_expr(test:'A',test:'B')).

test('class_expr(+,union_of(-))') :-
    assert_true(mongolog_call((subclass_of(test:'EUnion',Union4),class_expr(Union4,_)))),
    assert_true(once(mongolog_call((
        subclass_of(test:'EUnion',BUnion),
        class_expr(BUnion,Union),
        Union=union_of([test:'E1',test:'E2']))))).

test('class_expr(+,intersection_of(+))') :-
    assert_true(mongolog_call((subclass_of(test:'CInter',CInter1),
                               class_expr(CInter1,intersection_of([test:'C1',test:'C2']))))),
    assert_false(mongolog_call((subclass_of(test:'CInter',CInter2),
                                class_expr(CInter2,intersection_of([test:'C1']))))),
    assert_false(class_expr(test:'A',test:'B')).

test('class_expr(+,intersection_of(-))') :-
    assert_true(mongolog_call((subclass_of(test:'CInter',CInter4), class_expr(CInter4,_)))),
    (   mongolog_call((subclass_of(test:'CInter',CInter5),
                       class_expr(CInter5,B)))
    ->  assert_unifies(intersection_of([test:'C1',test:'C2']),B)
    ;   true
    ).

test('class_expr(+,complement_of(+))') :-
    assert_true(mongolog_call((subclass_of(test:'DCompl',Compl1),
                               class_expr(Compl1,complement_of(test:'D'))))),
    % Negative Case
    assert_false(class_expr(test:'A',test:'B')).

test('class_expr(+,complement_of(-))') :-
    (   mongolog_call((subclass_of(test:'DCompl',Compl), class_expr(Compl,B)))
    ->  assert_unifies(complement_of(test:'D'),B)
    ;   fail
    ).

test('subclass_of_expr(+,+Descr)') :-
    assert_true(mongolog_call(subclass_of_expr(test:'D', min(test:'s',2)))),
    assert_true(subclass_of_expr(test:'D',      min(test:'s',2))),
    assert_true(subclass_of_expr(test:'CInter', intersection_of([test:'C1',test:'C2']))),
    assert_true(subclass_of_expr(test:'A2',     some(test:'p', test:'B'))),
    % Negative Case
    assert_false(subclass_of_expr(test:'A',     only(test:'s', test:'Range1'))).

:- end_mongolog_tests(mongolog_semweb).
