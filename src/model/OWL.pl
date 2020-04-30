:- module(model_OWL,
    [ is_class(r),
      is_restriction(r),
      is_restriction(r,t),
      is_union_of(r,t),
      is_intersection_of(r,t),
      is_individual(r),
      is_object_property(r),
      is_data_property(r),
      is_functional_property(r),
      is_transitive_property(r),
      is_reflexive_property(r),
      is_property_chain(r,t),
      has_inverse_property(r,r),
      has_property_chain(r,t),
      has_disjoint_class(r,r),
      has_equivalent_class(r,r),
      owl_description(r,t),
      op(1000, fy, only),
      op(1000, fy, some),
      op(1000, fy, value)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

% TODO: allow expressions (A and B and (C or D))
% TODO: configure properties for DB storage (store hierarchy or not?)
%            - possibe through general transisity and co handling?

:- use_module(library('semweb/rdf_db'),
        [ rdf_register_ns/3 ]).
:- use_module(library('db/tripledb')
        [ tripledb_load/2 ]).
:- use_module('RDFS'
        [ has_type/2, rdfs_list/2 ]).

% load OWL model
:- tripledb_load(
        'http://knowrob.org/kb/owl.owl',
        [ graph(static) ]).
:- rdf_register_ns(owl,
        'http://www.w3.org/2002/07/owl#',
        [ keep(true) ]).

%% is_class(+Entity) is semidet.
%
% True iff Entity is a class IRI.
%
% @param Entity An entity IRI.
%
is_class(Entity) ?+>
  has_type(Entity, owl:'Class').

%%
%
is_restriction(Entity) ?>
  has_type(Entity, owl:'Restriction').

%%
%
is_restriction(R,only(P,O)) ?>
  { ground(P),! },
  holds(R,owl:onProperty,P),
  holds(R,owl:allValuesFrom,O).

is_restriction(R,only(P,O)) ?>
  holds(R,owl:allValuesFrom,O),
  holds(R,owl:onProperty,P).

is_restriction(R,only(P,O)) +>
  is_restriction(R),
  holds(R,owl:onProperty,P),
  holds(R,owl:allValuesFrom,O).


is_restriction(R,some(P,O)) ?>
  { ground(P),! },
  holds(R,owl:onProperty,P),
  holds(R,owl:someValuesFrom,O).

is_restriction(R,some(P,O)) ?>
  holds(R,owl:someValuesFrom,O),
  holds(R,owl:onProperty,P).

is_restriction(R,some(P,O)) +>
  is_restriction(R),
  holds(R,owl:onProperty,P),
  holds(R,owl:someValuesFrom,O).


is_restriction(R,value(P,O)) ?>
  { ground(P),! },
  holds(R,owl:onProperty,P),
  holds(R,owl:hasValue,O).

is_restriction(R,value(P,O)) ?>
  holds(R,owl:hasValue,O),
  holds(R,owl:onProperty,P).

is_restriction(R,value(P,O)) +>
  is_restriction(R),
  holds(R,owl:onProperty,P),
  holds(R,owl:hasValue,O).


is_restriction(R,min(P,M,O)) ?>
  { ground(P), ! },
  holds(R,owl:onProperty,P),
  holds(R,owl:onClass,O),
  holds(R,owl:minCardinality,M).

is_restriction(R,min(P,M,O)) ?>
  holds(R,owl:onClass,O),
  holds(R,owl:onProperty,P),
  holds(R,owl:minCardinality,M).

is_restriction(R,min(P,M,O)) +>
  is_restriction(R),
  holds(R,owl:onProperty,P),
  holds(R,owl:minCardinality,M),
  holds(R,owl:onClass,O).


is_restriction(R,max(P,M,O)) ?>
  { ground(P), ! },
  holds(R,owl:onProperty,P),
  holds(R,owl:onClass,O),
  holds(R,owl:minCardinality,M).

is_restriction(R,max(P,M,O)) ?>
  holds(R,owl:onClass,O),
  holds(R,owl:onProperty,P),
  holds(R,owl:minCardinality,M).

is_restriction(R,max(P,M,O)) +>
  is_restriction(R),
  holds(R,owl:onProperty,P),
  holds(R,owl:minCardinality,M),
  holds(R,owl:onClass,O).

%%
%
%
is_union_of(S,union_of(O1)) ?>
  holds(S,owl:unionOf,RDFList),
  rdfs_list(RDFList,O2),
  { forall(member(X,O1), member(X,O2)) }.

is_union_of(S,union_of(O)) +>
  rdfs_list(RDFList,O),
  is_class(S),
  holds(S,owl:unionOf,RDFList).

%%
%
%
is_intersection_of(S,intersection_of(O1)) ?>
  holds(S,owl:intersectionOf,RDFList),
  rdfs_list(RDFList,O2),
  { forall(member(X,O1), member(X,O2)) }.

is_intersection_of(S,intersection_of(O)) +>
  rdfs_list(RDFList,O),
  is_class(S),
  holds(S,owl:intersectionOf,RDFList).

%% is_individual(+Entity) is semidet.
%
% True iff Entity is an individual IRI.
%
% @param Entity An entity IRI.
%
is_individual(Entity) ?+>
  has_type(Entity, owl:'NamedIndividual').

%% is_object_property(+Entity) is semidet.
%
% True iff Entity is an object property IRI.
%
% @param Entity An entity IRI.
%
is_object_property(Entity) ?+>
  has_type(Entity, owl:'ObjectProperty').

%% is_data_property(+Entity) is semidet.
%
% True iff Entity is an datatype property IRI.
%
% @param Entity An entity IRI.
%
is_data_property(Entity) ?+>
  has_type(Entity, owl:'DatatypeProperty').

%% has_inverse_property(?P, ?P_inv)
%
has_inverse_property(P,P_inv) ?+>
  holds(P,owl:inverseOf,P_inv).

has_inverse_property(P,P_inv) ?>
  holds(P_inv,owl:inverseOf,P).

%%
%
is_property_chain(P,Chain) ?>
  holds(P,owl:propertyChainAxiom,RDFList),
  rdfs_list(RDFList,Chain).

is_property_chain(P,Chain) +>
  rdfs_list(RDFList,Chain),
  holds(P,owl:propertyChainAxiom,RDFList).

%%
%
is_subclass_of(S,only(P,O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_restriction(R,only(P,O)).

is_subclass_of(S,only(P,O)) ?>
  is_restriction(R,only(P,O)),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,only(P,O)) +>
  is_restriction(R,only(P,O)),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,some(P,O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_restriction(R,some(P,O)).

is_subclass_of(S,some(P,O)) ?>
  is_restriction(R,some(P,O)),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,some(P,O)) +>
  is_restriction(R,some(P,O)),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,value(P,O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_restriction(R,value(P,O)).

is_subclass_of(S,value(P,O)) ?>
  is_restriction(R,value(P,O)),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,value(P,O)) +>
  is_restriction(R,value(P,O)),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,min(P,M,O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_restriction(R,min(P,M,O)).

is_subclass_of(S,min(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,min(P,M,O)) +>
  is_restriction(R,min(P,M,O)),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,max(P,M,O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_restriction(R,max(P,M,O)).

is_subclass_of(S,max(P,M,O)) ?>
  is_restriction(R,max(P,M,O)),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,max(P,M,O)) +>
  is_restriction(R,max(P,M,O)),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,complement_of(O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  holds(R,owl:complementOf,O).

is_subclass_of(S,complement_of(O)) ?>
  holds(R,owl:complementOf,O),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,complement_of(O)) +>
  is_class(R),
  holds(R,owl:complementOf,O),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,intersection_of(O1)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_intersection_of(R,O).

is_subclass_of(S,intersection_of(O1)) ?>
  is_intersection_of(R,O),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,intersection_of(O)) +>
  is_intersection_of(R,O),
  holds(S,rdfs:subClassOf,R).


is_subclass_of(S,union_of(O)) ?>
  { ground(S),! },
  holds(S,rdfs:subClassOf,R),
  is_union_of(R,O).

is_subclass_of(S,union_of(O)) ?>
  is_union_of(R,O),
  holds(S,rdfs:subClassOf,R).

is_subclass_of(S,union_of(O)) +>
  is_union_of(R,O),
  holds(S,rdfs:subClassOf,R).

%%
%
is_instance_of(S,only(P,O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_restriction(R,only(P,O)).

is_instance_of(S,only(P,O)) ?>
  is_restriction(R,only(P,O)),
  holds(S,rdf:type,R).

is_instance_of(S,only(P,O)) +>
  is_restriction(R,only(P,O)),
  holds(S,rdf:type,R).


is_instance_of(S,some(P,O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_restriction(R,some(P,O)).

is_instance_of(S,some(P,O)) ?>
  is_restriction(R,some(P,O)),
  holds(S,rdf:type,R).

is_instance_of(S,some(P,O)) +>
  is_restriction(R,some(P,O)),
  holds(S,rdf:type,R).


is_instance_of(S,value(P,O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_restriction(R,value(P,O)).

is_instance_of(S,value(P,O)) ?>
  is_restriction(R,value(P,O)),
  holds(S,rdf:type,R).

is_instance_of(S,value(P,O)) +>
  is_restriction(R,value(P,O)),
  holds(S,rdf:type,R).


is_instance_of(S,min(P,M,O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_restriction(R,min(P,M,O)).

is_instance_of(S,min(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  holds(S,rdf:type,R).

is_instance_of(S,min(P,M,O)) +>
  is_restriction(R,min(P,M,O)),
  holds(S,rdf:type,R).


is_instance_of(S,max(P,M,O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_restriction(R,min(P,M,O)).

is_instance_of(S,max(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  holds(S,rdf:type,R).

is_instance_of(S,max(P,M,O)) +>
  is_restriction(R,max(P,M,O)),
  holds(S,rdf:type,R).


is_instance_of(S,complement_of(O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  holds(R,owl:complementOf,O).

is_instance_of(S,complement_of(O)) ?>
  holds(R,owl:complementOf,O),
  holds(S,rdf:type,R).

is_instance_of(S,complement_of(O)) +>
  is_class(R),
  holds(R,owl:complementOf,O),
  holds(S,rdf:type,R).


is_instance_of(S,union_of(O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_union_of(R,union_of(O)).

is_instance_of(S,union_of(O)) ?>
  is_union_of(R,union_of(O)),
  holds(S,rdf:type,R).

is_instance_of(S,union_of(O)) +>
  is_union_of(R,union_of(O)),
  holds(S,rdf:type,R).


is_instance_of(S,intersection_of(O)) ?>
  { ground(S), ! },
  holds(S,rdf:type,R),
  is_intersection_of(R,O).

is_instance_of(S,intersection_of(O)) ?>
  is_intersection_of(R,O),
  holds(S,rdf:type,R).

is_instance_of(S,intersection_of(O)) +>
  is_intersection_of(R,O),
  holds(S,rdf:type,R).

%%
%
%
holds(S,P,only(O)) +?>
  is_instance_of(S,only(P,O)).

holds(S,P,some(O)) +?>
  is_instance_of(S,some(P,O)).

holds(S,P,value(O)) +?>
  is_instance_of(S,value(P,O)).

holds(S,P,min(M,O)) +?>
  is_instance_of(S,min(P,M,O)).

holds(S,P,max(M,O)) +?>
  is_instance_of(S,max(P,M,O)).

holds(S,P,O) ?>
  is_instance_of(S,value(P,O)).

%%
has_equivalent_class(Class,EQ) :-
  ( tripledb_ask(Sub,owl:equivalentClass,Sub_EQ);
    tripledb_ask(Sub_EQ,owl:equivalentClass,Sub) ),

%% has_disjoint_class(?Class1, ?Class2) is nondet.
%
% Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
% properties and the OWL2 AllDisjointClasses into account.
%
has_disjoint_class(A,B,_Scope) :-
  ground([A,B]), A=B, !, fail.

has_disjoint_class(A,B,Scope) :-
  ground(A),!,
  ( has_disjoint_class1(A,B,Scope);
    has_disjoint_class2(A,B,Scope) ).

has_disjoint_class(A,B,Scope) :-
  ground(B),!,
  has_disjoint_class(B,A,Scope).

has_disjoint_class(A,B,Scope) :-
  is_class(A),
  has_disjoint_class(A,B,Scope).
  
%% OWL1 disjointWith
has_disjoint_class1(A,B,Scope) :-
  subclass_of(A,Sup_A,Scope),
  ( tripledb_ask(Sup_A,owl:disjointWith,Sup_B) ;
    tripledb_ask(Sup_B,owl:disjointWith,Sup_A) ),
  subclass_of(B,Sup_B,Scope).

%% OWL2 AllDisjointClasses
has_disjoint_class2(A,B,Scope) :-
  is_all_disjoint_classes(DC),
  tripledb_ask(DC,owl:members,RDF_list),
  rdfs_list(RDF_list,List),
  once((
    member(Sup_A,List), subclass_of(A,Sup_A,Scope),
    member(Sup_B,List), subclass_of(B,Sup_B,Scope)
  )).

%%
has_property_chain(P,Chain) :-
  tripledb_ask(P,owl:propertyChainAxiom,RDFList),
  rdfs_list(RDFList,Chain).

%% same_as(?X, ?Y, +Scope) is nondet.
%
% True if X and Y are  identical   or  connected by the owl:sameAs
% relation. Considers owl:sameAs transitive and symetric.
%
same_as(Entity,Same) :-
  ground(Entity),!,
  same_as1([Entity],Same,[]).

same_as(Entity,Same) :-
  ground(Same),!,
  same_as1([Same],Entity,[]).

same_as(Entity,Entity,_).

same_as1([Entity|_],Entity,_,_).

same_as1([Entity|Queue],Same,Visited) :-
  findall(Next, (
    same_as_direct(Entity,Next),
    \+ memberchk(Next,Visited),
    \+ memberchk(Next,Queue)
  ), List),
  append(Queue,List,Queue0),
  same_as1(Queue0,Same,[Entity|Visited]).

same_as_direct(Entity,Same) :-
  tripledb_ask(Entity, owl:sameAs, Next) ;
  tripledb_ask(Next,   owl:sameAs, Entity).

%% owl_description(+IRI, -Term) is det.
%
% Convert an owl description into a Prolog representation.  This
% representation is:
%
%    * class(Class)
%    * only(Property,Description)
%    * some(Property,Description)
%    * min(Property,Description,Min)
%    * max(Property,Description,Max)
%    * exactly(Property,Description,Count)
%    * union_of(ListOfDescriptions)
%    * intersection_of(ListOfDescriptions)
%    * complement_of(Description)
%    * one_of(Individuals)
%
%  For example, the union-of can be the result of
%
%  ==
%  <rdfs:Class rdf:ID="myclass">
%    <owl:unionOf parseType=Collection>
%      <rdf:Description rdf:about="gnu"/>
%      <rdf:Description rdf:about="gnat"/>
%    </owl:unionOf>
%  </rdfs:Class>
%  ==
%
owl_description(IRI,_) :-
  \+ ground(IRI),!,
  throw(error(instantiation_error, _)).

owl_description(Descr,Descr) :-
  compound(Descr),!.

owl_description(IRI,Descr) :-
  owl_description1(IRI,Descr),!.

owl_description(IRI,class(IRI)).

%%
owl_description1(IRI,Descr) :-
  tripledb_ask(IRI,owl:onProperty,P),
  owl_restriction_(IRI,P,Descr).

owl_description1(ID,union_of(Set))        :- is_union_of(ID,Set).
owl_description1(ID,intersection_of(Set)) :- is_intersection_of(ID,Set).
owl_description1(ID,complement_of(Cls))   :- is_complement_of(ID,Cls).
%owl_description1(ID,one_of(Set))          :- is_one_of(ID,Set).

%%
owl_restriction_(IRI,P,only(P,Cls)) :-
  tripledb_ask(IRI,owl:allValuesFrom,Cls).

owl_restriction_(IRI,P,some(P,Cls)) :-
  tripledb_ask(IRI,owl:someValuesFrom,Cls).

owl_restriction_(IRI,P,value(P,Value)) :-
  tripledb_ask(IRI,owl:hasValue,Value).

% TODO: also support unqualified cardinality restrictions?
owl_restriction_(IRI,P,min(P,Cls,Min)) :-
  tripledb_ask(IRI,owl:minCardinality,Min),
  tripledb_ask(IRI,owl:onClass,Cls).

owl_restriction_(IRI,P,max(P,Cls,Max)) :-
  tripledb_ask(IRI,owl:maxCardinality,Max),
  tripledb_ask(IRI,owl:onClass,Cls).

owl_restriction_(IRI,P,exactly(P,Cls,Count)) :-
  tripledb_ask(IRI,owl:qualifiedCardinality,Count),
  tripledb_ask(IRI,owl:onClass,Cls).
