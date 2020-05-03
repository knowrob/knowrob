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
      is_symmetric_property(r),
      has_inverse_property(r,r),
      has_property_chain(r,t),
      has_disjoint_class(r,r),
      has_equivalent_class(r,r),
      same_as(r,r),
      owl_description(r,t),
      op(1000, fy, only),
      op(1000, fy, some),
      op(1000, fy, value)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

% TODO: allow expressions (A and B and (C or D))

:- use_module(library('semweb/rdf_db'),
    [ rdf_register_ns/3
    ]).
:- use_module(library('db/tripledb')
    [ tripledb_load/2,
      tripledb_ask/3,
      tripledb_tell/3
    ]).
:- use_module('./RDFS.pl'
    [ has_type/2,
      rdfs_list/2
    ]).

% load OWL model
:- tripledb_load('http://knowrob.org/kb/owl.owl',
    [ graph(common)
    ]).
:- rdf_register_ns(owl,
    'http://www.w3.org/2002/07/owl#',
    [ keep(true)
    ]).

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
% FIXME: seems redundant with owl_description
%
is_restriction(R,only(P,O)) ?>
  { ground(O),! },
  tripledb_ask(R,owl:allValuesFrom,O),
  tripledb_ask(R,owl:onProperty,P).

is_restriction(R,only(P,O)) ?>
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:allValuesFrom,O).

is_restriction(R,only(P,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  tripledb_tell(R,owl:allValuesFrom,O).


is_restriction(R,some(P,O)) ?>
  { ground(O),! },
  tripledb_ask(R,owl:someValuesFrom,O),
  tripledb_ask(R,owl:onProperty,P).

is_restriction(R,some(P,O)) ?>
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:someValuesFrom,O).

is_restriction(R,some(P,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  tripledb_tell(R,owl:someValuesFrom,O).


is_restriction(R,value(P,O)) ?>
  { ground(P),! },
  tripledb_ask(R,owl:onProperty,P),
  % TODO: need to specify type for data properties
  tripledb_ask(R,owl:hasValue,O).

is_restriction(R,value(P,O)) ?>
  % TODO: need to specify type for data properties
  tripledb_ask(R,owl:hasValue,O),
  tripledb_ask(R,owl:onProperty,P).

is_restriction(R,value(P,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  % TODO: need to specify type for data properties
  tripledb_tell(R,owl:hasValue,O).


is_restriction(R,min(P,M,O)) ?>
  { ground(P), ! },
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:minCardinality,M).

is_restriction(R,min(P,M,O)) ?>
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:minCardinality,M).

is_restriction(R,min(P,M,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  tripledb_tell(R,owl:minCardinality,M),
  tripledb_tell(R,owl:onClass,O).


is_restriction(R,max(P,M,O)) ?>
  { ground(P), ! },
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:minCardinality,M).

is_restriction(R,max(P,M,O)) ?>
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:minCardinality,M).

is_restriction(R,max(P,M,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  tripledb_tell(R,owl:minCardinality,M),
  tripledb_tell(R,owl:onClass,O).


is_restriction(R,exactly(P,M,O)) ?>
  { ground(P), ! },
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:qualifiedCardinality,M).

is_restriction(R,exactly(P,M,O)) ?>
  tripledb_ask(R,owl:onClass,O),
  tripledb_ask(R,owl:onProperty,P),
  tripledb_ask(R,owl:qualifiedCardinality,M).

is_restriction(R,exactly(P,M,O)) +>
  is_restriction(R),
  tripledb_tell(R,owl:onProperty,P),
  tripledb_tell(R,owl:qualifiedCardinality,M),
  tripledb_tell(R,owl:onClass,O).

%%
%
%
is_union_of(S,union_of(O1)) ?>
  % TODO: would be nice having a better interface for lists
  tripledb_ask(S,owl:unionOf,RDFList),
  rdfs_list(RDFList,O2),
  { forall(member(X,O1), member(X,O2)) }.

is_union_of(S,union_of(O)) +>
  rdfs_list(RDFList,O),
  is_class(S),
  tripledb_tell(S,owl:unionOf,RDFList).

%%
%
%
is_intersection_of(S,intersection_of(O1)) ?>
  tripledb_ask(S,owl:intersectionOf,RDFList),
  rdfs_list(RDFList,O2),
  { forall(member(X,O1), member(X,O2)) }.

is_intersection_of(S,intersection_of(O)) +>
  rdfs_list(RDFList,O),
  is_class(S),
  tripledb_tell(S,owl:intersectionOf,RDFList).

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

%%
%
%
is_functional_property(Entity) ?+>
  has_type(Entity, owl:'FunctionalProperty').

%%
%
%
is_transitive_property(Entity) ?+>
  has_type(Entity, owl:'TransitiveProperty').

%%
%
%
is_symmetric_property(Entity) ?+>
  has_type(Entity, owl:'SymmetricProperty').

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
%
has_inverse_property(P,P_inv) ?>
  tripledb_ask(P,owl:inverseOf,P_inv).

has_inverse_property(P,P_inv) ?>
  tripledb_ask(P_inv,owl:inverseOf,P).

has_inverse_property(P,P_inv) +>
  tripledb_tell(P,owl:inverseOf,P_inv).

%%
%
%
has_property_chain(P,Chain) ?>
  tripledb_ask(P,owl:propertyChainAxiom,RDFList),
  rdfs_list(RDFList,Chain).

%%
%
lang_is_a:subclass_of(S,only(P,O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_restriction(R,only(P,O)).

lang_is_a:subclass_of(S,only(P,O)) ?>
  is_restriction(R,only(P,O)),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,only(P,O)) +>
  is_restriction(R,only(P,O)),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,some(P,O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_restriction(R,some(P,O)).

lang_is_a:subclass_of(S,some(P,O)) ?>
  is_restriction(R,some(P,O)),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,some(P,O)) +>
  is_restriction(R,some(P,O)),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,value(P,O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_restriction(R,value(P,O)).

lang_is_a:subclass_of(S,value(P,O)) ?>
  is_restriction(R,value(P,O)),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,value(P,O)) +>
  is_restriction(R,value(P,O)),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,min(P,M,O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_restriction(R,min(P,M,O)).

lang_is_a:subclass_of(S,min(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,min(P,M,O)) +>
  is_restriction(R,min(P,M,O)),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,max(P,M,O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_restriction(R,max(P,M,O)).

lang_is_a:subclass_of(S,max(P,M,O)) ?>
  is_restriction(R,max(P,M,O)),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,max(P,M,O)) +>
  is_restriction(R,max(P,M,O)),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,complement_of(O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  tripledb_ask(R,owl:complementOf,O).

lang_is_a:subclass_of(S,complement_of(O)) ?>
  tripledb_ask(R,owl:complementOf,O),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,complement_of(O)) +>
  is_class(R),
  tripledb_tell(R,owl:complementOf,O),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,intersection_of(O1)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_intersection_of(R,O).

lang_is_a:subclass_of(S,intersection_of(O1)) ?>
  is_intersection_of(R,O),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,intersection_of(O)) +>
  is_intersection_of(R,O),
  tripledb_tell(S,rdfs:subClassOf,R).


lang_is_a:subclass_of(S,union_of(O)) ?>
  { ground(S),! },
  tripledb_subclass_of(S,R),
  is_union_of(R,O).

lang_is_a:subclass_of(S,union_of(O)) ?>
  is_union_of(R,O),
  tripledb_subclass_of(S,R).

lang_is_a:subclass_of(S,union_of(O)) +>
  is_union_of(R,O),
  tripledb_tell(S,rdfs:subClassOf,R).

%%
%
lang_is_a:instance_of(S,only(P,O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_restriction(R,only(P,O)).

vinstance_of(S,only(P,O)) ?>
  is_restriction(R,only(P,O)),
  has_type(S,R).

lang_is_a:instance_of(S,only(P,O)) +>
  is_restriction(R,only(P,O)),
  has_type(S,R).


lang_is_a:instance_of(S,some(P,O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_restriction(R,some(P,O)).

lang_is_a:instance_of(S,some(P,O)) ?>
  is_restriction(R,some(P,O)),
  has_type(S,R).

lang_is_a:instance_of(S,some(P,O)) +>
  is_restriction(R,some(P,O)),
  has_type(S,R).


lang_is_a:instance_of(S,value(P,O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_restriction(R,value(P,O)).

lang_is_a:instance_of(S,value(P,O)) ?>
  is_restriction(R,value(P,O)),
  has_type(S,R).

lang_is_a:instance_of(S,value(P,O)) +>
  is_restriction(R,value(P,O)),
  has_type(S,R).


lang_is_a:instance_of(S,min(P,M,O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_restriction(R,min(P,M,O)).

lang_is_a:instance_of(S,min(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  has_type(S,R).

lang_is_a:instance_of(S,min(P,M,O)) +>
  is_restriction(R,min(P,M,O)),
  has_type(S,R).


lang_is_a:instance_of(S,max(P,M,O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_restriction(R,min(P,M,O)).

lang_is_a:instance_of(S,max(P,M,O)) ?>
  is_restriction(R,min(P,M,O)),
  has_type(S,R).

lang_is_a:instance_of(S,max(P,M,O)) +>
  is_restriction(R,max(P,M,O)),
  has_type(S,R).


lang_is_a:instance_of(S,complement_of(O)) ?>
  { ground(S), ! },
  has_type(S,R),
  tripledb_ask(R,owl:complementOf,O).

lang_is_a:instance_of(S,complement_of(O)) ?>
  tripledb_ask(R,owl:complementOf,O),
  has_type(S,R).

lang_is_a:instance_of(S,complement_of(O)) +>
  is_class(R),
  tripledb_tell(R,owl:complementOf,O),
  has_type(S,R).


lang_is_a:instance_of(S,union_of(O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_union_of(R,union_of(O)).

lang_is_a:instance_of(S,union_of(O)) ?>
  is_union_of(R,union_of(O)),
  has_type(S,R).

lang_is_a:instance_of(S,union_of(O)) +>
  is_union_of(R,union_of(O)),
  has_type(S,R).


lang_is_a:instance_of(S,intersection_of(O)) ?>
  { ground(S), ! },
  has_type(S,R),
  is_intersection_of(R,O).

lang_is_a:instance_of(S,intersection_of(O)) ?>
  is_intersection_of(R,O),
  has_type(S,R).

lang_is_a:instance_of(S,intersection_of(O)) +>
  is_intersection_of(R,O),
  has_type(S,R).

%%
%
%
lang_holds:holds(S,P,only(O)) +?>
  instance_of(S,only(P,O)).

lang_holds:holds(S,P,some(O)) +?>
  instance_of(S,some(P,O)).

lang_holds:holds(S,P,value(O)) +?>
  instance_of(S,value(P,O)).

lang_holds:holds(S,P,min(M,O)) +?>
  instance_of(S,min(P,M,O)).

lang_holds:holds(S,P,max(M,O)) +?>
  instance_of(S,max(P,M,O)).

lang_holds:holds(S,P,exactly(M,O)) +?>
  instance_of(S,exactly(P,M,O)).

lang_holds:holds(S,P,value(O)) +?>
  instance_of(S,value(P,O)).

lang_holds:holds(S,P,O) ?>
  instance_of(S,value(P,O)).

%% has_disjoint_class(?Class1, ?Class2) is nondet.
%
% Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
% properties and the OWL2 AllDisjointClasses into account.
%
has_disjoint_class(A,B) :-
  ground([A,B]), A=B, !, fail.

has_disjoint_class(A,B) :-
  ground(A),!,
  ( has_disjoint_class1(A,B);
    has_disjoint_class2(A,B) ).

has_disjoint_class(A,B) :-
  ground(B),!,
  has_disjoint_class(B,A).

has_disjoint_class(A,B) :-
  is_class(A),
  has_disjoint_class(A,B).
  
%% OWL1 disjointWith
has_disjoint_class1(A,B) :-
  % test if there are super-classes of A and B (including A and B)
  % with disjointness axiom.
  subclass_of(A,Sup_A),
  ( tripledb_ask(Sup_A,owl:disjointWith,Sup_B) ;
    tripledb_ask(Sup_B,owl:disjointWith,Sup_A) ),
  subclass_of(B,Sup_B).

%% OWL2 AllDisjointClasses
has_disjoint_class2(A,B) :-
  is_all_disjoint_classes(DC),
  tripledb_ask(DC,owl:members,RDF_list),
  rdfs_list(RDF_list,List),
  once((
    member(Sup_A,List), subclass_of(A,Sup_A),
    member(Sup_B,List), subclass_of(B,Sup_B)
  )).

%%
%
%
has_equivalent_class(Cls,EQ) :-
  ground(Cls),!,
  has_equivalent_class1([Cls],EQ,[]),
  EQ \= Cls.

has_equivalent_class(Cls,EQ) :-
  ground(EQ),!,
  has_equivalent_class1([EQ],Cls,[]),
  EQ \= Cls.

has_equivalent_class(Cls,EQ) :-
  is_class(Cls),
  has_equivalent_class(Cls,EQ).

%%
has_equivalent_class1([Entity|_],Entity,_).

has_equivalent_class1([Entity|Queue],Same,Visited) :-
  findall(Next, (
    has_equivalent_direct(Entity,Next),
    \+ memberchk(Next,Visited),
    \+ memberchk(Next,Queue)
  ), List),
  append(Queue,List,Queue0),
  has_equivalent_class1(Queue0,Same,[Entity|Visited]).

has_equivalent_direct(Cls,EQ) :-
  tripledb_ask(Cls, owl:equivalentClass, EQ) ;
  tripledb_ask(EQ,  owl:equivalentClass, Cls).

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

same_as(Entity,Same) :-
  is_individual(Entity),
  same_as(Entity,Same).

%%
same_as1([Entity|_],Entity,_).

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
owl_description(Descr,Descr) :-
  compound(Descr),
  !.

owl_description(IRI,_) :-
  \+ ground(IRI),
  !,
  throw(error(instantiation_error, _)).

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
  % TODO: need to specify type for data properties
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
