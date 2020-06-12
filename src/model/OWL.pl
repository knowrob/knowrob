:- module(model_OWL,
    [ is_class(r),
      is_restriction(r),
      is_restriction(r,t),
      is_union_of(r,t),
      is_intersection_of(r,t),
      is_complement_of(r,t),
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
      has_description(r,t),
      same_as(r,r)
    ]).
/** <module> The Web Ontology Language (OWL) model.

@author Daniel Beßler
*/

%%
% Manchester OWL Syntax Precedence:
%   - some, all, value, min, max, exactly, that
%   - not
%   - and
%   - or
%
:- op(1000, fy, user:only).
%:- op(1000, fy, user:all).
:- op(1000, fy, user:some).
:- op(1000, fy, user:value).
%:- op(1000, fy, user:min).
%:- op(1000, fy, user:max).
%:- op(1000, fy, user:exactly).
%:- op(1000, xfy, user:that).
:- op(999,  fy, user:not).
% TODO: allow expressions (A and B and (C or D))
%        stop using intersection_of and union_of terms
%:- op(998, xfy, user:and).
%:- op(997, xfy, user:or).

:- use_module(library('semweb/rdf_db'),
    [ rdf_register_ns/3,
      rdf_equal/2 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2,
      tripledb_ask/3
    ]).
:- use_module('./RDFS.pl',
    [ has_type/2,
      is_rdf_list/2
    ]).

% load OWL model
:- tripledb_load('http://www.w3.org/2002/07/owl.rdf',
    [ graph(common),
      namespace(owl,'http://www.w3.org/2002/07/owl#')
    ]).

%% is_class(+Entity) is semidet.
%
% True for OWL classes.
%
% @param Entity An entity IRI.
%
is_class(Entity) ?+>
  has_type(Entity, owl:'Class').

%% is_restriction(+Entity) is semidet.
%
% True for OWL restrcitions.
%
% @param Entity An entity IRI.
%
is_restriction(Entity) ?+>
  has_type(Entity, owl:'Restriction').

%% is_individual(+Entity) is semidet.
%
% True for OWL individuals.
%
% @param Entity An entity IRI.
%
is_individual(Entity) ?+>
  has_type(Entity, owl:'NamedIndividual').

%% is_object_property(+Entity) is semidet.
%
% True for OWL object properties.
%
% @param Entity An entity IRI.
%
is_object_property(Entity) ?+>
  has_type(Entity, owl:'ObjectProperty').

%% is_functional_property(+Entity) is semidet.
%
% True for functional properties.
%
% @param Entity An entity IRI.
%
is_functional_property(Entity) ?+>
  has_type(Entity, owl:'FunctionalProperty').

%% is_transitive_property(+Entity) is semidet.
%
% True for transitive properties.
%
% @param Entity An entity IRI.
%
is_transitive_property(Entity) ?+>
  has_type(Entity, owl:'TransitiveProperty').

%% is_symmetric_property(+Entity) is semidet.
%
% True for symmetric properties.
%
% @param Entity An entity IRI.
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

%% has_description(+Class,-Descr) is semidet.
%
% Convert an OWL class into a Prolog representation.  This
% representation is:
%
%    * class(Class)
%    * only(Property,Class)
%    * some(Property,Class)
%    * min(Property,Min,Class)
%    * max(Property,Max,Class)
%    * exactly(Property,Count,Class)
%    * union_of(Classes)
%    * intersection_of(Classes)
%    * not(Class)
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
% @param Class an OWL class
% @param Descr Prolog term representing the class
%
has_description(Descr,Descr) ?> { compound(Descr), ! }.

has_description(Class,_) ?>
  { \+ ground(Class), ! },
  { throw(error(instantiation_error, has_description)) }.

has_description(Class,Descr) ?> is_restriction1(Class,Descr), { ! }.
has_description(Class,Descr) ?> is_union_of(Class,Descr), { ! }.
has_description(Class,Descr) ?> is_intersection_of(Class,Descr), { ! }.
has_description(Class,Descr) ?> is_complement_of(Class,Descr), { ! }.
has_description(Class,class(Class)) ?> { true }.

%% is_description_of(+Descr,-Class) is semidet.
%
% TODO: why not has_description in two directions?
%
%is_description_of(Descr,Resource) ?>
%  { fail }.

%% is_restriction(?Restr,?Descr) is nondet.
%
% Convert an OWL restriction class into a Prolog representation.
%
% @param Restr OWL restriction class
% @param Descr Prolog term representing the class
%
is_restriction(R,Descr) +>
  % try to find existing restriction first.
  { ask(is_restriction1(R,Descr)) },
  { ! }.

is_restriction(R,Descr) +>
  is_restriction(R),
  is_restriction1(R,Descr).

is_restriction(R,Descr) ?>
  is_restriction1(R,Descr).

%%
is_restriction1(R,only(P,O)) ?+>
  triple(R,owl:allValuesFrom,O),
  triple(R,owl:onProperty,P).

is_restriction1(R,some(P,O)) ?+>
  triple(R,owl:someValuesFrom,O),
  triple(R,owl:onProperty,P).

is_restriction1(R,value(P,O)) ?+>
  triple(R,owl:onProperty,P),
  triple(R,owl:hasValue,O).

is_restriction1(R,min(P,M,O)) ?+>
  triple(R,owl:onProperty,P),
  triple(R,owl:onClass,O),
  triple(R,owl:minCardinality,M).

is_restriction1(R,max(P,M,O)) ?+>
  triple(R,owl:onProperty,P),
  triple(R,owl:onClass,O),
  triple(R,owl:maxCardinality,M).

is_restriction1(R,exactly(P,M,O)) ?+>
  triple(R,owl:onProperty,P),
  triple(R,owl:onClass,O),
  triple(R,owl:qualifiedCardinality,M).

%%
is_restriction_term_(only(_,_)).
is_restriction_term_(some(_,_)).
is_restriction_term_(value(_,_)).
is_restriction_term_(min(_,_,_)).
is_restriction_term_(max(_,_,_)).
is_restriction_term_(exactly(_,_,_)).

%% is_union_of(?UnionClass,?Descr) is nondet.
%
% Convert an OWL union class into a Prolog representation.
%
% @param UnionClass OWL union class
% @param Descr Prolog term representing the class
%
is_union_of(UnionClass,union_of(List_pl)) +>
  is_rdf_list(List_rdf,List_pl),
  is_class(UnionClass),
  triple(UnionClass,owl:unionOf,List_rdf).

is_union_of(UnionClass,union_of(List_pl)) ?>
  triple(UnionClass,owl:unionOf,List_rdf),
  is_rdf_list(List_rdf,List_pl).

%% is_intersection_of(?IntersectionClass,?Descr) is nondet.
%
% Convert an OWL intersection class into a Prolog representation.
%
% @param IntersectionClass OWL intersection class
% @param Descr Prolog term representing the class
%
is_intersection_of(IntersectionClass,intersection_of(List_pl)) +>
  is_rdf_list(List_rdf,List_pl),
  is_class(IntersectionClass),
  triple(IntersectionClass,owl:intersectionOf,List_rdf).

is_intersection_of(IntersectionClass,intersection_of(List_pl)) ?>
  triple(IntersectionClass,owl:intersectionOf,List_rdf),
  is_rdf_list(List_rdf,List_pl).

%% is_complement_of(?ComplementClass,?Descr) is nondet.
%
% Convert an OWL complement class into a Prolog representation.
%
% @param ComplementClass an OWL restriction class
% @param Descr Prolog term representing the class
%
is_complement_of(ComplementClass,not(Class)) +>
  is_class(ComplementClass),
  triple(ComplementClass,owl:complementOf,Class).

is_complement_of(ComplementClass,not(Class)) ?>
  triple(ComplementClass,owl:complementOf,Class).

%% has_inverse_property(?Property, ?Inverse) is nondet.
%
% The inverse-of relation between two properties.
%
% @param Property property resource
% @param Inverse inverse of the property
%
has_inverse_property(P,P_inv) ?+>
  triple(P,owl:inverseOf,P_inv).

has_inverse_property(P,P_inv) ?>
  triple(P_inv,owl:inverseOf,P).

%% has_property_chain(?Property, -Chain) is nondet.
%
% Reads chain axioms of a property resource.
%
% @param Property property resource
% @param Chain list of property resources
%
has_property_chain(P,Chain) ?>
  triple(P,owl:propertyChainAxiom,RDFList),
  is_rdf_list(RDFList,Chain).

%% has_disjoint_class(?Class1, ?Class2) is nondet.
%
% Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
% properties and the OWL2 AllDisjointClasses into account.
%
% @param Class1 OWL class
% @param Class2 a disjoint OWL class
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
  % TODO rather use triple(_,rdfs:subClassOf,_) here?
  ( Sup_A=A ; transitive(subclass_of(A,Sup_A)) ),
  ( tripledb_ask(Sup_A,owl:disjointWith,Sup_B) ;
    tripledb_ask(Sup_B,owl:disjointWith,Sup_A) ),
  ( unify_disjoint_(B,Sup_B) ).

%% OWL2 AllDisjointClasses
has_disjoint_class2(A,B) :-
  is_all_disjoint_classes(DC),
  tripledb_ask(DC,owl:members,RDF_list),
  is_rdf_list(RDF_list,List),
  once((
    member(Sup_A,List), subclass_of(A,Sup_A),
    member(Sup_B,List), unify_disjoint_(B,Sup_B)
  )).

unify_disjoint_(B,Disjoint) :-
  ( var(B) -> B=Disjoint ; subclass_of(B,Disjoint) ).

%% has_equivalent_class(?Class1, ?Class2) is nondet.
%
% Maps an OWL class to classes it is equivalent to.
% This is in particular important for classification
% as suffcicient conditions can be defined using equivalent
% class statements.
%
% @param Class1 OWL class
% @param Class2 an equivalent OWL class
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
has_equivalent_class1([Entity|_],Entity,_) :- !.

has_equivalent_class1([Entity|Queue],Same,Visited) :-
  findall(Next, (
    has_equivalent_direct(Entity,Next),
    Entity \= Next,
    \+ memberchk(Next,Visited),
    \+ memberchk(Next,Queue)
  ), List),
  append(Queue,List,Queue0),
  has_equivalent_class1(Queue0,Same,[Entity|Visited]).

has_equivalent_direct(Cls,EQ) :-
  tripledb_ask(Cls, owl:equivalentClass, EQ) ;
  tripledb_ask(EQ,  owl:equivalentClass, Cls).

%% same_as(?X, ?Y) is nondet.
%
% True if X and Y are  identical   or  connected by the owl:sameAs
% relation. Considers owl:sameAs transitive and symmetric.
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
  tripledb_ask(Entity, owl:sameAs, Same) ;
  tripledb_ask(Same,   owl:sameAs, Entity).

% TODO: needs to be handled elsewhere
%tripledb_tell(S,P,O,Scope,Graph) :-
  %% HACK: in many cases convinient, but could cause issues
  %%           to avoid adding more then one value to a functional property
  %is_functional_property(P),!,
  %time_scope_data(Scope,[Since,_]),
  %tripledb_stop(S,P,Since),
  %tripledb_tell1(S,P,O,Scope,Graph).

% TODO: needs to be handled elsewhere
%tripledb_tell1(S,P,O,Scope,Graph) :-
  %itripledb_tell(S,P,O,Scope,Graph),
  %( is_symmetric_property(P) ->
    %itripledb_tell(O,P,S,Scope);
    %true
  %).


		 /*******************************
		 *	    LANGUAGE EXTENSIONS		*
		 *******************************/

%%
% Allow OWL descriptions in subclass_of expressions.
%
subclass_of(S,Descr) ?+>
  { compound(Descr) },
  subclass_of_description(S,Descr).

%%
subclass_of_description(Class,Descr) ?>
  % equal class
  has_description(Class,Descr).

subclass_of_description(Class,intersection_of(List)) ?+>
  { ! }, subclass_of_description_(Class,intersection_of(List),is_intersection_of).

subclass_of_description(Class,union_of(List)) ?+>
  { ! }, subclass_of_description_(Class,union_of(List),is_union_of).
  
subclass_of_description(Class,not(O)) ?+>
  { ! }, subclass_of_description_(Class,not(O),is_complement_of).

subclass_of_description(Class,Descr) ?+>
  { is_restriction_term_(Descr),! },
  subclass_of_description_(Class,Descr,is_restriction).

%%
subclass_of_description_(Class,Descr,_Goal) ?>
  { ground(Class), ! },
  triple(Class,rdfs:subClassOf,SuperClass),
  has_description(SuperClass,Descr).

subclass_of_description_(Class,Descr,Goal) ?+>
  { Goal0=..[Goal,Restriction,Descr] },
  call(Goal0),
  subclass_of_description1_(Class,Restriction).

subclass_of_description1_(Class0,Class1) ?+> { ground([Class0,Class1]), Class0=Class1, true,! }.
subclass_of_description1_(Class,Restr)   ?+> triple(Class,rdfs:subClassOf,Restr).

%%
% Allow OWL descriptions in instance_of expressions.
%
instance_of(S,Descr) ?+>
  { compound(Descr) },
  instance_of_description(S,Descr).

%%
instance_of_description(S,intersection_of(List)) ?+>
  { ! }, instance_of_all(S,List).

instance_of_description(S,union_of(List)) ?+>
  { ! }, instance_of_description_(S,union_of(List),is_union_of).
  
instance_of_description(S,not(Cls)) ?+>
  { ! }, instance_of_description_(S,not(Cls),is_complement_of).

instance_of_description(S,value(P,O)) ?>
  { var(P),! },
  triple(S,P,O),
  { is_object_property(P) ;
    is_data_property(P)
  }.
  
instance_of_description(S,value(P,O)) ?>
  triple(S,P,O).

instance_of_description(S,Descr) ?+>
  { is_restriction_term_(Descr),! },
  instance_of_description_(S,Descr,is_restriction).

%%
instance_of_description_(S,Descr,_Goal) ?>
  { ground(S), ! },
  has_type(S,SType),
  { \+ rdf_equal(SType,owl:'NamedIndividual') },
  subclass_of(SType,Descr).

instance_of_description_(S,Descr,Goal) ?+>
  { Goal0=..[Goal,SType,Descr] },
  call(Goal0),
  has_type(S,SType).

%%
% Allow OWL descriptions in holds expressions.
%
holds(S,P,O) ?>
  { \+ compound(O) },
  instance_of_description_(S,value(P,O),is_restriction).

holds(S,P,Descr) ?+>
  { compound(Descr) },
  holds_description(S,P,Descr).

holds_description(S,P,only(O))      ?+> instance_of(S,only(P,O)).
holds_description(S,P,some(O))      ?+> instance_of(S,some(P,O)).
holds_description(S,P,value(O))     ?+> instance_of(S,value(P,O)).
holds_description(S,P,min(M,O))     ?+> instance_of(S,min(P,M,O)).
holds_description(S,P,max(M,O))     ?+> instance_of(S,max(P,M,O)).
holds_description(S,P,exactly(M,O)) ?+> instance_of(S,exactly(P,M,O)).
holds_description(S,P,value(O))     ?+> instance_of(S,value(P,O)).
