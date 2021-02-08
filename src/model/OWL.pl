:- module(model_OWL,
	[ is_owl_term(t),
	  is_restriction_term(t),
	  is_class(r),
	  is_restriction(r),
	  is_restriction(r,t),
	  is_union_of(r,t),
	  is_intersection_of(r,t),
	  is_complement_of(r,t),
	  is_all_disjoint_classes(r),
	  is_individual(r),
	  is_object_property(r),
	  is_data_property(r),
	  is_functional_property(r),
	  is_transitive_property(r),
	  is_symmetric_property(r),
	  has_inverse_property(r,r),
	  has_property_chain(r,t),
	  disjoint_with(r,r),
	  disjoint_with_direct(r,r),
	  has_equivalent_class(r,r),
	  has_description(r,t),
	  same_as(r,r),
	  is_a(r,r)          % +Resource, ?Type
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
%:- op(999,  fy, user:not).
% TODO: allow expressions (A and B and (C or D))
%        stop using intersection_of and union_of terms
%:- op(998, xfy, user:and).
%:- op(997, xfy, user:or).
:- op(1000, xfx, user:is_a).

:- use_module(library('semweb/rdf_db'),
	[ rdf_register_ns/3, rdf_equal/2 ]).
:- use_module(library('lang/db'),
	[ load_owl/2 ]).
:- use_module('RDFS',
	[ has_type/2, rdf_list/2 ]).

% load OWL model
:- load_owl('http://www.w3.org/2002/07/owl.rdf',
	[ graph(common),
	  namespace(owl,'http://www.w3.org/2002/07/owl#')
	]).

%% is_owl_term(+Term) is semidet.
%
%
is_owl_term(Var) :- var(Var), !, fail.
is_owl_term(union_of(_)) :- !.
is_owl_term(intersection_of(_)) :- !.
is_owl_term(complement_of(_)) :- !.
is_owl_term(one_of(_)) :- !.
is_owl_term(Term) :-
	is_restriction_term(Term).

%% is_restriction_term(+Term) is semidet.
%
%
is_restriction_term(Var) :- var(Var), !, fail.
is_restriction_term(only(_,_)).
is_restriction_term(some(_,_)).
is_restriction_term(value(_,_)).
is_restriction_term(min(_,_)).
is_restriction_term(min(_,_,_)).
is_restriction_term(max(_,_,_)).
is_restriction_term(max(_,_)).
is_restriction_term(exactly(_,_)).
is_restriction_term(exactly(_,_,_)).

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

%%
is_restriction1(R, only(P,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:allValuesFrom,O).

is_restriction1(R, some(P,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:someValuesFrom,O).

is_restriction1(R, value(P,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:hasValue,O).

is_restriction1(R, min(P,M,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:minQualifiedCardinality,M),
	triple(R,owl:onClass,O).

is_restriction1(R, min(P,M)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:minCardinality,M).

is_restriction1(R, max(P,M,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:maxQualifiedCardinality,M),
	triple(R,owl:onClass,O).

is_restriction1(R, max(P,M)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:maxCardinality,M).

is_restriction1(R, exactly(P,M,O)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:qualifiedCardinality,M),
	triple(R,owl:onClass,O).

is_restriction1(R, exactly(P,M)) ?+>
	triple(R,owl:onProperty,P),
	triple(R,owl:cardinality,M).

%% is_restriction(?Restr,?Descr) is nondet.
%
% Convert an OWL restriction class into a Prolog representation.
%
% @param Restr OWL restriction class
% @param Descr Prolog term representing the class
%

%is_restriction(R,Descr) +>
%  % try to find existing restriction first.
%  ask(is_restriction1(R,Descr)),
%  !.

is_restriction(R,Descr) +>
	is_restriction(R),
	is_restriction1(R,Descr).

is_restriction(R,Descr) ?>
	is_restriction1(R,Descr).

%% is_union_of(?UnionClass,?Descr) is nondet.
%
% Convert an OWL union class into a Prolog representation.
%
% @param UnionClass OWL union class
% @param Descr Prolog term representing the class
%
is_union_of(UnionClass, union_of(List_pl)) +>
	rdf_list(List_rdf, List_pl),
	is_class(UnionClass),
	triple(UnionClass, owl:unionOf, List_rdf).

is_union_of(UnionClass, union_of(List_pl)) ?>
	var(List_pl),
	triple(UnionClass, owl:unionOf, List_rdf),
	rdf_list(List_rdf, List_pl).

is_union_of(UnionClass, union_of(List_pl)) ?>
	ground(List_pl),
	rdf_list(List_rdf, List_pl),
	triple(UnionClass, owl:unionOf, List_rdf).

%% is_intersection_of(?IntersectionClass,?Descr) is nondet.
%
% Convert an OWL intersection class into a Prolog representation.
%
% @param IntersectionClass OWL intersection class
% @param Descr Prolog term representing the class
%
is_intersection_of(IntersectionClass, intersection_of(List_pl)) +>
	rdf_list(List_rdf,List_pl),
	is_class(IntersectionClass),
	triple(IntersectionClass, owl:intersectionOf, List_rdf).

is_intersection_of(IntersectionClass, intersection_of(List_pl)) ?>
	var(List_pl),
	triple(IntersectionClass, owl:intersectionOf, List_rdf),
	rdf_list(List_rdf,List_pl).

is_intersection_of(IntersectionClass, intersection_of(List_pl)) ?>
	ground(List_pl),
	rdf_list(List_rdf, List_pl),
	triple(IntersectionClass, owl:intersectionOf, List_rdf).

%% is_complement_of(?ComplementClass,?Descr) is nondet.
%
% Convert an OWL complement class into a Prolog representation.
%
% @param ComplementClass an OWL restriction class
% @param Descr Prolog term representing the class
%
is_complement_of(ComplementClass, complement_of(Class)) ?+>
	triple(ComplementClass, owl:complementOf, Class).


%% has_description(+Class,-Descr) is semidet.
%
% Convert an OWL class into a Prolog representation.  This
% representation is:
%
%    * only(Property,Class)
%    * some(Property,Class)
%    * min(Property,Min,Class)
%    * max(Property,Max,Class)
%    * exactly(Property,Count,Class)
%    * union_of(Classes)
%    * intersection_of(Classes)
%    * complement_of(Class)
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
has_description(Class,_) ?>
	var(Class),
	!,
	% TODO: throw exception once supported
	%throw(error(instantiation_error, has_description)).
	fail.

has_description(Class,Descr) ?>
	(	is_restriction1(Class,Descr)
	;	is_union_of(Class,Descr)
	;	is_intersection_of(Class,Descr)
	;	is_complement_of(Class,Descr)
	),
	!.


%% is_all_disjoint_classes(?AllDisjointClasses) is nondet.
%
% True for OWL2 AllDisjointClasses
%
% @param Entity An entity IRI.
%
is_all_disjoint_classes(Entity) ?+>
	has_type(Entity, owl:'AllDisjointClasses').

%% has_inverse_property(?Property, ?Inverse) is nondet.
%
% The inverse-of relation between two properties.
%
% @param Property property resource
% @param Inverse inverse of the property
%
has_inverse_property(P, P_inv) ?+>
	triple(P, owl:inverseOf, P_inv).

has_inverse_property(P, P_inv) ?>
	triple(P_inv, owl:inverseOf, P).

%% has_property_chain(?Property, -Chain) is nondet.
%
% Reads chain axioms of a property resource.
%
% @param Property property resource
% @param Chain list of property resources
%
has_property_chain(P, Chain) +>
	rdf_list(RDFList, Chain),
	triple(P, owl:propertyChainAxiom, RDFList).

has_property_chain(P, Chain) ?>
	var(Chain),
	triple(P, owl:propertyChainAxiom, RDFList),
	rdf_list(RDFList, Chain).

has_property_chain(P, Chain) ?>
	ground(Chain),
	rdf_list(RDFList, Chain),
	triple(P, owl:propertyChainAxiom, RDFList).

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
has_equivalent_class(X,Y) ?>
	triple(X, transitive(owl:equivalentClass), Y).

has_equivalent_class(X,Y) ?>
	triple(Y, transitive(owl:equivalentClass), X).

has_equivalent_class(X,Y) +>
	triple(X, owl:equivalentClass, Y).

%% same_as(?X, ?Y) is nondet.
%
% True if X and Y are  identical   or  connected by the owl:sameAs
% relation. Considers owl:sameAs transitive and symmetric.
%
same_as(X,Y) ?>
	(ground(X);ground(Y)),
	X = Y.

same_as(X,Y) ?>
	ground(X),!,
	triple(X, transitive(owl:sameAs), Y).

same_as(X,Y) ?>
	ground(Y),!,
	triple(Y, transitive(owl:sameAs), X).

same_as(X,Y) ?+>
	triple(X, owl:sameAs, Y).

%% disjoint_with_direct(?Cls, ?Disjoint) is semidet.
%
% True if there is a disjointness axioms for Cls and
% Disjoint, not taking into account the super classes of both.
%
disjoint_with_direct(A,B) ?>
	(	triple(A, owl:disjointWith, B)
	;	triple(B, owl:disjointWith, A)
	).

%has_disjoint_direct(A,B) ?>
%	% OWL2 disjointness axioms
%	% ground(A),
%	% iterate over all lists where A is an element
%	triple(SubList, rdf:first, A),
%	rdf_list_head(SubList, ListHead),
%	% only proceed for lists of OWL2 disjointness axioms
%	triple(DC, owl:members, ListHead),
%	is_all_disjoint_classes(DC),
%	% yield all members of the list except of A
%	rdf_list(ListHead, PrologList),
%	member(B, PrologList),
%	B \= A.

%%
disjoint_with_indirect(A,B) ?>
	% ground(A),
	triple(A, rdfs:subClassOf, include_parents(SupA)),
	disjoint_with_direct(SupA, SupB),
	(	var(B) -> B=SupB
	;	triple(B, rdfs:subClassOf, SupB)
	).

%% disjoint_with(?Class1, ?Class2) is nondet.
%
% Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
% properties and the OWL2 AllDisjointClasses into account.
%
% @param Class1 OWL class
% @param Class2 a disjoint OWL class
%
%disjoint_with(A,B) :-
%  ground([A,B]), A=B, !, fail.

disjoint_with(A,B) ?>
	disjoint_with_direct(A,B).

disjoint_with(A,B) ?>
	ground(A),!,
	disjoint_with_indirect(A,B).

disjoint_with(A,B) ?>
	ground(B),!,
	disjoint_with_indirect(B,A).

		 /*******************************
		 *	    LANGUAGE EXTENSIONS		*
		 *******************************/

%%
% Allow OWL descriptions in subclass_of expressions.
%
subclass_of(Class, Descr) ?>
	pragma(is_owl_term(Descr)),
	ground(Class),
	triple(Class, rdfs:subClassOf, include_parents(SuperClass)),
	has_description(SuperClass, Descr).

%%
instance_of_description(S, value(P,O)) ?>
	var(P),
	triple(S,P,O),
	(	is_object_property(P)
	;	is_data_property(P)
	).
  
instance_of_description(S, value(P,O)) ?>
	ground(P),
	triple(S,P,O).

instance_of_description(S, Descr) ?>
	ground(S),
	has_type(S, SType),
	subclass_of(SType, Descr).

%%
% Allow OWL descriptions in instance_of expressions.
%
% TODO: allow tell(instance_of(S,Descr))
%
instance_of(S,Descr) ?>
	pragma(is_owl_term(Descr)),
	instance_of_description(S,Descr).

%%
holds_description(S,P,only(O))      ?+> instance_of(S,only(P,O)).
holds_description(S,P,some(O))      ?+> instance_of(S,some(P,O)).
holds_description(S,P,value(O))     ?+> instance_of(S,value(P,O)).
holds_description(S,P,min(M,O))     ?+> instance_of(S,min(P,M,O)).
holds_description(S,P,max(M,O))     ?+> instance_of(S,max(P,M,O)).
holds_description(S,P,exactly(M,O)) ?+> instance_of(S,exactly(P,M,O)).
holds_description(S,P,value(O))     ?+> instance_of(S,value(P,O)).

%%
% Allow OWL descriptions in holds expressions.
%
holds(S,P,O) ?>
	pragma(\+ is_owl_term(O)),
	instance_of_description(S, value(P,O)).

holds(S,P,Descr) ?+>
	pragma(is_owl_term(Descr)),
	holds_description(S,P,Descr).


%% is_a(+Resource,?Type) is nondet.
%
% Wrapper around instance_of, subclass_of, and subproperty_of.
% Using this is a bit slower as an additional type check
% is needed.
% For example: `Cat is_a Animal` and `Nibbler is_a Cat`.
% 
% Note that contrary to wrapped predicates, is_a/2 requires
% the Resource to be ground.
%
% @param Resource a RDF resource
% @param Type the type of the resource
%
is_a(A,B) ?>
	ground(A),
	is_individual(A),
	!,
	instance_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_class(A),
	!,
	subclass_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_property(A),
	!,
	subproperty_of(A,B).

