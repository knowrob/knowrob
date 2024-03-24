:- module(mongolog_semweb,
	[ is_resource(r),
	  is_property(r),
	  is_literal(r),
	  is_datatype(r),
	  is_class(r),
	  is_restriction(r),
	  is_all_disjoint_classes(r),
	  is_object_property(r),
	  is_data_property(r),
	  is_functional_property(r),
	  is_transitive_property(r),
	  is_symmetric_property(r),
	  is_individual(r),

	  has_type(r,r),
	  has_range(r,r),
	  has_domain(r,r),
	  has_label(r,?),
	  has_comment(r,?),
	  rdf_list(r,t),

	  instance_of(r,r),    % ?Individual, ?Class
	  subclass_of(r,r),    % ?Class, ?SuperClass
	  subproperty_of(r,r), % ?Property, ?SuperProperty

	  disjoint_with(r,r),
	  disjoint_with_direct(r,r),
	  has_equivalent_class(r,r),
	  has_inverse_property(r,r),
	  has_property_chain(r,t),
	  same_as(r,r),

	  class_expr(r,t),
	  instance_of_expr(r,t),
	  subclass_of_expr(r,t)
	]).
/** <module> Monglog predicates related to semantic web.

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'), [ rdf_equal/2 ]).
:- use_module('client',
        [ mng_distinct_values/4
        , mng_find/4
        , mng_get_dict/3
        , mng_store/3
        ]).
:- use_module(library('scope')).

		 /*******************************
		  *            RDFS             *
		  *******************************/

%% has_type(+Resource,?Type) is semidet.
%
% rdf:type is an instance of rdf:Property that is used to
% state that a resource is an instance of a class.
%
% @param Resource a RDF resource
% @param Type a rdf:type of the resource
%
has_type(Resource,Type) ?+> triple(Resource, rdf:type, Type).

%% instance_of(?Entity,?Type) is nondet.
%
% Same as has_type/2.
%
% @param Entity a named individual
% @param Type the type of the entity
%
instance_of(Resource,Type) ?+> triple(Resource, rdf:type, Type).

%% is_resource(+Entity) is semidet.
%
% All things described by RDF are called resources,
% and are instances of the class rdfs:Resource.
% This is the class of everything.
% All other classes are subclasses of this class.
% rdfs:Resource is an instance of rdfs:Class.
%
% @param Entity An entity IRI.
%
is_resource(Entity) ?+> has_type(Entity, rdfs:'Resource').

%% is_property(+Entity) is semidet.
%
% rdf:Property is the class of RDF properties.
% rdf:Property is an instance of rdfs:Class.
%
% @param Entity An entity IRI.
%
is_property(Entity) ?+> has_type(Entity, rdf:'Property').

%% is_literal(+Entity) is semidet.
%
% The class rdfs:Literal is the class of literal values such as
% strings and integers. Property values such as textual strings
% are examples of RDF literals.
%
% rdfs:Literal is an instance of rdfs:Class.
% rdfs:Literal is a subclass of rdfs:Resource.
%
% @param Entity An entity IRI.
%
is_literal(Entity) ?+> has_type(Entity, rdfs:'Literal').

%% is_datatype(+Entity) is semidet.
%
% rdfs:Datatype is the class of datatypes.
% All instances of rdfs:Datatype correspond to the RDF model
% of a datatype described in the RDF Concepts specification.
% rdfs:Datatype is both an instance of and a subclass of rdfs:Class.
% Each instance of rdfs:Datatype is a subclass of rdfs:Literal.
%
% @param Entity An entity IRI.
%
is_datatype(Entity) ?+> has_type(Entity, rdfs:'Datatype').

%% has_range(?Property,?Range) is nondet.
%
% The range of a property globally restricts values
% of the property to instances of the range.
%
% @param Property a property
% @param Range the range of the property
%
has_range(Property,Range) ?+> triple(Property, rdfs:range, Range).

%% has_domain(?Property,?Domain) is nondet.
%
% The domain of a property globally restricts hosts
% of the property to instances of the domain.
%
% @param Property a property
% @param Domain the range of the property
%
has_domain(Property,Domain) ?+> triple(Property, rdfs:domain, Domain).

%% has_label(+Resource,?Comment) is semidet.
%
% rdfs:label is an instance of rdf:Property that may be used
% to provide a human-readable version of a resource's name.
%
% @param Resource a RDF resource
% @param Label a label atom
%
has_label(Resource,Label) ?+> annotation(Resource, rdfs:label, Label).

%% has_comment(+Resource,?Comment) is semidet.
%
% rdfs:comment is an instance of rdf:Property that may be used
% to provide a human-readable description of a resource.
%
% @param Resource a RDF resource
% @param Comment a comment atom
%
has_comment(Resource,Comment) ?> annotation(Resource, rdfs:comment, Comment).

%% subclass_of(?Class,?SuperClass) is nondet.
%
% The subclass-of relation (rdfs:subClassOf).
% For example: `Cat subclass_of Animal`.
%
% @param Class a class IRI
% @param SuperClass a class IRI
%
subclass_of(A,B) ?+> triple(A, rdfs:subClassOf, B).

%% subproperty_of(?Property,?SuperProperty) is nondet.
%
% The subproperty-of relation (rdfs:subPropertyOf).
%
% @param Property a property IRI
% @param SuperProperty a property IRI
%
subproperty_of(A,B) ?+> triple(A, rdfs:subPropertyOf, B).

%% rdf_list(+RDF_list, -Pl_List) is semidet.
%
% Read a RDF list into a Prolog list.
%
rdf_list(RDF_list, Pl_List) ?>
	ground(RDF_list),
	findall(X,
		(	triple(RDF_list, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		), Pl_List).

rdf_list(RDF_list, Pl_List) +>
	pragma((ground(Pl_List),
		    model_RDFS:rdf_list_expand(RDF_list, Pl_List, Expanded))),
	call([ has_type(RDF_list, rdf:'List') | Expanded ]).

% expand list into series of triple/3 predicates
:- rdf_meta(rdf_list_expand(r,t,t)).
rdf_list_expand(rdf:nil, [], []) :- !.
rdf_list_expand(This, [Child|Rest],
		[ triple(This, rdf:first, Child),
		  triple(This, rdf:rest, Next) | Xs ]) :-
	rdf_list_expand(Next, Rest, Xs).

%% rdf_list_head(+SubList, ?ListHead)
%
% True if ListHead is the head of the rdf list SubList.
%
rdf_list_head(SubList, ListHead) ?>
	findall(X,
		(	X = SubList
		;	triple(X, transitive(rdf:rest), SubList)
		),  ListHeads),
	length(ListHeads,NumHeads),
	Index is NumHeads-1,
	nth0(Index, ListHeads, ListHead).

		 /*******************************
		  *            OWL              *
		  *******************************/

%% is_class(+Entity) is semidet.
%
% True for OWL classes.
%
% @param Entity An entity IRI.
%
is_class(Entity) ?+> has_type(Entity, owl:'Class').

%% is_restriction(+Entity) is semidet.
%
% True for OWL restrcitions.
%
% @param Entity An entity IRI.
%
is_restriction(Entity) ?+> has_type(Entity, owl:'Restriction').

%% is_individual(+Entity) is semidet.
%
% True for OWL individuals.
%
% @param Entity An entity IRI.
%
is_individual(Entity) ?+> has_type(Entity, owl:'NamedIndividual').

%% is_object_property(+Entity) is semidet.
%
% True for OWL object properties.
%
% @param Entity An entity IRI.
%
is_object_property(Entity) ?+> has_type(Entity, owl:'ObjectProperty').

%% is_functional_property(+Entity) is semidet.
%
% True for functional properties.
%
% @param Entity An entity IRI.
%
is_functional_property(Entity) ?+> has_type(Entity, owl:'FunctionalProperty').

%% is_transitive_property(+Entity) is semidet.
%
% True for transitive properties.
%
% @param Entity An entity IRI.
%
is_transitive_property(Entity) ?+> has_type(Entity, owl:'TransitiveProperty').

%% is_symmetric_property(+Entity) is semidet.
%
% True for symmetric properties.
%
% @param Entity An entity IRI.
%
is_symmetric_property(Entity) ?+> has_type(Entity, owl:'SymmetricProperty').

%% is_data_property(+Entity) is semidet.
%
% True iff Entity is an datatype property IRI.
%
% @param Entity An entity IRI.
%
is_data_property(Entity) ?+> has_type(Entity, owl:'DatatypeProperty').

%% is_all_disjoint_classes(?AllDisjointClasses) is nondet.
%
% True for OWL2 AllDisjointClasses
%
% @param Entity An entity IRI.
%
is_all_disjoint_classes(Entity) ?+> has_type(Entity, owl:'AllDisjointClasses').

%% has_inverse_property(?Property, ?Inverse) is nondet.
%
% The inverse-of relation between two properties.
%
% @param Property property resource
% @param Inverse inverse of the property
%
has_inverse_property(P, P_inv) ?+> triple(P, owl:inverseOf, P_inv).
has_inverse_property(P, P_inv) ?> triple(P_inv, owl:inverseOf, P).

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
	triple(P, owl:propertyChainAxiom, RDFList),
	findall(X,
		(	triple(RDFList, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		), Chain).

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
	once(( ground(X); ground(Y) )),
	(	triple(X, owl:equivalentClass, Y)
	;	triple(Y, owl:equivalentClass, X)
	).

has_equivalent_class(X,Y) ?> triple(X, transitive(owl:equivalentClass), Y).
has_equivalent_class(X,Y) ?> triple(Y, transitive(owl:equivalentClass), X).
has_equivalent_class(X,Y) +> triple(X, owl:equivalentClass, Y).

%% same_as(?X, ?Y) is nondet.
%
% True if X and Y are  identical   or  connected by the owl:sameAs
% relation. Considers owl:sameAs transitive and symmetric.
%
same_as(X,Y) ?> X = Y.
same_as(X,Y) ?> triple(X, owl:sameAs, Y).
same_as(X,Y) ?> triple(Y, owl:sameAs, X).
same_as(X,Y) +> triple(X, owl:sameAs, Y).

%% disjoint_with_direct(+Cls, ?Disjoint) is semidet.
%
% True if there is a disjointness axioms for Cls and
% Disjoint, not taking into account the super classes of both.
%
disjoint_with_direct(A,B) ?>
	% OWL1 disjointness axioms
	(	triple(A, owl:disjointWith, B)
	;	triple(B, owl:disjointWith, A)
	).

disjoint_with_direct(A,B) ?>
	% OWL2 disjointness axioms
	ground(A),
	% iterate over all lists where A is an element.
	% here we assume that this is more efficient compared to
	% iterating over all AllDisjointClasses resources and parsing
	% a list for each.
	triple(SubList, rdf:first, A),
	rdf_list_head(SubList, ListHead),
	% only proceed for lists of OWL2 disjointness axioms
	triple(DC, owl:members, ListHead),
	is_all_disjoint_classes(DC),
	% yield all members of the list except of A
	rdf_list(ListHead, PrologList),
	member(B, PrologList),
	B \== A.

%%
disjoint_with_indirect(A,B) ?>
	% ground(A),
	triple(A, transitive(rdfs:subClassOf), SupA),
	disjoint_with_direct(SupA, SupB),
%	triple(B, reflexive(rdfs:subClassOf), SupB).
	(	var(B) -> B=SupB
	;	(B=SupB ; triple(B, rdfs:subClassOf, SupB))
	).

%% disjoint_with(+Class1, ?Class2) is nondet.
%
% Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
% properties and the OWL2 AllDisjointClasses into account.
%
% @param Class1 OWL class
% @param Class2 a disjoint OWL class
%
%disjoint_with(A,B) :-
%  ground([A,B]), A=B, !, fail.

disjoint_with(A,B) ?> disjoint_with_direct(A,B).
disjoint_with(A,B) ?> ground(A), disjoint_with_indirect(A,B).
disjoint_with(A,B) ?> ground(B), disjoint_with_indirect(B,A).

		 /*******************************
		 *	    CLASS EXPRESSIONS		*
		 *******************************/

%% class_expr(+Class,-Descr) is semidet.
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
class_expr(R, only(P,O)) ?>
    triple(R, owl:allValuesFrom,O),
    triple(R, owl:onProperty, P).

class_expr(R, some(P,O)) ?>
    triple(R, owl:someValuesFrom,O),
    triple(R, owl:onProperty, P).

class_expr(R, value(P,O)) ?>
    triple(R, owl:hasValue,O),
    triple(R, owl:onProperty, P).

class_expr(R, min(P,M,O)) ?>
    triple(R, owl:minQualifiedCardinality,M),
    triple(R, owl:onProperty, P),
    triple(R, owl:onClass, O).

class_expr(R, max(P,M,O)) ?>
    triple(R, owl:maxQualifiedCardinality,M),
    triple(R, owl:onProperty, P),
    triple(R, owl:onClass, O).

class_expr(R, exactly(P,M,O)) ?>
    triple(R, owl:qualifiedCardinality,M),
    triple(R, owl:onProperty, P),
    triple(R, owl:onClass, O).

class_expr(R, min(P,M)) ?>
    triple(R, owl:minCardinality,M),
    triple(R, owl:onProperty, P).

class_expr(R, max(P,M)) ?>
    triple(R, owl:maxCardinality,M),
    triple(R, owl:onProperty, P).

class_expr(R, exactly(P,M)) ?>
    triple(R, owl:cardinality,M),
    triple(R, owl:onProperty, P).

class_expr(UnionClass, union_of(List_pl)) ?>
	triple(UnionClass, owl:unionOf, List_rdf),
	findall(X,
		(	triple(List_rdf, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		), List_pl).

class_expr(IntersectionClass, intersection_of(List_pl)) ?>
	triple(IntersectionClass, owl:intersectionOf, List_rdf),
	findall(X,
		(	triple(List_rdf, reflexive(transitive(rdf:rest)), Ys),
			triple(Ys, rdf:first, X)
		), List_pl).

class_expr(ComplementClass, complement_of(Class)) ?+>
	triple(ComplementClass, owl:complementOf, Class).


%%
subclass_of_expr(Class, Descr) ?>
	ground(Class),
	triple(Class, transitive(rdfs:subClassOf), SuperClass),
	class_expr(SuperClass, Descr).

%%
instance_of_expand(_S, intersection_of([]), []) :- !.
instance_of_expand(S,  intersection_of([X|Xs]), [instance_of(S,X)|Ys]) :-
	instance_of_expand(S, intersection_of(Xs), Ys).

instance_of_expand(S, union_of(Classes), Expanded) :-
	instance_of_expand_union(S, Classes, List),
	semicolon_list(Expanded, List).

instance_of_expand_union(_S, [], []) :- !.
instance_of_expand_union(S, [X|Xs], [instance_of(S,X)|Ys]) :-
	instance_of_expand_union(S, Xs, Ys).

%%
instance_of_expr(S, Descr) ?>
	pragma(compound(Descr)),
	pragma(mongolog_semweb:instance_of_expand(S, Descr, Expanded)),
	!,
	call(Expanded).

instance_of_expr(S, Descr) ?>
	ground(S),
	has_type(S, SType),
	subclass_of_expr(SType, Descr).
