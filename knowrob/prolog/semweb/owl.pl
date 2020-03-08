/*  owl.pl

    Part of SWI-Prolog

    Author:        Jan Wielemaker
    E-mail:        jan@swi.psy.uva.nl
    WWW:           http://www.swi-prolog.org
    Copyright (C): 1985-2002, University of Amsterdam

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    As a special exception, if you link this library with other files,
    compiled with a Free Software compiler, to produce an executable, this
    library does not by itself cause the resulting executable to be covered
    by the GNU General Public License. This exception does not however
    invalidate any other reasons why the executable file might be covered by
    the GNU General Public License.
*/

:- module(t20_owl,
	  [ owl_restriction_on/2,	% ?Class, ?Restriction
	    owl_restriction_on/3,	% ?Class, ?Property, ?Restriction
	    owl_restriction_object_domain/2,
	    owl_restriction_subject_type/2,
	    owl_merged_restriction/3,	% ?Class, ?Property, ?Restriction
	    owl_restriction/2,		% +Resource, -Restriction,
	    owl_restriction_assert/2,		% +Restriction, -Resource,
	    owl_restriction_assert/3,
	    owl_unsatisfied_restriction/2,	% +Resource, ?Restriction
	    owl_unsatisfied_restriction/3,	% +Resource, ?Restriction, ?DB
	    owl_description/2,		% +Resource, -Description
	    owl_description_recursive/2,		% +Resource, -Description
	    owl_description_assert/2,		% +Restriction, -Resource,
	    owl_description_assert/3,
	    owl_description_list_assert/2,
	    owl_description_list_assert/3,
	    owl_property_range_on_resource/3,	% +Resource, +Pred, -Range
	    owl_property_range_on_subject/3,	% +Subject, +Pred, -Range
	    owl_property_range_on_class/3,		% +Class, +Pred, -Range
	    owl_cardinality_on_resource/4,
	    owl_cardinality_on_subject/4, % +Subject, +Predicate, +Description, -Card
	    owl_cardinality_on_class/4,	% idem BJW
	    owl_cardinality/3,
	    owl_cardinality/4,
	    owl_cardinality/5,
	    owl_satisfies/2,		% +Spec, +Resource
	    owl_individual_of/2,	% ?Resource, +Description
	    owl_individual_of/3,
	    owl_individual_of_description/2,
	    owl_individual_of_all/2,
	    owl_individual_of_all/3,
	    owl_individual_from_range/2,
	    owl_inverse_property/2,
	    owl_inverse_property_chain/2,
	    owl_inverse_functional/1,
	    owl_most_specific_predicate/2,
	    owl_most_specific/2,
	    owl_common_ancestor/2,
	    owl_direct_subclass_of/2,	% ?Resource, ?Class
	    owl_subclass_of/2,		% ?Class, ?Super
	    owl_subproperty_of/2,
	    owl_has/3,			% ?Subject, ?Predicate, ?Object
	    owl_has/4,
	    owl_has_direct/3,		% ?Subject, ?Predicate, ?Object
	    owl_same_as/2,		% ?X, ?Y
	    owl_disjoint_with/2,        % ?Class1, ?Class2
	    owl_find/5,			% +For, +Dom, ?Props, +Method, -Subj
	    non_negative_integer/4,
	    non_negative_int/2,
	    rdf_assert_literal/3,
	    rdf_assert_literal/4,
	    rdf_phas/3,
owl_has_direct_type/2,
	    owl_property_range_clear_cache/2
	  ]).
:- use_module(library(lists)).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).


		 /*******************************
		 *	    EXPANSION		*
		 *******************************/

%	user:goal_expansion(+NSGoal, -Goal)
%
%	This predicate allows for writing down rdf queries in a friendly
%	name-space fashion.

:- multifile
	user:goal_expansion/2.

:- dynamic
	owl_property_range_cached/3,
	owl_cardinality_cached/4.

:- rdf_meta
	owl_restriction_on(r, t),
	owl_restriction_on(r, r, r),
	owl_restriction_object_domain(r, r),
	owl_restriction_subject_type(r, r),
	owl_merged_restriction(r, r, t),
	owl_restriction(r, t),
	owl_restriction_assert(t, r),
	owl_restriction_assert(t, r, +),
	owl_unsatisfied_restriction(r, r),
	owl_unsatisfied_restriction(r, r, +),
	owl_description(r, t),
  owl_description_recursive(r, -),
	owl_description_assert(t, t),
	owl_description_assert(t, t, +),
	owl_property_range_on_resource(r, r, -),
	owl_property_range_on_subject(r, r, -),
	owl_property_range_on_class(r, r, -),
	owl_cardinality_on_resource(r, r, r, -),
	owl_cardinality_on_subject(r, r, r, -),
	owl_cardinality_on_class(r, r, r, -),
	owl_cardinality(r,r,?),
	owl_cardinality(r,r,r,?),
	owl_satisfies(r, t),
	owl_individual_of(r, t),
	owl_individual_of_description(r, t),
	owl_individual_of_all(t,r),
	owl_individual_of_all(t,r,+),
	owl_individual_from_range(r, t),
	owl_direct_subclass_of(r, r),
	owl_subclass_of(r, r),
	owl_subproperty_of(r, r),
	owl_has(r, r, o),
	owl_most_specific_predicate(t,t),
	owl_most_specific(t,t),
	owl_common_ancestor(t,r),
	owl_has_direct(r, r, o),
	owl_inverse_property(r, r),
	owl_inverse_functional(r),
	owl_inverse_property_chain(t, t),
	owl_same_as(r, r),
	owl_disjoint_with(r, r),
	owl_find(+, t, t, +, -),
	rdf_assert_literal(r, r, +),
	rdf_assert_literal(r, r, +, +),
	owl_db_has(+,r,r,t).


		 /*******************************
		 *	       FACTS		*
		 *******************************/

%	owl_individual(?IndividualID, ?Type)
%	owl_property(?IndividualID, ?PropertyID, ?PropertyValue)
%	owl_same_individual(?IndividualID1, ?IndividualID2)
%	owl_different_individual(?IndividualID1, ?IndividualID2)


		 /*******************************
		 *	      AXIOMS		*
		 *******************************/

%	owl_class(?ClassID, ?Super)
%	owl_class_modality(?ClassID, ?Modality)
%	owl_same_class(?ClassID1, ?ClassID2)


		 /*******************************
		 *	   RESTRICTIONS		*
		 *******************************/

%%	owl_restriction_on(+ClassID,
%%			   -Restriction:restriction(?PropertyID, ?Restriction)) is nondet.
%
%	Enumerate the restrictions that apply to PropertyID for Class.
%	Restriction is one of
%
%		* all_values_from(Class)
%		* some_values_from(Class)
%		* has_value(Value)
%		* cardinality(Min, Max, Class)

owl_restriction_on(Class, Restriction) :-
	owl_subclass_of(Class, Super),
	(   rdfs_individual_of(Super, owl:'Restriction'),
	    owl_restriction(Super, Restriction)
	;   Restriction = restriction(Property,
				      all_values_from(Range)),
	    rdf_phas(Property, rdfs:domain, Super),
	    (	rdf_phas(Property, rdfs:range, Range)
	    *-> true
	    ;	rdf_equal(Range, rdfs:'Resource')
	    )
	).

%% owl_restriction_on(?Resource, ?Property, ?Restriction)
%
owl_restriction_on(Resource, Property, Restriction) :-
  rdfs_individual_of(Resource, Restriction),
  rdfs_individual_of(Restriction, owl:'Restriction'),
  rdf_has(Restriction, owl:onProperty, Property).

%% owl_restriction_object_domain(?Resource, ?Domain)
%
owl_restriction_object_domain(RestrictionID, Domain) :-
	rdf_has(RestrictionID, owl:allValuesFrom, Domain), !.
owl_restriction_object_domain(RestrictionID, Domain) :-
	rdf_has(RestrictionID, owl:hasValue, Domain), !.
owl_restriction_object_domain(RestrictionID, Domain) :-
	rdf_has(RestrictionID, owl:someValuesFrom, Domain), !.
owl_restriction_object_domain(RestrictionID, Domain) :-
	restriction_facet(RestrictionID, cardinality(Min, _, Domain)),
	Min > 0, !.

%% owl_restriction_subject_type(?Restriction, ?SubjectType)
%
owl_restriction_subject_type(Restriction, SubjectType) :-
  rdf_has(Restriction, owl:onProperty, P),
  owl_restriction_object_domain(Restriction, Domain),
  owl_inverse_property(P,P_inv),
  owl_property_range_on_resource(Domain, P_inv, SubjectType).

%%	owl_restriction(+Resource, -Prolog) is det.
%
%	Translate Resource, an individual of owl:restriction into a Prolog term.
%
%	@see owl_restriction_on/2 for the Prolog representation.

owl_restriction(RestrictionID, restriction(Property, Restriction)) :-
	rdf_has(RestrictionID, owl:onProperty, Property),
	restriction_facet(RestrictionID, Restriction).

restriction_facet(RestrictionID, R) :-
	(   rdf_has(RestrictionID, owl:allValuesFrom, Class)
	->  R = all_values_from(Class)
	;   rdf_has(RestrictionID, owl:someValuesFrom, Class)
	->  R = some_values_from(Class)
	).
restriction_facet(RestrictionID, has_value(Value)) :-
	rdf_has(RestrictionID, owl:hasValue, Value).
restriction_facet(R, cardinality(Min, Max, Class)) :-
	once(( rdf_has(R, owl:onClass, Class) ;
	       Class='http://www.w3.org/2002/07/owl#Thing' )),
	(   once(( rdf_has(R, owl:cardinality, literal(Atom)) ;
	           rdf_has(R, owl:qualifiedCardinality, literal(Atom)) ))
	->  non_negative_integer(Atom, Min, R, owl:qualifiedCardinality),
	    Max = Min
	;   rdf_has(R, owl:minCardinality, literal(MinAtom))
	->  non_negative_integer(MinAtom, Min, R, owl:minCardinality),
	    (   rdf_has(R, owl:maxCardinality, literal(MaxAtom))
	    ->  non_negative_integer(MaxAtom, Max, R, owl:maxCardinality)
	    ;	Max = inf
	    )
	;   rdf_has(R, owl:maxCardinality, literal(MaxAtom))
	->  non_negative_integer(MaxAtom, Max, R, owl:maxCardinality),
	    Min = 0
	).

%%	owl_restriction_assert(+Prolog, -Resource) is det.
%
owl_restriction_assert(R, Id) :-
  owl_restriction_assert(R, Id, user).
owl_restriction_assert(restriction(P,all_values_from(Cls)), Id, Graph) :-
	owl_description_assert(Cls, ClsId, Graph),
	owl_assert_description('http://www.w3.org/2002/07/owl#Restriction', Id, Graph),
	rdf_assert(Id, rdf:type, owl:'Restriction', Graph),
	rdf_assert(Id, owl:onProperty, P, Graph),
	rdf_assert(Id, owl:allValuesFrom, ClsId, Graph), !.
owl_restriction_assert(restriction(P,some_values_from(Cls)), Id, Graph) :-
	owl_description_assert(Cls, ClsId, Graph),
	owl_assert_description('http://www.w3.org/2002/07/owl#Restriction', Id, Graph),
	rdf_assert(Id, rdf:type, owl:'Restriction', Graph),
	rdf_assert(Id, owl:onProperty, P, Graph),
	rdf_assert(Id, owl:someValuesFrom, ClsId, Graph), !.
owl_restriction_assert(restriction(P,cardinality(Card,Card,Cls)), Id, Graph) :- !,
	owl_description_assert(Cls, ClsId, Graph),
	owl_assert_description('http://www.w3.org/2002/07/owl#Restriction', Id, Graph),
	rdf_assert(Id, rdf:type, owl:'Restriction', Graph),
	rdf_assert(Id, owl:onProperty, P, Graph),
	rdf_assert(Id, owl:onClass, ClsId, Graph),
	rdf_assert_literal(Id, owl:cardinality, Card, Graph), !.
owl_restriction_assert(restriction(P,cardinality(Min,Max,Cls)), Id, Graph) :-
	owl_description_assert(Cls, ClsId, Graph),
	owl_assert_description('http://www.w3.org/2002/07/owl#Restriction', Id, Graph),
	rdf_assert(Id, rdf:type, owl:'Restriction', Graph),
	rdf_assert(Id, owl:onProperty, P, Graph),
	rdf_assert(Id, owl:onClass, ClsId, Graph),
	once(( Min is 0 ;  rdf_assert_literal(Id, owl:minCardinality, Min, Graph) )),
	once(( Max = inf ; rdf_assert_literal(Id, owl:maxCardinality, Max, Graph) )), !.
owl_restriction_assert(restriction(P,has_value(V)), Id, Graph) :-
	owl_assert_description('http://www.w3.org/2002/07/owl#Restriction', Id, Graph),
	rdf_assert(Id, rdf:type, owl:'Restriction', Graph),
	rdf_assert(Id, owl:onProperty, P, Graph),
	rdf_assert(Id, owl:hasValue, V, Graph), !.

owl_assert_description(Type, Instance) :-
  owl_assert_description(Type, Instance, user).
owl_assert_description(Type, Instance, Graph) :-
	rdf_node(Instance),
	rdf_assert(Instance, rdf:type, Type, Graph).

%	non_negative_integer(+Atom, -Integer, +Subject, +Predicate)
%
%	Deduce integer value from rdf(Subject, Predicate, literal(Atom))
%	and if a conversion error occurs warn compatible to the rdfs_validate
%	library.
%
%	TBD: If argument is typed we should check the type is compatible
%	to xsd:nonNegativeInteger.

non_negative_integer(type(_Type, Atom), Int, S, P) :-
	nonvar(Atom), !,
	non_negative_integer(Atom, Int, S, P).
non_negative_integer(Atom, Int, _, _) :-
	catch(atom_number(Atom, Int), _, fail), !,
	integer(Int),
	Int >= 0.
non_negative_integer(Atom, _, S, P) :-
	rdf_equal(xsd:nonNegativeInteger, Range),
	rdf_global_id(P, Pred),
	print_message(error,
		      rdf_illegal_object(S,Pred,literal(Atom),Range)),
	fail.

%%	owl_merged_restriction(+Class, ?Property, ?Restriction) is nondet.
%
%	As owl_restriction_on/2, but combines multiple restrictions into
%	the   least   strict   restriction   satisfying   the   declared
%	restrictions.

owl_merged_restriction(Class, Property, Restriction) :-
	setof(Decl,
	      owl_restriction_on(Class, restriction(Property, Decl)),
	      Decls),
	join_decls(Decls, Minimal),
	member(Restriction, Minimal).

%	input is sorted, thus the following holds:
%
%		cardinality < has_value < values_from

join_decls([], []).
join_decls([cardinality(Min1, Max1), cardinality(Min2, Max2)|T], Set) :- !,
	Min is max(Min1, Min2),
	max_cardinality(Max1, Max2, Max),
	join_decls([cardinality(Min, Max)|T], Set).
join_decls([has_value(Value)|T], [has_value(Value)]) :- !,
	satisfies_restrictions(T, Value).
join_decls([values_from(AS1, C1), values_from(AS2, C2)|T], Set) :-
	merge_values_from(AS1, C1, AS2, C2, AS, C), !,
	join_decls([values_from(AS, C)|T], Set).
join_decls([H|T0], [H|T]) :-
	join_decls(T0, T).

max_cardinality(infinite, Min, Min) :- !.
max_cardinality(Min, infinite, Min) :- !.
max_cardinality(Min1, Min2, Min) :-
	Min is min(Min1, Min2).

%	satisfies_restrictions(+Restrictions, +Value)
%
%	See whether Value satisfies all restrictions, so we can indeed
%	use it as a value.

satisfies_restrictions([], _).
satisfies_restrictions([H|T], Value) :-
	satisfies_restriction(H, Value),
	satisfies_restrictions(T, Value).

satisfies_restriction(has_value(Value), Value).
satisfies_restriction(values_from(some, _), _).
satisfies_restriction(values_from(all, Class), Value) :-
	rdfs_individual_of(Value, Class).

:- rdf_meta rdf_phas(r,r,o).

rdf_phas(Property, P, O) :-
	rdfs_subproperty_of(Property, Super),
	rdf_has(Super, P, O2), !,
	O = O2.

%	merge_values_from(+AllSome2, +C1, +AllSome2, +C2, -AllSome, -C)
%
%	Merge multiple allValuesFrom and someValuesFrom restrictions.
%	This needs some thought, but as we don't need it for the MIA
%	tool right now we'll leave it.

merge_values_from(all, C1, all, C2, all, C) :-
	rdfs_subclass_of(C, C1),
	rdfs_subclass_of(C, C2).

%%	owl_property_range_on_resource(+Resource, +Pred, -Range) is semidet.
%
owl_property_range_on_resource(Resource, Predicate, Range) :-
	rdfs_individual_of(Resource, owl:'Class'), !,
	owl_property_range_on_class(Resource, Predicate, Range).
owl_property_range_on_resource(Resource, Predicate, Range) :-
	owl_property_range_on_subject(Resource, Predicate, Range).

%%	owl_property_range_on_subject(+Subject, +Pred, -Range) is semidet.
%
owl_property_range_on_subject(Subject, Predicate, Range) :-
	range_on_subject(Subject, Predicate, Range) *->
		true ; Range='http://www.w3.org/2002/07/owl#Thing'.

range_on_subject(Subject, Predicate, Range) :-
	% infer range based on value type of functional property
	owl_inverse_property(Predicate, Predicate_inv),
	once(( rdfs_individual_of(Predicate, owl:'FunctionalProperty') ;
	       rdfs_individual_of(Predicate_inv, owl:'InverseFunctionalProperty') )),
	once(( rdf_has(Subject, Predicate, O) ;
	       rdf_has(O, Predicate_inv, Subject) )),
	rdf_has(O, rdf:type, Range),
	Range \= 'http://www.w3.org/2002/07/owl#NamedIndividual'.

range_on_subject(Subject, Predicate, Range) :-
	rdf_has(Subject, rdf:type, Class),
	Class \= 'http://www.w3.org/2002/07/owl#NamedIndividual',
	owl_property_range_on_class(Class, Predicate, Range).

%%	owl_property_range_on_class(+Subject, +Pred, -Range) is semidet.
%
% NOTE(DB): Ranges on classes are only inferred once and then cached because inferring
%           the range based on cardinality restrictions is very expensive.
% TODO(DB): only keep last n inferred ranges in the cache
% TODO(DB): use `rdf_generation` to check if cache needs to be whiped!
%
owl_property_range_on_class('http://www.w3.org/2002/07/owl#Thing', _,
                            'http://www.w3.org/2002/07/owl#Thing') :- !.
owl_property_range_on_class(Class, Predicate, Range) :-
	owl_property_range_cached(Class, Predicate, Ranges_cached) ->
	member(Range, Ranges_cached) ; (
		% avoid cycles
		assertz(owl_property_range_cached(Class,Predicate,['http://www.w3.org/2002/07/owl#Thing'])),
		% cache miss -> infer range
		findall(X, owl_property_range_on_class_(Class,Predicate,X), Ranges_inferred),
		list_to_set(Ranges_inferred,Ranges_inferred_s),
		retractall(owl_property_range_cached(Class,Predicate,_)),
		assertz(owl_property_range_cached(Class,Predicate,Ranges_inferred_s)),
		member(Range, Ranges_inferred_s)
	).

owl_property_range_on_class_(Class, Predicate, Range) :-
	findall(R, (
		range_on_class(Class, Predicate, R) ;
		rdf_phas(Predicate, rdfs:range, R)
	), Ranges),
	list_to_set(Ranges,Set),
	( range_on_cardinality_(Class, Predicate, Set, Range) *->
		true ; Range='http://www.w3.org/2002/07/owl#Thing' ).

owl_property_range_clear_cache(Class, Predicate) :-
	retractall(owl_property_range_cached(Class,Predicate,_)).

range_on_cardinality_(Class, Predicate, [X|Xs], Range) :-
	owl_most_specific([X|Xs], R_specific),
	( range_on_cardinality(Class, Predicate, R_specific, Range) *->
		true ; Range=R_specific ).

range_on_cardinality(_, _, 'http://www.w3.org/2002/07/owl#Thing',
                           'http://www.w3.org/2002/07/owl#Thing') :- !.
range_on_cardinality(_, _, literal(X), literal(X)) :- !.
range_on_cardinality(_, _, literal(type(X,Y)), literal(type(X,Y))) :- !.
range_on_cardinality(Class, Predicate, RangeIn, RangeOut) :-
	% for each range, find terminal classes that are subclass of range
	bagof(X, owl_terminal_subclass_of(RangeIn, X), Terminals),
	length(Terminals, NumTerminals),
	% FIXME: There are potentially many terminal subclasses.
	%        Limit search to classes that have not more then 19 terminal subclasses
	%        Maybe better limit depth of search in owl_terminal_subclass_of?
	NumTerminals < 20, % bad smell magic number
	once(((
		% if class restriction, use infered inverse predicate range for cardinality computation
		rdfs_individual_of(Class, owl:'Restriction'),
		rdf_has(Class, owl:onProperty, P_restr),
		owl_restriction_object_domain(Class, Obj_Domain),
		owl_inverse_property(P_restr, P_inv),
		owl_property_range_on_resource(Obj_Domain, P_inv, CardCls)
	);(
		CardCls = Class
	))),
	% infer cardinality for each terminal class
	findall(X, (
		member(X,Terminals),
		( owl_cardinality_on_class(CardCls, Predicate, X, cardinality(Min,_)) -> Min > 0 ; true )
	), RangesOut),
	% if a terminal class was eliminated with cardinality=0 then create union class of remaining
	length(RangesOut, NumCandidates), NumCandidates < NumTerminals,
	(  RangesOut=[X]
	-> RangeOut=X
	;  owl_description_assert(union_of(RangesOut), RangeOut)
	).

range_on_class(Class, Predicate, Range) :-
	rdf_has(Class, owl:unionOf, Set),
	rdfs_list_to_prolog_list(Set, Members),
	findall(R, (
		member(Descr, Members),
		range_on_class(Descr, Predicate, R),
		R \= 'http://www.w3.org/2002/07/owl#Thing'
	), Ranges),
	% if each union member restricts the range
	length(Ranges, N), length(Members, N),
	owl_description_assert(union_of(Ranges), Range).

range_on_class(Class, Predicate, Range) :-
	rdf_has(Class, owl:intersectionOf, Set),
	rdfs_list_to_prolog_list(Set, Members),
	member(Descr, Members),
	range_on_class(Descr, Predicate, Range).

range_on_class(Class, Predicate, Range) :-
	rdfs_subclass_of(Class, RestrictionID),
	once((
		rdfs_individual_of(RestrictionID, owl:'Restriction'),
		owl_restriction(RestrictionID, Restr),
		range_on_restriction(Restr, Predicate, Range)
	)).

range_on_restriction(restriction(Predicate, has_value(Range)),       Predicate, Range) :- !.
range_on_restriction(restriction(Predicate, all_values_from(Range)), Predicate, Range) :- !.
range_on_restriction(restriction(P,         Facet),                  Predicate, Range) :-
	P \= Predicate,
	once(( Facet=all_values_from(Cls) ;
	       Facet=some_values_from(Cls) ;
	     ( Facet=cardinality(Min,_,Cls), Min > 0 ) )),
	Cls \= 'http://www.w3.org/2001/XMLSchema#anyURI',
	Cls \= 'http://www.w3.org/2002/07/owl#Thing',
	owl_inverse_property(P, P_inv),
	% check if restricted class has range restriction for inverse property `P`,
	% and check if this inferred class description has a range restriction
	% for `Predicate`.
	owl_property_range_on_class(Cls, P_inv, Cls_P_inv_range),
	Cls_P_inv_range \= 'http://www.w3.org/2002/07/owl#Thing',
	(  owl_property_range_on_class(Cls_P_inv_range, Predicate, Range_inv) *->
	   true ; Range_inv = 'http://www.w3.org/2002/07/owl#Thing' ),
	Range=Range_inv.
	% NOTE(DB): disabled due to performance issues using DUL.
	%(  Range_inv \= 'http://www.w3.org/2002/07/owl#Thing' ->
	   %Range=Range_inv ; (
	   %owl_inverse_property(Predicate, Predicate_inv),
	   %owl_description_assert(restriction(Predicate_inv,
	                          %some_values_from(Cls_P_inv_range)), Range)
	%)).

		 /*******************************
		 *	    CARDINALITY		*
		 *******************************/

cardinality_on_property(Predicate, cardinality(0,1)) :-
	rdfs_individual_of(Predicate, owl:'FunctionalProperty').


owl_cardinality_on_resource(Resource, Predicate, Range, Cardinality) :-
	rdfs_individual_of(Resource, owl:'Class'), !,
	owl_cardinality_on_class(Resource, Predicate, Range, Cardinality).
owl_cardinality_on_resource(Resource, Predicate, Range, Cardinality) :-
	owl_cardinality_on_subject(Resource, Predicate, Range, Cardinality).

%%	owl_cardinality_on_subject(+Subject, +Pred, +Descr, -Card:cardinality(Min, Max)) is semidet.
%
%	Deduces the minimum and maximum cardinality for a property of a
%	resource.  This predicate may fail if no information is available.
%

owl_cardinality_on_subject(Subject, Predicate, Range, Cardinality) :-
  ground(Range), !,
	findall(C, cardinality_on_subject(Subject, Predicate, Range, C), L),
	join_decls(L, [Cardinality]).

owl_cardinality_on_subject(Subject, Predicate, Range, Cardinality) :-
  cardinality_on_subject(Subject, Predicate, Range, Cardinality).

cardinality_on_subject(Subject, Predicate, Range, C) :-
	rdf_has(Subject, rdf:type, Class),
	Class \= 'http://www.w3.org/2002/07/owl#NamedIndividual',
	owl_cardinality_on_class(Class, Predicate, Range, C).

%%	owl_cardinality_on_class(+Class, ?Predicate, ?Range, -Card:cardinality(Min, Max)) is semidet.
%
% TODO(DB): could also infer min cardinality based on restrictions on parent and siblings
%		--> accumulate max values and take difference to superclass min value (only if all direct subclasses restricted)
%
owl_cardinality_on_class(Class, Predicate, Range, Cardinality) :-
  ground(Range), !, (
	owl_cardinality_cached(Class, Predicate, Range, Cardinality_cached) *->
	Cardinality = Cardinality_cached; (
		% cache miss -> infer cardinality
		  owl_cardinality_on_class_(Class,Predicate,Range,Cardinality),
		  assertz(owl_cardinality_cached(Class,Predicate,Range,Cardinality))
	)).
  
owl_cardinality_on_class(Class, Predicate, Range, Cardinality) :-
	owl_cardinality_cached(Class, Predicate, Range_cached, Cardinality_cached) *->
	( Cardinality = Cardinality_cached, Range = Range_cached ); (
		% cache miss -> infer cardinality
		  forall(
		    owl_cardinality_on_class_(Class,Predicate,R,C),
		    assertz(owl_cardinality_cached(Class,Predicate,R,C))),
		  owl_cardinality_cached(Class, Predicate, Range, Cardinality)
	).

owl_cardinality_on_class_(Class, Predicate, Range, Cardinality) :-
	ground(Range),!,
	findall(C, (
		  cardinality_on_property(Predicate, C)
		; cardinality_on_class(Class, Predicate, Range, C)
		; cardinality_from_sibling_range(Class, Predicate, Range, C)
	), L),
	join_decls(L, [Cardinality]).

owl_cardinality_on_class_(Class, Predicate, Range, Cardinality) :-
	%ground(Range),
	%findall(C, (
	cardinality_on_class(Class, Predicate, Range, Cardinality)
	%), L),
	%join_decls(L, [Cardinality])
	.

cardinality_on_class(Class, Predicate, Range, cardinality(Min, Max)) :-
	rdfs_subclass_of(Class, RestrictionID),
	rdfs_individual_of(RestrictionID, owl:'Restriction'),
	rdf_has(RestrictionID, owl:onProperty, P),
	once( rdfs_subproperty_of(P,Predicate) ),
	restriction_facet(RestrictionID, cardinality(Min_restr, Max_restr, Descr)),
	(  Descr = Range
	-> ( Min is Min_restr, Max is Max_restr )
	;  ( rdfs_individual_of(Range, Descr), Min is 0, Max is Max_restr )
	).

cardinality_from_sibling_range(Class, Predicate, Range, cardinality(0, Max)) :-
	% infer cardinality of parent
	rdf_has(Range, rdfs:subClassOf, SR),
	once(rdf_has(SR, rdfs:subClassOf, _)),
	cardinality_on_class(Class, Predicate, SR, cardinality(_, Max_sr)),
	% infer cardinality of siblings
	findall(Min_other, (
		rdf_has(Range_other, rdfs:subClassOf, SR),
		Range_other \= Range,
		cardinality_on_class(Class, Predicate, Range_other, cardinality(Min_other, _))
	),	Min_others),
	Min_others \= [],
	% compute maximum cardinality from accumulated min cardinality of siblings
	sumlist(Min_others, Card_reserved),
	Allowed is Max_sr - Card_reserved,
	( Allowed < 0 -> Max is 0 ; Max is Allowed ).

%%	owl_unsatisfied_restriction(?Resource, +Restriction)
%	
%	True if Resource does not satisfy the class description Restriction.
%	

owl_unsatisfied_restriction(Resource, Restriction) :-
	owl_rdf_db(DB),
	owl_unsatisfied_restriction(Resource, Restriction, DB).

owl_unsatisfied_restriction(Resource, Restriction, DB) :-
	ground(Restriction), ground(Resource), !,
	\+ owl_satisfies_restriction(Resource, Restriction, DB).
owl_unsatisfied_restriction(Resource, Restriction, DB) :-
	ground(Resource),
	bagof(Cls, (
		rdfs_individual_of(Resource, Cls),
		rdfs_individual_of(Cls, owl:'Restriction')
	), Restrictions),
	member(Restriction, Restrictions),
	\+ owl_satisfies_restriction(Resource, Restriction, DB).

%%	owl_satisfies_restriction(?Resource, +Restriction)
%
%	True if Resource satisfies the restriction imposed by Restriction.
%	The current implementation makes the following assumptions:
%
%		* Only one of owl:hasValue, owl:allValuesFrom or owl:someValuesFrom
%		  is present.

owl_satisfies_restriction(Resource, Restriction) :-
	owl_rdf_db(DB),
	owl_satisfies_restriction(Resource, Restriction, DB).
owl_satisfies_restriction(Resource, Restriction, DB) :-
	rdf_has(Restriction, owl:onProperty, Property),
	once( owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) ),
	once( owl_satisfies_cardinality(Resource, Restriction, DB) ), !.
owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:hasValue, Value), !,
	once( owl_has(Resource, Property, Value, DB) ).
owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:allValuesFrom, Class), !,
	once(( bagof(V, owl_has(Resource, Property, V, DB), Vs) ; Vs=[] )),
	all_individual_of(Vs, Class, DB).
owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:someValuesFrom, Class), !,
	once(( owl_has(Resource, Property, Value, DB),
	       owl_individual_of(Value, Class, DB) )).
owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:hasSelf, literal(type(xsd:boolean,true))), !,
	owl_has(Resource, Property, Resource, DB), !.
owl_satisfies_restriction_internal(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:hasSelf, literal(type(xsd:boolean,false))), !,
	\+ owl_has(Resource, Property, Resource, DB).
owl_satisfies_restriction_internal(Resource, _, _, _) :-
	rdf_subject(Resource).

all_individual_of([], _, _).
all_individual_of([H|T], Class, DB) :-
	owl_individual_of(H, Class, DB), !,
	all_individual_of(T, Class, DB).

%	owl_satisfies_cardinality(?Resource[, +Property], +Restriction)
%
%	True if Resource satisfies the cardinality restrictions on
%	Property imposed by Restriction.

owl_satisfies_cardinality(Resource, Restriction) :-
	owl_rdf_db(DB),
	owl_satisfies_cardinality(Resource, Restriction, DB).

owl_satisfies_cardinality(Resource, Restriction, DB) :-
	rdf_has(Restriction, owl:onProperty, Property),
	owl_satisfies_cardinality(Resource, Property, Restriction, DB).

owl_satisfies_cardinality(Resource, Property, Restriction, DB) :-
	once(( rdf_has(Restriction, owl:cardinality, literal(Atom)) ;
	       rdf_has(Restriction, owl:qualifiedCardinality, literal(Atom)) )), !,
	non_negative_int(Atom, Card),
	once(( rdf_has(Restriction, owl:onClass, Cls) ;
	       Cls='http://www.w3.org/2002/07/owl#Thing' )),
	owl_cardinality(Resource, Property, Cls, Card, DB).
owl_satisfies_cardinality(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:minCardinality, literal(MinAtom)),
	non_negative_int(MinAtom, Min), !,
	once(( rdf_has(Restriction, owl:onClass, Cls) ;
	       Cls='http://www.w3.org/2002/07/owl#Thing' )),
	owl_cardinality(Resource, Property, Cls, Count, DB),
	Count >= Min,
	(   rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
	    atom_number(MaxAtom, Max)
	->  Count =< Max
	;   true
	).
owl_satisfies_cardinality(Resource, Property, Restriction, DB) :-
	rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
	non_negative_int(MaxAtom, Max), !,
	once(( rdf_has(Restriction, owl:onClass, Cls) ;
	       Cls='http://www.w3.org/2002/07/owl#Thing' )),
	owl_cardinality(Resource, Property, Cls, Count, DB),
	Count =< Max.
owl_satisfies_cardinality(Resource, _, _, _) :-
	rdf_subject(Resource).

non_negative_int(type('http://www.w3.org/2001/XMLSchema#nonNegativeInteger', Atom), Number) :-
	catch(atom_number(Atom, Number), _, fail).
non_negative_int(Atom, Number) :-
	atom(Atom),
	catch(atom_number(Atom, Number), _, fail).

%%	owl_cardinality(+Resource, +Property, +Cls, -Card) is det.
owl_cardinality(Resource, Property, Cls, Card) :-
	owl_rdf_db(DB),
	owl_cardinality(Resource, Property, Cls, Card, DB).
owl_cardinality(Resource, Property, Cls, Card, DB) :-
	once((setof(V, (
		owl_has(Resource, Property, V, DB), % need to use owl_has here for property chains
		once(owl_individual_of(V,Cls,DB))
	), Vs) ; Vs=[])),
	length(Vs, Card).
%%	owl_cardinality(+Resource, +Property, -Card) is det.
owl_cardinality(Resource, Property, Card) :-
	once((setof(V, owl_has(Resource, Property, V), Vs) ; Vs=[])),
	length(Vs, Card).


		 /*******************************
		 *	    DESCRIPTION		*
		 *******************************/

%%	owl_description(+DescriptionID, -Prolog) is det.
%
%	Convert an owl description into a Prolog representation.  This
%	representation is:
%
%		* class(Class)
%		* restriction(Property, Restriction)
%		* union_of(ListOfDescriptions)
%		* intersection_of(ListOfDescriptions)
%		* complement_of(Description)
%		* one_of(Individuals)
%		* thing
%		* nothing
%
%	where Restriction is defined by owl_restriction_on/2.
%	For example, the union-of can be the result of
%
%	==
%	<rdfs:Class rdf:ID="myclass">
%	  <owl:unionOf parseType=Collection>
%	    <rdf:Description rdf:about="gnu"/>
%	    <rdf:Description rdf:about="gnat"/>
%	  </owl:unionOf>
%	</rdfs:Class>
%	==

owl_description(Descr, Descr) :- compound(Descr), !.
owl_description('http://www.w3.org/2002/07/owl#Thing',   thing)   :- !.
owl_description('http://www.w3.org/2002/07/owl#Nothing', nothing) :- !.
owl_description(ID, Restriction) :-
	(   rdf_has(ID, rdf:type, owl:'Restriction')
	->  owl_restriction(ID, Restriction)
	;   rdf_has(ID, rdf:type, owl:'Class')
	->  (   (   rdf_has(ID, owl:unionOf, Set)
		->  Restriction = union_of(SubDescriptions)
		;   rdf_has(ID, owl:intersectionOf, Set)
		->  Restriction = intersection_of(SubDescriptions)
		)
	    ->	rdfs_list_to_prolog_list(Set, Members),
		maplist(owl_description, Members, SubDescriptions)
	    ;	rdf_has(ID, owl:complementOf, Arg)
	    ->	Restriction = complement_of(SubDescription),
		owl_description(Arg, SubDescription)
	    ;	rdf_has(ID, owl:oneOf, Arg)
	    ->	Restriction = one_of(Individuals),
		rdfs_list_to_prolog_list(Arg, Individuals)
	    ;	Restriction = class(ID)
	    )
	).


%%	owl_description_recursice(+DescriptionID, -Prolog) is det.
%
%	Same as owl_description, but continues for nested descirptions.
%
owl_description_recursive(Resource,Descr) :-
  owl_description(Resource,Resource_x),
  owl_description_recursive_(Resource_x,Descr), !.
owl_description_recursive_(complement_of(Cls), complement_of(Cls_descr)) :-
  owl_description_recursive(Cls, Cls_descr), !.
owl_description_recursive_(restriction(P,some_values_from(Cls)),
                           restriction(P,some_values_from(Cls_descr))) :-
  owl_description_recursive(Cls, Cls_descr), !.
owl_description_recursive_(restriction(P,all_values_from(Cls)),
                           restriction(P,all_values_from(Cls_descr))) :-
  owl_description_recursive(Cls, Cls_descr), !.
owl_description_recursive_(restriction(P,cardinality(Min,Max,Cls)),
                           restriction(P,cardinality(Min,Max,Cls_descr))) :-
  owl_description_recursive(Cls, Cls_descr), !.
owl_description_recursive_(Cls, Cls).

%%	owl_description_assert(+Prolog, -Resource) is det.
%
owl_description_assert(X, Y) :-
  owl_description_assert(X, Y, user).
owl_description_assert(Cls, Cls, _Graph) :- atom(Cls), !.
owl_description_assert(class(Cls), Cls, _Graph) :- !.
owl_description_assert(thing, 'http://www.w3.org/2002/07/owl#Thing', _Graph) :- !.
owl_description_assert(nothing, 'http://www.w3.org/2002/07/owl#Nothing', _Graph) :- !.
owl_description_assert(restriction(P,Facet), Id, Graph) :-
  owl_restriction_assert(restriction(P,Facet), Id, Graph), !.
owl_description_assert(union_of(List), Id, Graph) :-
  owl_assert_description('http://www.w3.org/2002/07/owl#Class', Id, Graph),
  owl_description_list_assert(List,ListId, Graph),
  rdf_assert(Id, owl:unionOf, ListId, Graph), !.
owl_description_assert(intersection_of(List), Id, Graph) :-
  owl_assert_description('http://www.w3.org/2002/07/owl#Class', Id, Graph),
  owl_description_list_assert(List,ListId, Graph),
  rdf_assert(Id, owl:intersectionOf, ListId, Graph), !.
owl_description_assert(complement_of(Cls), Id, Graph) :-
  owl_assert_description('http://www.w3.org/2002/07/owl#Class', Id, Graph),
  owl_description_assert(Cls,ClsId, Graph),
  rdf_assert(Id, owl:complementOf, ClsId, Graph), !.
owl_description_assert(one_of(List), Id, Graph) :-
  owl_assert_description('http://www.w3.org/2002/07/owl#Class', Id, Graph),
  owl_description_list_assert(List,ListId, Graph),
  rdf_assert(Id, owl:oneOf, ListId, Graph), !.

owl_description_list_assert(X, Y) :-
  owl_description_list_assert(X,Y,user).
owl_description_list_assert([], 'http://www.w3.org/1999/02/22-rdf-syntax-ns#nil', _Graph) :- !.
owl_description_list_assert(List, ListId, Graph) :-
  owl_assert_description('http://www.w3.org/1999/02/22-rdf-syntax-ns#List', ListId, Graph),
  owl_description_list_assert_(ListId, List, Graph).

owl_description_list_assert_(Id, [First|Rest], Graph) :-
  owl_description_assert(First, FirstId, Graph),
  owl_description_list_assert(Rest, RestId, Graph),
  rdf_assert(Id, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#first', FirstId, Graph),
  rdf_assert(Id, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#rest', RestId, Graph).

rdf_assert_literal(S,P,V) :-
  rdf_assert_literal(S,P,V,user).
rdf_assert_literal(S,P,V,Graph) :-
  once(( (atom(V), V_atom=V) ; atom_number(V_atom,V) )),
  rdf_assert(S, P, literal(V_atom),Graph).

		 /*******************************
		 *	   OWL_SATISFIES	*
		 *******************************/

%%	owl_satisfies(+Specification, ?Resource) is nondet.
%
%	Test whether Resource satisfies Specification. All resources are
%	considered to belong  to  rdfs:Resource,   which  is  not really
%	enforced. Domain is one of
%
%	| rdfs:Resource		   | Allow for any resource	  |
%	| class(Class)		   | Allow for a subclass of Class|
%	| union_of(Domains)	   |				  |
%	| intersection_of(Domains) |				  |
%	| complement_of(Domain)	   |				  |
%	| one_of(Resources)	   | One of these values	  |
%	| all_values_from(Class)   | Individual of this class	  |
%	| some_values_from(Class)  | Not used			  |
%	| has_value(Value)	   | Must have this value	  |
%
%	Resource can be a term individual_of(Class),  in which case this
%	predicate succeeds if any individual  of   Class  is accepted by
%	Domain.

					% Short-cut
owl_satisfies(Domain, Resource) :-
	rdf_equal(rdfs:'Resource', Domain), !,
	(   atom(Resource)
	->  true
	;   var(Resource)
	->  rdf_subject(Resource)
	;   Resource = individual_of(_)
	).
					% Descriptions
owl_satisfies(class(Domain), Resource) :- !,
	(   rdf_equal(Domain, rdfs:'Resource')
	->  true
	;   Resource = individual_of(Class),
	    atom(Class)
	->  fail
	;   owl_subclass_of(Resource, Domain)
	).
owl_satisfies(union_of(Domains), Resource) :- !,
	member(Domain, Domains),
	owl_satisfies(Domain, Resource).
owl_satisfies(intersection_of(Domains), Resource) :- !,
	in_all_domains(Domains, Resource).
owl_satisfies(complement_of(Domain), Resource) :- !,
	(   atom(Resource)
	->  true
	;   var(Resource)
	->  rdf_subject(Resource)
	;   fail			% individual_of(Class)
	),
	\+ owl_satisfies(Domain, Resource).
owl_satisfies(one_of(List), Resource) :- !,
	member(Resource, List).
					% Restrictions
owl_satisfies(all_values_from(Domain), Resource) :- !,
	(   Resource = individual_of(Class),
	    atom(Class)
	->  owl_subclass_of(Class, Domain)
	;   owl_individual_of(Resource, Domain)
	).
owl_satisfies(some_values_from(_Domain), _Resource) :- !.
owl_satisfies(has_value(Value), Resource) :-
	rdf_equal(Value, Resource).	% TBD: equality


in_all_domains([], _).
in_all_domains([H|T], Resource) :-
	owl_satisfies(H, Resource),
	in_all_domains(T, Resource).


		 /*******************************
		 *	   INDIVIDUAL OF	*
		 *******************************/

%%	owl_individual_of(?Resource, +Description) is nondet.
%
%	Test  or  generate  the  resources    that  satisfy  Description
%	according the the OWL-Description entailment rules.

owl_individual_of(Resource, Description) :-
	owl_rdf_db(DB),
	owl_individual_of(Resource, Description, DB).
owl_individual_of(Resource, Description, DB) :-
	ground([Resource,Description]) ->
		once(owl_individual_of_internal(Resource, Description, DB));
		owl_individual_of_internal(Resource, Description, DB).

owl_individual_of_internal(Resource, Thing, _) :-
	ground(Thing),
	rdf_equal(owl:'Thing', Thing), !,
	rdf_subject(Resource).
owl_individual_of_internal(_Resource, Nothing, _) :-
	ground(Nothing),
	rdf_equal(owl:'Nothing', Nothing),
	fail, !. % MT 16032011
owl_individual_of_internal(Resource, Class, DB) :-
	nonvar(Class), % MT 03122014 -- does not allow generic classification of instances any more, but avoids search through all equivalents of all classes whenever Class is unbound
	rdfs_individual_of(Class, owl:'Class'),
	(   rdf_has(Class, owl:equivalentClass, EQ)
	->  owl_individual_of(Resource, EQ, DB)
	;   rdfs_individual_of(Class, owl:'Restriction')
	->  owl_satisfies_restriction(Resource, Class, DB)
	;   owl_individual_of_description(Resource, Class, DB),
	    findall(SC, rdf_has(Class, rdfs:subClassOf, SC), SuperClasses),
	    owl_individual_of_all(SuperClasses, Resource, DB)
	).
owl_individual_of_internal(Resource, Description, DB) :-
	owl_db_individual_of(DB, Resource, Description).
owl_individual_of_internal(Resource, Description, _) :-
	owl_individual_from_range(Resource, Description).


%%	owl_individual_of_description(?Resource, +Description) is nondet.
%
% 	@tbd	Can a description have multiple of these facets?

owl_individual_of_description(Resource, Description) :-
	owl_rdf_db(DB),
	owl_individual_of_description(Resource, Description, DB).
owl_individual_of_description(Resource, Description, DB) :-
	(   rdf_has(Description, owl:unionOf, Set)
	->  rdfs_member(Sub, Set),
	    owl_individual_of(Resource, Sub, DB)
	;   rdf_has(Description, owl:intersectionOf, Set)
	->  intersection_of(Set, Resource, DB)
	;   rdf_has(Description, owl:complementOf, Arg)
	->  rdf_subject(Resource),
	    \+ owl_individual_of(Resource, Arg, DB)
	;   rdf_has(Description, owl:oneOf, Arg)
	->  rdfs_member(Resource, Arg)
	;   fail			% not an OWL description % MT: changed to 'fail' -> TODO: check if this makes problems if the super-class *is* a restriction
	).


owl_individual_of_all(T, Resource) :-
	owl_rdf_db(DB),
	owl_individual_of_all(T, Resource, DB).
owl_individual_of_all([], _, _).
owl_individual_of_all([C|T], Resource, DB) :-
	owl_individual_of(Resource, C, DB),
	owl_individual_of_all(T, Resource, DB).


owl_individual_from_range(Resource, Class) :-
	nonvar(Resource), !,
	rdf_has(_, P, Resource), atom(P), % DB: P could be inverse_of(..)
	rdf_has(P, rdfs:range, Class), !.
owl_individual_from_range(Resource, Class) :-
	rdf_has(P, rdfs:range, Class),
	rdf_has(_, P, Resource).	% owl_has?

intersection_of(List, Resource, DB) :-
	rdf_has(List, rdf:first, First),
	owl_individual_of(Resource, First, DB),
	(   rdf_has(List, rdf:rest, Rest)
	->  intersection_of(Rest, Resource, DB)
	;   true
	).
intersection_of(Nil, _, _) :-
	rdf_equal(rdf:nil, Nil).

		 /*******************************
		 *	  OWL PROPERTIES	*
		 *******************************/

%%	owl_has(?Subject, ?Predicate, ?Object)
%
%	True if this relation is specified or can be deduced using OWL
%	inference rules.  It adds transitivity to owl_has_direct/3.

owl_has(S, P, O) :-
	owl_rdf_db(DB),
	owl_has_transitive(S, P, O, DB).

owl_has(S, P, O, DB) :-
	(   var(P)
	->  rdf_current_predicate(P)
	;   true
	),
	owl_has_transitive(S, P, O, DB).


%%	owl_has_transitive(?Subject, ?Predicate, ?Object)
%
%	If Predicate is transitive, do a transitive closure on the
%	relation.

owl_has_transitive(S, P, O) :-
	owl_rdf_db(DB),
	owl_has_transitive_(S, P, O, DB, [O]).

owl_has_transitive(S, P, O, DB) :-
	owl_has_transitive_(S, P, O, DB, [O]).

owl_has_transitive_(S, P, O, DB, _) :-
	owl_has_equivalent(S, P, O, DB).

owl_has_transitive_(S, P, O, DB, Visited) :-
	rdfs_individual_of(P, owl:'TransitiveProperty'),
	rdf_reachable(SP, rdfs:subPropertyOf, P),
	owl_has_equivalent(S, SP, O1, DB),          % MT: pulled the rdfs_subprop_of in here to allow transitive sup-property chains
	atom(O1),                       %     of the form P -> SP1 -> SP2 -> P ->... with SP1, SP2 transitive sub-properties of P
	\+ memberchk(O1, Visited),
	owl_has_transitive_(O1, P, O, DB, [O1|Visited]).

%	owl_has_equivalent(?Subject, ?Predicate, ?Object)
%
%	Adds owl:sameAs on Subject and Object to owl_has_direct/3

owl_has_equivalent(S, P, O) :-
	owl_rdf_db(DB),
	owl_has_equivalent(S, P, O, DB).

owl_has_equivalent(S, P, O, DB) :-
	nonvar(O),
	nonvar(S), !,
	owl_same_as(S, S1),
	owl_same_as(O, O1),
	owl_has_direct(S1, P, O1, DB).

owl_has_equivalent(S, P, O, DB) :-
	nonvar(S), !,
	owl_same_as(S, S1),
	owl_has_direct(S1, P, O0, DB),
	owl_same_as(O0, O).

owl_has_equivalent(S, P, O, DB) :-
	nonvar(O), !,
	owl_same_as(O1, O),
	owl_has_direct(S0, P, O1, DB),
	owl_same_as(S0, S).

%%	owl_same_as(?X, ?Y) is nondet.
%
%	True if X and Y are  identical   or  connected by the owl:sameAs
%	relation. Considers owl:sameAs transitive and symetric.

owl_same_as(X,Y) :-
  strip_data_value(X,X_),
  strip_data_value(Y,Y_),!,
  owl_same_as_(X_,Y_).

owl_same_as_(X, Y) :-
	nonvar(X), !,
	owl_same_as(X, Y, [X]).
owl_same_as_(X, Y) :-
	owl_same_as(Y, X, [X]).

owl_same_as(X, X, _).
owl_same_as(X, Y, Visited) :-
	atom(X),
	(   rdf_has(X, owl:sameAs, X1)
	;   rdf_has(X1, owl:sameAs, X)
	),
	X1 \= literal(_),
	\+ memberchk(X1, Visited),
	owl_same_as(X1, Y, [X1|Visited]).

% FIXME redundant
strip_data_value(Value, Value) :- var(Value), !.
strip_data_value(literal(type(_, Value)), Value) :- !.
strip_data_value(literal(Value), Value) :- !.
strip_data_value(Value, Value).


%%	owl_has_direct(?Subject, ?Predicate, ?Object)
%
%	Deals  with  `One-step'  OWL  inferencing:  inverse  properties,
%	symmetric properties and being subtype of  a restriction with an
%	owl:hasValue statement on this property.
%
%%	TODO: add support for Equivalent properties and mixes of sub properties/Equivalent ones

owl_has_direct(S, P, O) :-
	owl_rdf_db(DB),
	owl_satisfies_restriction(S, P, O, DB).

owl_has_direct(S, P, O, DB) :-
	owl_has_direct_internal(S, P, O, DB).

owl_has_direct(S, P, O, DB) :-
	rdfs_individual_of(P, owl:'SymmetricProperty'),
	owl_has_direct_internal(O, P, S, DB).

owl_has_direct(S, P, O, DB) :-
	(  rdf_has(P, owl:inverseOf, P2)
	-> true
	;  rdf_has(P2, owl:inverseOf, P)
	),
	(  owl_has_direct_internal(O, P2, S, DB) ; (
	   rdfs_individual_of(P2, owl:'SymmetricProperty'),
	   owl_has_direct_internal(S, P2, O, DB)
	)).

%% Simplest branch: find an explicitly stored rdf triple (S, P, O)
owl_has_direct_internal(S, P, O, DB) :-
	owl_db_has(DB, S, P, O).

%% If P is bound to an object property, see if any of its PropertyChain axioms is able to produce explicitly known triples.
%% ASSUMPTION: no circular PropertyChain axioms (example, P defined as A o B and A defined as P o B)
owl_has_direct_internal(S, P, O, DB) :-
	rdf_has(P, owl:propertyChainAxiom, RDFList),
	rdfs_list_to_prolog_list(RDFList, PropChain),
	owl_has_property_chain(S, PropChain, O, DB).

owl_has_direct_internal(S, P, O, _) :-
	owl_use_has_value(S, P, O).

%% owl_has_property_chain
owl_has_property_chain(S, PropChain, O, DB) :-
	nonvar(S), !,
	owl_has_property_chain_S2O(S, PropChain, O, DB).
owl_has_property_chain(S, PropChain, O, DB) :-
	reverse(PropChain, PropChainRev),
	owl_has_property_chain_O2S(O, PropChainRev, S, DB).

owl_has_property_chain_S2O(O, [], O, _).
owl_has_property_chain_S2O(S, [P|RestChain], O, DB) :-
	owl_has(S, P, Oi, DB),
	owl_has_property_chain_S2O(Oi, RestChain, O, DB).

owl_has_property_chain_O2S(S, [], S, _).
owl_has_property_chain_O2S(O, [P|RestChain], S, DB) :-
	owl_has(Si, P, O, DB),
	owl_has_property_chain_O2S(Si, RestChain, S, DB).

%% owl_use_has_value
owl_use_has_value(S, P, O) :-
	ground(S), !,
	owl_has_direct_type(S,Type), % this won't allow to find has-values of inferred types, except of intersection classes
	( ground(P) -> (
	  rdf(Type, owl:onProperty, P_sub),
	  rdf(Type, owl:hasValue, O),
	  once(rdfs_subproperty_of(P_sub, P))) ; (
	  rdf(Type, owl:onProperty, P),
	  rdf(Type, owl:hasValue, O))
	).
owl_use_has_value(S, P, O) :-
	ground(O), !,
	rdf(Type, owl:hasValue, O),
	( ground(P) -> (
	  rdf(Type, owl:onProperty, P_sub),
	  once(rdfs_subproperty_of(P_sub, P))) ;
	  rdf_has(Type, owl:onProperty, P)
	),
	rdfs_individual_of(S,Type).

owl_has_direct_type(S,Type) :-
	findall(X, (
		rdfs_individual_of(S,Class),
		owl_has_direct_type_(S,Class,X)
	), Types),
	list_to_set(Types,Set),
	member(Type,Set).

owl_has_direct_type_(S,Class,Type) :-
	rdf_has(Class, owl:intersectionOf, List) ->
	% if it is an intersection class
	( rdfs_member(R, List),
	  rdfs_subclass_of(R, Class2),
	  owl_has_direct_type_(S,Class2,Type) ) ;
	% else
	( Type = Class ).

		 /*******************************
		 *	   DB ACCESS	*
		 *******************************/

owl_rdf_db(db(rdf_has,rdfs_individual_of)).

owl_db_has(db(Has,_), S, P, O) :-
	prepend_arguments(Has, [S,P,O], Goal),
	call(Goal).

owl_db_individual_of(db(_,IndividualOf), S, Description) :-
	prepend_arguments(IndividualOf, [S,Description], Goal),
	call(Goal).

prepend_arguments(Goal, NewArgs, GoalWithArgs) :-
  Goal=..[Head|Args],
  append(NewArgs, Args, Y),
  GoalWithArgs=..[Head|Y].

		 /*******************************
		 *     OWL CLASS HIERARCHY	*
		 *******************************/

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
TBD: It is here that we must use a DL classifier!
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%%	owl_direct_subclass_of(-SubClass, +Class) is nondet.
%%	owl_direct_subclass_of(+SubClass, -Class) is nondet.
%
%	Returns both the RDFS subclasses and  subclass relations implied by
%	owl:intersectionOf and owl:unionOf descriptions.
%
%	@tbd	Should use full DL reasoning

owl_direct_subclass_of(Class, R) :-
	rdf_has(Class, rdfs:subClassOf, R).
owl_direct_subclass_of(Class, R) :-	% added BJW (hack for symetry)
	rdf_has(R, owl:equivalentClass, Class).
owl_direct_subclass_of(Class, R) :-
	(   nonvar(R)
	->  (   rdf_has(R, owl:unionOf, Union),
	        rdfs_member(Class, Union)
	    ;   rdf_has(List, rdf:first, R),
		list_head(List, Head),
		rdf_has(Class, owl:intersectionOf, Head)
	    )
	;   nonvar(Class)
	->  (   rdf_has(Class, owl:intersectionOf, List),
	        rdfs_member(R, List)
	    ;   rdf_has(List, rdf:first, Class),
	        list_head(List, Head),
		rdf_has(R, owl:unionOf, Head)
	    )
	;   throw(error(instantiation_error, _))
	).

list_head(List, Head) :-
	(   rdf_has(H, rdf:rest, List)
	->  list_head(H, Head)
	;   Head = List
	).


%%	owl_subclass_of(+Sub, -Super) is nondet.
%%	owl_subclass_of(-Sub, +Super) is nondet.
%
%	Transitive version of owl_direct_subclass_of/2.

owl_subclass_of(Class, 'http://www.w3.org/2002/07/owl#Thing') :-
	nonvar(Class),
	rdfs_individual_of(Class, owl:'Class').
owl_subclass_of(Class, Super) :-
	rdf_equal(rdfs:'Resource', Resource),
	Super == Resource, !,
	(   nonvar(Class)
	->  true
	;   rdfs_individual_of(Class, owl:'Class')
	).
owl_subclass_of(Class, Super) :-
	nonvar(Class), nonvar(Super), !,
	owl_test_subclass(Class, Super).
owl_subclass_of(Class, Super) :-
	nonvar(Class), !,
	owl_gen_supers(Class, [], Super).
owl_subclass_of(Class, Super) :-
	nonvar(Super), !,
	owl_gen_subs(Super, [], Class).
owl_subclass_of(_, _) :-
	throw(error(instantiation_error, _)).

owl_terminal_subclass_of(Class, Terminal) :-
	rdf_has(Sub, rdfs:'subClassOf', Class) *->
		owl_terminal_subclass_of(Sub, Terminal) ; Terminal=Class.

%%	owl_subproperty_of(+Sub, -Super) is nondet.
%
owl_subproperty_of(Sub,Super) :-
	rdfs_subproperty_of(Sub,Super).
owl_subproperty_of(Sub,Super) :-
	rdf_has(Sub, owl:inverseOf, Sub_inv),
	rdf_has(Super, owl:inverseOf, Super_inv),
	rdfs_subproperty_of(Sub_inv,Super_inv).

%%	owl_most_specific(+Types, -Specific) is semidet.
%
owl_most_specific(Types, Specific) :-
	member(Specific, Types),
	%member(Specific, ['http://www.w3.org/2002/07/owl#Thing'|Types]),
	forall(( % ensure there is no class in Types that is more specific then Cls
		member(Cls_other, Types),
		Specific \= Cls_other
	), \+ owl_subclass_of(Cls_other, Specific)).

%%	owl_most_specific_predicate(+Predicates, -P) is semidet.
%
owl_most_specific_predicate(Predicates, P) :-
	member(P, Predicates),
	forall(( % ensure there is no class in Types that is more specific then Cls
		member(P_other, Predicates),
		\+ rdf_equal(P, P_other)
	), \+ rdfs_subproperty_of(P_other, P)).

%%	owl_common_ancestor(+Types, Common) is semidet.
%
owl_common_ancestor(Types, Common) :-
	member(Cls_a, Types),
	bagof(X, (
		rdfs_subclass_of(Cls_a, X),
		\+ rdfs_individual_of(X, owl:'Restriction'),
		forall( member(Cls_b, Types), rdfs_subclass_of(Cls_b, X) )
	), CommonTypes),
	owl_most_specific(CommonTypes, Common).

owl_gen_supers(Class, _, Class).
owl_gen_supers(Class, Visited, Super) :-
	(   owl_direct_subclass_of(Class, Super0)
	*-> true
	;   rdf_equal(Super0, rdfs:'Resource')
	),
	\+ memberchk(Super0, Visited),
	owl_gen_supers(Super0, [Super0|Visited], Super).

owl_gen_subs(Class, _, Class).
owl_gen_subs(Class, Visited, Sub) :-
	owl_direct_subclass_of(Sub0, Class),
	\+ memberchk(Sub0, Visited),
	owl_gen_subs(Sub0, [Sub0|Visited], Sub).


%%	owl_test_subclass(+Class, +Super) is semidet.
%
%	Cached check for OWL subclass relation.

:- dynamic
	subclass_cache/3,		% +C1, +C2, -Boolean
	subclass_generation/1.		% RDF generation of last compute

owl_test_subclass(Class, Super) :-
	(   rdf_generation(G),
	    subclass_generation(G2),
	    G \== G2
	->  retractall(subclass_cache(_,_,_))
	;   true
	),
	(   subclass_cache(Class, Super, Bool)
	->  Bool = true
	;   (   owl_gen_supers(Class, [], Super)
	    ->	assert(subclass_cache(Class, Super, true))
	    ;	assert(subclass_cache(Class, Super, false)),
		fail
	    )
	).

%% owl_inverse_property(?P, ?P_inv)
%
owl_inverse_property(P, P_inv) :-
	( rdf_has(P, owl:inverseOf, P_inv) ;
	  rdf_has(P_inv, owl:inverseOf, P) ), !.
owl_inverse_property(P, P_inv) :-
	%owl_assert_description('http://www.w3.org/2002/07/owl#Description', P_inv),
	atomic_list_concat([P,'_inv'],P_inv),
	( rdf_has(P,rdfs:domain,X) -> rdf_assert(P_inv,rdfs:range,X)  ; true ),
	( rdf_has(P,rdfs:range,Y)  -> rdf_assert(P_inv,rdfs:domain,Y) ; true ),
	rdf_assert(P_inv, owl:inverseOf, P),
	rdf_assert(P_inv, rdf:type, owl:'ObjectProperty').

%% owl_inverse_property_chain(?P, ?P_inv)
%
owl_inverse_property_chain(PropChain, PropChain_inv) :-
	reverse(PropChain, PropChain_reversed),
	owl_inverse_property_chain_(PropChain_reversed,PropChain_inv).
owl_inverse_property_chain_([], []) :- !.
owl_inverse_property_chain_([P|Rest],[P_inv|Rest_inv]) :-
	owl_inverse_property(P, P_inv),
	owl_inverse_property_chain_(Rest,Rest_inv).

%% owl_inverse_functional(?P)
%
owl_inverse_functional(P) :-
  rdfs_subproperty_of(P, Super),
  rdfs_individual_of(Super, owl:'InverseFunctionalProperty'), !.

%% owl_disjoint_with(?Class1, ?Class2)
%
%  MT: Tests if Class1 and Class2 are disjoint, taking both individual disjointWith
%      properties and the OWL2 AllDisjointClasses into account
%

owl_disjoint_with(A, A) :-
    fail,!.

% direct assertions (OWL1):
owl_disjoint_with(A, B) :-
    owl_subclass_of(A, Asuper),
    owl_subclass_of(B, Bsuper),
    owl_has(Asuper, owl:disjointWith, Bsuper).

owl_disjoint_with(A, B) :-
    owl_subclass_of(A, Asuper),
    owl_subclass_of(B, Bsuper),
    owl_has(Bsuper, owl:disjointWith, Asuper).

% OWL2 AllDisjointClasses list:
owl_disjoint_with(A, B) :-
    A\=B,
    owl_individual_of(DisjointClasses, owl:'AllDisjointClasses'),
    owl_has(DisjointClasses, owl:members, DisjointClassList),
    rdfs_member(Asuper, DisjointClassList), owl_subclass_of(A, Asuper),
    rdfs_member(Bsuper, DisjointClassList), owl_subclass_of(B, Bsuper).


		 /*******************************
		 *     SEARCH IN HIERARCHY	*
		 *******************************/

%%	owl_find(+String, +Domain, ?Properties, +Method, -Subject) is nondet.
%
%	Search all classes below Domain for a literal property with
%	that matches String.  Method is one of
%
%		* substring
%		* word
%		* prefix
%		* exact
%
%	domain is defined by owl_satisfies/2 from owl.pl
%
%	Note that the rdfs:label field is handled by rdfs_label/2,
%	making the URI-ref fragment name the last resort to determine
%	the label.
%
%	@tbd	Use the RDF literal primitives

owl_find(String, Domain, Fields, Method, Subject) :-
	var(Fields), !,
	For =.. [Method,String],
	rdf_has(Subject, Field, literal(For, _)),
	owl_satisfies(Domain, Subject),
	Fields = [Field].		% report where we found it.
owl_find(String, Domain, Fields, Method, Subject) :-
	globalise_list(Fields, GlobalFields),
	For =.. [Method,String],
	member(Field, GlobalFields),
	(   Field == resource
	->  rdf_subject(Subject),
	    rdf_match_label(Method, String, Subject)
	;   rdf_has(Subject, Field, literal(For, _))
	),
	owl_satisfies(Domain, Subject).

globalise_list([], []) :- !.
globalise_list([H0|T0], [H|T]) :- !,
	globalise_list(H0, H),
	globalise_list(T0, T).
globalise_list(X, G) :-
	rdf_global_id(X, G).
