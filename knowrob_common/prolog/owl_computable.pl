/*  $Id$

    Extensions to merge OWL reasoning with computables.

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

:- module(owl_computable,
	  [ 
	    owl_triple/3,			% ?Subject, ?Predicate, ?Object
	    owl_triple_direct/3		% ?Subject, ?Predicate, ?Object
	  ]).

:- use_module(library(lists)).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('semweb/owl')).



		 /*******************************
		 *	    EXPANSION		*
		 *******************************/

%	user:goal_expansion(+NSGoal, -Goal)
%
%	This predicate allows for writing down rdf queries in a friendly
%	name-space fashion.

:- multifile
	user:goal_expansion/2.

:- rdf_meta
  owl_restriction_on(r, t),
  owl_merged_restriction(r, r, t),
  owl_restriction(r, -),
  owl_description(r, -),
  owl_cardinality_on_subject(r, r, -),
  owl_cardinality_on_class(r, r, -),
  owl_satisfies(r, t),
  owl_instance_of(r, t),
  owl_direct_subclass_of(r, r),
  owl_subclass_of(r, r),
  owl_has(r, r, o),
  owl_has_direct(r, r, o),
  owl_same_as(r, r).





%%	owl_satisfies_restriction(?Resource, +Restriction)
%
%	True if Restriction satisfies the restriction imposed by Restriction.
%	The current implementation makes the following assumptions:
%
%		* Only one of owl:hasValue, owl:allValuesFrom or owl:someValuesFrom
%		  is present.

owl_satisfies_restriction(Resource, Restriction) :-
	rdf_has(Restriction, owl:onProperty, Property),
	(   rdf_has(Restriction, owl:hasValue, Value)
	->  owl_triple(Resource, Property, Value)
	;   rdf_has(Restriction, owl:allValuesFrom, Class)
	->  setof(V, owl_triple(Resource, Property, V), Vs),
	    all_individual_of(Vs, Class)
	;   rdf_has(Restriction, owl:someValuesFrom, Class)
	->  owl_triple(Resource, Property, Value),
	    owl_instance_of(Value, Class)
	;   rdf_subject(Resource)
	),
	owl_satisfies_cardinality(Resource, Restriction).

all_individual_of([], _).
all_individual_of([H|T], Class) :-
	owl_instance_of(H, Class), !,
	all_individual_of(T, Class).

%	owl_satisfies_cardinality(?Resource[, +Property], +Restriction)
%
%	True if Resource satisfies the cardinality restrictions on
%	Property imposed by Restriction.

owl_satisfies_cardinality(Resource, Restriction) :-
	rdf_has(Restriction, owl:onProperty, Property),
	owl_satisfies_cardinality(Resource, Property, Restriction).

owl_satisfies_cardinality(Resource, Property, Restriction) :-
	rdf_has(Restriction, owl:cardinality, literal(Atom)), !,
	non_negative_int(Atom, Card),
	findall(V, owl_triple(Resource, Property, V), Vs0),
	sort(Vs0, Vs),			% remove duplicates
	length(Vs, Card).
owl_satisfies_cardinality(Resource, Property, Restriction) :-
	rdf_has(Restriction, owl:minCardinality, literal(MinAtom)),
	non_negative_int(MinAtom, Min), !,
	findall(V, owl_triple(Resource, Property, V), Vs0),
	sort(Vs0, Vs),			% remove duplicates
	length(Vs, Count),
	Count >= Min,
	(   rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
	    atom_number(MaxAtom, Max)
	->  Count =< Max
	;   true
	).
owl_satisfies_cardinality(Resource, Property, Restriction) :-
	rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
	non_negative_int(MaxAtom, Max), !,
	findall(V, owl_triple(Resource, Property, V), Vs0),
	sort(Vs0, Vs),			% remove duplicates
	length(Vs, Count),
	Count =< Max.
owl_satisfies_cardinality(Resource, _, _) :-
	rdf_subject(Resource).

non_negative_int(type(Type, Atom), Number) :-
	rdf_equal(xsd:nonNegativeInteger, Type),
	catch(atom_number(Atom, Number), _, fail).
non_negative_int(Atom, Number) :-
	atom(Atom),
	catch(atom_number(Atom, Number), _, fail).



     /*******************************
     *     INDIVIDUAL OF  *
     *******************************/

%%  owl_instance_of(?Resource, +Description) is nondet.
%
% Test  or  generate  the  resources    that  satisfy  Description
% according the the OWL-Description entailment rules.

owl_instance_of(Resource, Thing) :-
  rdf_equal(Thing, owl:'Thing'), %!, MT 16032011
  (   atom(Resource)
  ->  true
  ;   rdf_subject(Resource)
  ).
owl_instance_of(_Resource, Nothing) :-
  rdf_equal(Nothing, owl:'Nothing'), %!, MT 16032011
  fail.
owl_instance_of(Resource, Description) :-     % RDFS
  rdfs_instance_of(Resource, Description).
owl_instance_of(Resource, Class) :-
  nonvar(Resource),
  setof(C, rdf_triple(rdf:type, Resource, C), Cs), %!, MT 16032011
  member(C, Cs),
  owl_subclass_of(C, Class).
owl_instance_of(Resource, Class) :-
  rdfs_individual_of(Class, owl:'Class'),
  (   rdf_has(Class, owl:equivalentClass, EQ)
  ->  owl_instance_of(Resource, EQ)
  ;   rdfs_individual_of(Class, owl:'Restriction')
  ->  owl_satisfies_restriction(Resource, Class)
  ;   owl_instance_of_description(Resource, Class),
      findall(SC, rdf_has(Class, rdfs:subClassOf, SC), SuperClasses),
      owl_instance_of_all(SuperClasses, Resource)
  ).
owl_instance_of(Resource, Description) :-     % RDFS
  owl_instance_from_range(Resource, Description).


%%  owl_instance_of_description(?Resource, +Description) is nondet.
%
%   @tbd  Can a description have multiple of these facets?

owl_instance_of_description(Resource, Description) :-
  (   rdf_has(Description, owl:unionOf, Set)
  ->  rdfs_member(Sub, Set),
      owl_instance_of(Resource, Sub)
  ;   rdf_has(Description, owl:intersectionOf, Set)
  ->  intersection_of(Set, Resource)
  ;   rdf_has(Description, owl:complementOf, Arg)
  ->  rdf_subject(Resource),
      \+ owl_instance_of(Resource, Arg)
  ;   rdf_has(Description, owl:oneOf, Arg)
  ->  rdfs_member(Resource, Arg)
  ;   fail      % not an OWL description % MT: changed to 'fail' -> TODO: check if this makes problems if the super-class *is* a restriction
  ).


owl_instance_of_all([], _).
owl_instance_of_all([C|T], Resource) :-
  owl_instance_of(Resource, C),
  owl_instance_of_all(T, Resource).


owl_instance_from_range(Resource, Class) :-
  nonvar(Resource), !,
  rdf_has(_, P, Resource),
  rdf_has(P, rdfs:range, Class), !.
owl_instance_from_range(Resource, Class) :-
  rdf_has(P, rdfs:range, Class),
  rdf_has(_, P, Resource).  % owl_has?

intersection_of(List, Resource) :-
  rdf_has(List, rdf:first, First),
  owl_instance_of(Resource, First),
  (   rdf_has(List, rdf:rest, Rest)
  ->  intersection_of(Rest, Resource)
  ;   true
  ).
intersection_of(Nil, _) :-
  rdf_equal(rdf:nil, Nil).
  
owl_use_has_value(S, P, O) :-
	nonvar(P), !,
	rdf_has(Super, owl:onProperty, P),
	rdf_has(Super, owl:hasValue, O),
	owl_direct_subclass_of(Type, Super),
	rdf_has(S, rdf:type, Type).
owl_use_has_value(S, P, O) :-
	rdf_has(S, rdf:type, Type),
	owl_direct_subclass_of(Type, Super),
	rdfs_individual_of(Super, owl:'Restriction'),
	rdf_has(Super, owl:onProperty, P),
	rdf_has(Super, owl:hasValue, O).


		 /*******************************
		 *	  OWL PROPERTIES	*
		 *******************************/

%%	owl_has(?Subject, ?Predicate, ?Object)
%
%	True if this relation is specified or can be deduced using OWL
%	inference rules.  It adds transitivity to owl_has_direct/3.

owl_triple(S, P, O) :-
	(   var(P)
	->  rdf_current_predicate(P)
	;   true
	),
% 	rdf_reachable(SP, rdfs:subPropertyOf, P),
% 	owl_triple_transitive(S, SP, O).
  owl_triple_transitive(S, P, O).


%%	owl_triple_transitive(?Subject, ?Predicate, ?Object)
%
%	If Predicate is transitive, do a transitive closure on the
%	relation.

owl_triple_transitive(S, P, O) :-
	rdfs_individual_of(P, owl:'TransitiveProperty'), !,
	owl_triple_transitive(S, P, O, [P]).
owl_triple_transitive(S, P, O) :-
	owl_has_equivalent(S, P, O).

owl_triple_transitive(S, P, O, Visited) :-
  rdf_reachable(SP, rdfs:subPropertyOf, P),
	owl_has_equivalent(S, SP, O1),          % MT: pulled the rdfs_subprop_of in here to allow transitive sup-property chains
	O1 \= literal(_),                       %     of the form P -> SP1 -> SP2 -> P ->... with SP1, SP2 transitive sub-properties of P
	\+ memberchk(O1, Visited),
	(   O = O1
	;   owl_triple_transitive(O1, P, O, [O1|Visited])
	).




% owl_has_equivalent(?Subject, ?Predicate, ?Object)
%
% Adds owl:sameAs on Subject and Object to owl_triple_direct/3

owl_has_equivalent(S, P, O) :-
  nonvar(S), !,
  owl_same_as(S, S1),
  owl_triple_direct(S1, P, O0),
  owl_same_as(O0, O).
owl_has_equivalent(S, P, O) :-
  nonvar(O), !,
  owl_same_as(O1, O),
  owl_triple_direct(S0, P, O1),
  owl_same_as(S0, S).
owl_has_equivalent(S, P, O) :-
  owl_triple_direct(S0, P, O0),
  owl_same_as(S0, S),
  owl_same_as(O0, O).





%%	owl_triple_direct(?Subject, ?Predicate, ?Object)
%
%	Deals  with  `One-step'  OWL  inferencing:  inverse  properties,
%	symmetric properties and being subtype of  a restriction with an
%	owl:hasValue statement on this property.
%
%	@bug	owl_triple_direct/3 also uses SWRL rules.  This should be
%		moved elsewhere.

owl_triple_direct(S, P, O) :-
	rdf_triple(P, S, O).
owl_triple_direct(S, P, O) :-
	(   rdf_has(P, owl:inverseOf, P2)
	->  true
	;   rdf_has(P2, owl:inverseOf, P)
	),
	rdf_triple(P2, O, S).		% TBD: must call owl_triple_direct/3
owl_triple_direct(S, P, O) :-
	rdfs_individual_of(P, owl:'SymmetricProperty'),
	rdf_triple(P, O, S).
owl_triple_direct(S, P, O) :-
	owl_use_has_value(S, P, O).





