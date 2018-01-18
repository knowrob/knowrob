/*
  Copyright (C) 2008-10 Bernhard Kirchlechner, Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(knowrob_rdfs,
    [
      rdf_instance_from_class/2,
      rdf_instance_from_class/3,
      rdf_unique_id/2,
      rdf_phas/3,
      rdf_has_prolog/3,
      rdf_assert_prolog/3,
      rdf_assert_prolog/4,
      rdf_vector_prolog/2,
      rdfs_value_prolog/3,
      rdfs_type_of/2,
      rdfs_common_ancestor/2,
      strip_literal_type/2
    ]).
/** <module> Reasoning using procedural attachments, called "computables"

@author Bernhard Kirchlechner, Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_meta rdf_phas(r,r,o),
            rdf_has_prolog(r,r,t),
            rdfs_value_prolog(r,t,?),
            rdf_instance_from_class(r,r),
            rdf_instance_from_class(r,r,r),
            rdfs_type_of(r,r),
            rdfs_common_ancestor(t,r).

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

		 /*******************************
		 *		  ASSERTIONS			*
		 *******************************/

%% rdf_instance_from_class(+Class, -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers
%
% @param Class   Class describing the type of the instance
% @param Inst    Identifier of the generated instance of Class
%
rdf_instance_from_class(Class, Instance) :-
    rdf_instance_from_class(Class, _, Instance).

%% rdf_instance_from_class(+Class, +SourceRef, -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers using
% the source reference SourceRef for rdf_assert
%
% @param Class     Class describing the type of the instance
% @param SourceRef Atom as source reference for rdf_assert
% @param Inst      Identifier of the generated instance of Class
%
rdf_instance_from_class(Class, SourceRef, Instance) :-
  ( is_uri(Class) ->
    T=Class ;
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Class, T)),
  rdf_unique_id(Class, Instance),
  ( var(SourceRef) ->
    rdf_assert(Instance, rdf:type, T) ;
    rdf_assert(Instance, rdf:type, T, SourceRef)).

%% rdf_unique_id(+Class, -UniqID) is det.
%
% UniqID is a IRI that uses Class as prefix and is
% not yet used in the RDF triple store.
%
% @param Class Class IRI
% @param UniqID Unused IRI with prefix Class
%
rdf_unique_id(Class, UniqID) :-
  % generate 8 random alphabetic characters
  randseq(8, 25, Seq_random),
  maplist(plus(65), Seq_random, Alpha_random),
  atom_codes(Sub, Alpha_random),
  atom_concat(Class,  '_', Class2),
  atom_concat(Class2, Sub, Instance),
  % check if there is no triple with this identifier as subject or object yet
  ((rdf(Instance,_,_);rdf(_,_,Instance)) ->
    (rdf_unique_id(Class, UniqID));
    (UniqID = Instance)).

		 /*******************************
		 *		  Properties				*
		 *******************************/

%% rdf_phas(+Property, ?P, ?O) is nondet.
%
% True if Property or one of its super properties
% has a property assertion for P with value O.
%
% @param Property A RDF property
% @param P A RDF property
% @param O A RDF resource
%
rdf_phas(Property, P, O) :-
	rdfs_subproperty_of(Property, Super),
	rdf_has(Super, P, O).

%% rdf_has_prolog(?Subject,?Predicate,?Value).
%
% Calls rdf_has and converts data values to
% Prolog representation (e.g., xsd:float to Prolog number).
%
% @param Subject RDF resource iri
% @param Predicate RDF predicate iri
% @param Value RDF iri or Prolog encoded data value
%
rdf_has_prolog(S,P,O) :-
  rdf_has(S,P,O_rdf),
  rdfs_value_prolog(P,O_rdf,O).

%% rdf_assert_prolog(?Subject,?Predicate,?Value).
%% rdf_assert_prolog(?Subject,?Predicate,?Value,+Graph).
%
% Asserts a triple to the RDF datastore,
% and accepts some Prolog encoded data values.
%
% @param Subject RDF resource iri
% @param Predicate RDF predicate iri
% @param Value RDF iri or Prolog encoded data value
%
rdf_assert_prolog(S,P,O) :- rdf_assert_prolog(S,P,O,user).
rdf_assert_prolog(S,P,O,Graph) :-
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  strip_literal_type(O,Value),
  (  rdf_phas(P, rdfs:range, Range)
  -> rdf_assert(S, P, literal(type(Range,Value)), Graph)
  ;  rdf_assert(S, P, literal(Value), Graph)
  ).
rdf_assert_prolog(S,P,O,Graph) :-
  rdf_assert(S,P,O,Graph).
  

%% rdfs_value_prolog(+Property, +RDFValue, -PrologValue).
%
% Converts RDF value to native Prolog representation.
%
% @tbd convert to standard SI unit
%
rdfs_value_prolog('http://knowrob.org/kb/knowrob.owl#boundingBoxSize', literal(type(_,Val)), Vec) :- % TODO: make subproperty of knowrob:vector
  rdf_vector_prolog(Val, Vec), !.
rdfs_value_prolog('http://knowrob.org/kb/knowrob.owl#mainColorOfObject', literal(type(_,Val)), Vec) :- % TODO: make subproperty of knowrob:vector
  rdf_vector_prolog(Val, Vec), !.
rdfs_value_prolog(P, literal(type(_,Val)), Vec) :-
  rdfs_subproperty_of(P, knowrob:vector),
  rdf_vector_prolog(Val, Vec), !.
rdfs_value_prolog(_, literal(type('http://www.w3.org/2001/XMLSchema#float',Atom)), Number) :-
  atom_number(Atom, Number), !.
rdfs_value_prolog(_, literal(type('http://www.w3.org/2001/XMLSchema#double',Atom)), Number) :-
  atom_number(Atom, Number), !.
rdfs_value_prolog(_, literal(type('http://www.w3.org/2001/XMLSchema#integer',Atom)), Number) :-
  atom_number(Atom, Number), !.
rdfs_value_prolog(_, literal(type(_,Atom)), Atom) :- !.
rdfs_value_prolog(_, literal(Atom), Atom) :- !.
rdfs_value_prolog(_, V, V).

%% strip_literal_type(+Input,-Output).
%
% Strip the literal(type(..., Value)) and return value if present, else return the original.
%
strip_literal_type(literal(type(_, Value)), Value) :- !.
strip_literal_type(literal(Value), Value) :- !.
strip_literal_type(Value, Value).

%% rdf_vector_prolog(+In, -Out) is semidet.
rdf_vector_prolog([X|Y], [X|Y]).
rdf_vector_prolog(In, Numbers) :-
  rdf_vector_prolog(In, Numbers, ' ').
rdf_vector_prolog(In, Numbers, Delimiter) :-
  atom(In),
  normalize_space(atom(In_Normalized),In),
  atomic_list_concat(Atoms, Delimiter, In_Normalized),
  maplist(atom_number, Atoms, Numbers).

		 /*******************************
		 *		  Class hierarchy		*
		 *******************************/

%% rdfs_type_of(?Resource, ?Type) is nondet.
%
% True if Type is one of the most specific types of Resoure.
%
% @param Resource OWL resource with rdf:type statements
% @param Type One of the most specific type statements
%
rdfs_type_of(Resource, Type) :-
  rdf_has(Resource, rdf:type, Type),
  Type \= 'http://www.w3.org/2002/07/owl#NamedIndividual',
  % ensure there is no class in Types that is more specific then Cls
  forall((
     rdf_has(Resource, rdf:type, Cls_other),
     Type \= Cls_other
  ), \+ rdfs_subclass_of(Cls_other, Type)).

%% rdfs_common_ancestor(+Classes, ?Ancestor).
%
% Get one of the most specific common ancestors of rdf_classes.
%
rdfs_common_ancestor([C], C).
rdfs_common_ancestor([C1, C2| Cs], C) :-
  rdf_superclass_list([C1, C2| Cs], SCs),
  intersection_of_sets(SCs, CSCs),
  most_specific_class(CSCs, C0),
  C = C0.

		 /*******************************
		 *	  Utility predicates		*
		 *******************************/

rdf_superclass_list([], []).
rdf_superclass_list([C|CRest], [SCs| SCRest]) :-
  findall(SC, rdfs_subclass_of(C, SC), SCs),
  rdf_superclass_list(CRest, SCRest).

intersection_of_sets([], []).
intersection_of_sets([L], L).
intersection_of_sets([L0, L1|LRest], Intersection) :-
  intersection(L0, L1, L),
  intersection_of_sets([L|LRest], Intersection).

most_specific_class([C], C).
most_specific_class([C1,C2|Cs], C) :-
  rdfs_subclass_of(C2, C1)
  -> most_specific_class([C2|Cs], C)
  ; most_specific_class([C1|Cs], C). % Either not comparable or C2 is superclass of C1

is_uri(X) :-
  concat_atom(List, '#', X),
  length(List, Length),
  Length>1.
