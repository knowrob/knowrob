/*
  Copyright (C) 2016 Daniel Beßler
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

:- module(entities,
    [
      entity/2,
      entity_has/3,
      entity_assert/2,
      entity_retract/1,
      entity_write/1,
      with_owl_description/3
    ]).
/** <module> Reasoning via partial descriptions of entities.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/computable')).

:- rdf_meta entity(r,?),
            entity_property(+,?,?),
            entity_assert(r,?),
            entity_retract(r),
            with_owl_description(r,r,t).

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% methods for querying OWL entities

%% entity(?Entity, +Descr) is nondet
%
% Query for entity matching an entity description or build
% description for entity.
%
% @param Entity IRI of matching entity
% @param Descr An entity description, OWL individual or class IRI
%
entity(Entity, EntityClass) :-
  atom(EntityClass),
  owl_individual_of(Entity, EntityClass), !.

entity(Entity, Descr) :-
  var(Descr), !,
  rdfs_individual_of(Entity,owl:'Thing'),
  once(entity_head(Entity, [A,TypeBase], _, TypeIri)),
  entity_generate(Entity, [A,TypeBase], TypeIri, Descr).

entity(Entity, Descr) :-
  entity_(Entity, Descr),
  % make sure it's an individual and not a class
  once(owl_individual_of(Entity, owl:'Thing')),
  \+ rdfs_individual_of(Entity, knowrob:'TemporalPart').

%% Time point entities
entity_(Entity, [a, timepoint, Time]) :-
  number(Time), 
  owl_instance_from_class(knowrob:'TimePoint', [instant=Time], Entity), !.
entity_(Entity, [a, timepoint, [value,TimeValue]]) :-
  nonvar(TimeValue),
  owl_instance_from_class(knowrob:'TimePoint', [instant=TimeValue], Entity), !.
entity_(Entity, [a, timepoint, [name,Name]]) :-
  nonvar(Name),
  rdf_global_term(Name, Entity),
  time_term(Entity, TimeValue),
  owl_instance_from_class(knowrob:'TimePoint', [instant=TimeValue], Entity), !.

%% Time interval entities
entity_(Entity, [an, interval, [Begin, End]]) :-
  owl_instance_from_class(knowrob:'TimeInterval', [begin=Begin,end=End], Entity), !.
entity_(Entity, [an, interval, [Begin]]) :-
  owl_instance_from_class(knowrob:'TimeInterval', [begin=Begin], Entity), !.
entity_(Entity, [an, interval|Descr]) :-
  entity_has(Descr, start_time, BeginDescr),
  entity(Begin, BeginDescr),
  (( entity_has(Descr, end_time, EndDescr),
     entity(End, EndDescr),
     owl_instance_from_class(knowrob:'TimeInterval', [begin=Begin,end=End], Entity) );
     owl_instance_from_class(knowrob:'TimeInterval', [begin=Begin], Entity) ), !.

%% Pose entities
entity_(Entity, [a, pose, Pos, Rot]) :-
  owl_instance_from_class(knowrob:'Pose', [pose=(Pos,Rot)], Entity), !.
entity_(Entity, [a|[pose|Descr]]) :-
  entity_has(Descr, translation, Pos),
  entity_has(Descr, quaternion, Rot),
  (  entity_has(Descr, reference_frame, Frame)
  -> owl_instance_from_class(knowrob:'Pose', [pose=(Frame,Pos,Rot)], Entity)
  ;  owl_instance_from_class(knowrob:'Pose', [pose=(Pos,Rot)], Entity) ), !.

%% Location entities
entity_(Entity, [a, location|Descr]) :-
  entity_axioms(Descr, 'http://knowrob.org/kb/knowrob.owl#spatiallyRelated', Axioms),
  length(Axioms, L), (L > 0),
  owl_instance_from_class(knowrob:'SpaceRegion', [axioms=Axioms], Entity), !.

%% Entity type description `a|an pose|location|...`
entity_(Entity, [a, Type|Descr]) :-
  nonvar(Type), !,
  entity_name(Descr, Entity),
  entity_body(Entity, [a, Type|Descr]).
entity_(Entity, [an, Type|Descr]) :-
  nonvar(Type), !,
  entity_name(Descr, Entity),
  entity_body(Entity, [an, Type|Descr]).

%% name of the OWL individual
entity_(Entity, [[name,EntityName]|Descr]) :-
  entity_name([name,EntityName], Entity),
  entity_(Entity, Descr).

entity_(Entity, [[type,restriction(P,Facet2)|_]|Descr]) :-
  nonvar(P), nonvar(Facet2), !,
  rdf_global_term(P,P_glob),
  % find restrictions on P
  owl_restriction_on(Entity, Q_glob, Restr),
  rdfs_subproperty_of(P_glob, Q_glob),
  % unify the facets
  owl_description(Restr, restriction(_,Facet1)),
  match_facet(Facet1,Facet2),
  entity_(Entity, Descr).

%% ignore type, types are handled in `entity_head`
entity_(Entity, [[type,Type|_]|Descr]) :-
  nonvar(Type), !,
  entity_(Entity, Descr).

%% key-value property
% TODO: support specification of property units
%    - [an, object, [height, 20, qudt:meter]]
%    - [an, object, [height, 20, qudt:meter, during, Interval]]
entity_(Entity, [[Key,Value|ValDescr]|Descr]) :-
  nonvar(Key),
  entity_iri(PropIri, Key, lower_camelcase),
  
  % use temporal axioms to restrict Interval var before calling holds
  (nonvar(ValDescr)
  -> (
    entity_interval_axioms(ValDescr, Axioms),
    entity_intersection_interval(Axioms, RestrictedInterval),
    (RestrictedInterval=[] -> true ; Interval=RestrictedInterval)
  ) ; true),
  
  (nonvar(Value)
  -> (
    rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
    -> (
      strip_literal_type(Value, X),
      holds(Entity,PropIri,PropValue,Interval),
      % ignore literal(type(..)) terms, just extract value
      strip_literal_type(PropValue, X)
    ) ; (
      entity_object_value(PropValue, Value),
      holds(Entity,PropIri,PropValue,Interval)
    )
  ) ; (
    holds(Entity,PropIri,PropValue,Interval), (
    rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
    -> strip_literal_type(PropValue, Value)
    ;  Value = PropValue)
  )),
  
  entity_(Entity, Descr).

entity_(_, []).


%% Nested object description
entity_object_value(Iri, Value) :-
  is_list(Value), !,
  entity(Iri, Value).
%% Object iri
entity_object_value(Iri, Value) :-
  once( var(Iri) ; atom(Iri) ),
  entity_ns(Value, NS, ValueUnderscore),
  atom(NS), atom(ValueUnderscore),
  camelcase(ValueUnderscore, ValueCamel),
  atom_concat(NS, ValueCamel, Iri), !.
%% Object iri
entity_object_value(Iri, Value) :-
  rdf_global_term(Value, Iri), !.


%% Handle description
entity_body(Entity, [A, Type|Descr]) :-
  entity_head(Entity, [A,Type], Descr, _),
  entity_(Entity, Descr),
  % check general type last because it matches many entities
  entity_type([A,Type], TypeIri),
  once(owl_individual_of(Entity, TypeIri)).


%% "name" keys in the description
entity_name(Descr, Entity) :-
  entity_has(Descr,name,Name),
  !, % names must match!
  rdf_global_term(Name, Entity),
  (   rdf_has(Entity, _, _)
  -> true
  ;  entity_iri(Entity, Name, camelcase) ), !.
entity_name(_, _).


%% entity_has(+Descr, +Key, Value).
% Read entity description value
entity_has([[Key,Val|_]|_], Key, Val).
entity_has([_|Tail], Key, Val) :- entity_has(Tail, Key, Val).


%% entity_type(?Descr, ?Iri) is det.
%
% Maps ectity type description to IRI
%
% @param Descr The type description "[a,an] ?type_name"
% @param Iri The corresponding type iri
%
entity_type([a,timepoint],  'http://knowrob.org/kb/knowrob.owl#TimePoint').
entity_type([an,interval],  'http://knowrob.org/kb/knowrob.owl#TimeInterval').
entity_type([an,action],    'http://knowrob.org/kb/knowrob.owl#Action').
entity_type([an,event],     'http://knowrob.org/kb/knowrob.owl#Event').
entity_type([an,object],    'http://knowrob.org/kb/knowrob.owl#EnduringThing-Localized').
entity_type([a,location],   'http://knowrob.org/kb/knowrob.owl#SpaceRegion').
entity_type([a,pose],       'http://knowrob.org/kb/knowrob.owl#Pose').
entity_type([a,trajectory], 'http://knowrob.org/kb/knowrob.owl#Trajectory').

entity_type(Entity, TypeBase, Entity) :-
  atom(Entity), rdf_reachable(Entity, rdfs:subClassOf, TypeBase), !.

entity_type(Entity, TypeBase, Type) :-
  rdf_has(Entity, rdf:type, Type),
  rdf_reachable(Type, rdfs:subClassOf, TypeBase).


%% entity_assert(-Entity, +Descr).
%
% Assert entity description in RDF triple store as new individual.
%
% @param Entity IRI of matching entity
% @param Descr An entity description
%
entity_assert(Entity, [a,timepoint|Descr]) :- entity(Entity, [a,timepoint|Descr]), !. 
entity_assert(Entity, [an,interval|Descr]) :- entity(Entity, [an,interval|Descr]), !.
entity_assert(Entity, [a,location|Descr]) :-  entity(Entity, [a,location|Descr]), !.

entity_assert(Entity, [a, pose, Pos, Rot]) :-
  owl_instance_from_class(knowrob:'Pose', [pose=(Pos,Rot)], Entity), !.
entity_assert(Entity, [a, pose|Descr]) :-
  entity_has(Descr, translation, Pos),
  entity_has(Descr, quaternion, Rot),
  (  entity_has(Descr, relative_to, RelObjDescr)
  -> (
    entity(RelObj, RelObjDescr),
    owl_instance_from_class(knowrob:'Pose', [pose=(RelObj,Pos,Rot)], Entity)
  ) ; (
    owl_instance_from_class(knowrob:'Pose', [pose=(Pos,Rot)], Entity) )
  ), !.

entity_assert(Entity, [A,Type|Descr]) :-
  nonvar(Type),
  entity_type([A,Type], TypeIri_),
  (( entity_has(Descr,type,AType),
     entity_iri(ATypeIri, AType, camelcase),
     rdf_reachable(ATypeIri, rdfs:subClassOf, TypeIri_) )
  -> TypeIri = ATypeIri
  ;  TypeIri = TypeIri_ ),
  (( entity_has(Descr, name, Entity_),
     rdf_global_term(Entity_, Entity),
     rdf_assert(Entity, rdf:type, TypeIri) );
     rdf_instance_from_class(TypeIri, Entity) ),
  entity_assert(Entity, Descr), !.

entity_assert(Entity, [[name,_]|Descr]) :- entity_assert(Entity, Descr), !.

entity_assert(Entity, [[type,TypeDescr]|Descr]) :-
  nonvar(Entity), nonvar(TypeDescr),
  entity_iri(TypeIri, TypeDescr, camelcase),
  rdf_assert(Entity, rdf:type, TypeIri),
  entity_assert(Entity, Descr), !.

entity_assert(Entity, [[Key,Value]|Descr]) :-
  nonvar(Entity), nonvar(Key), nonvar(Value),
  entity_iri(PropIri, Key, lower_camelcase),
  (  rdf_has(PropIri, rdf:type, owl:'ObjectProperty')
  ->  ( % nested entity
      % TODO: support recursive asserting (i.e., call enity_assert instead)
      entity(ValueEntity, Value),
      rdf_assert(Entity, PropIri, ValueEntity)
  ) ; ( % data property
      rdf_has(PropIri, rdf:type, owl:'DatatypeProperty'),
      ( rdf_phas(PropIri, rdfs:range, Range) ->
        rdf_assert(Entity, PropIri, literal(type(Range,Value))) ;
        rdf_assert(Entity, PropIri, literal(Value)) )
  )),
  entity_assert(Entity, Descr).

entity_assert(Entity, [[Key,Value,during,IntervalDescr]|Descr]) :-
  nonvar(Entity), nonvar(Key), nonvar(Value),
  entity_iri(PropIri, Key, lower_camelcase),
  entity(Interval, IntervalDescr),
  (  rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
  ->  ( % data property
      rdf_phas(PropIri, rdfs:range, Range) ->
        assert_temporal_part(Entity, PropIri, literal(type(Range,Value)), Interval) ;
        assert_temporal_part(Entity, PropIri, literal(Value), Interval)
  ) ; ( % nested entity
      rdf_has(PropIri, rdf:type, owl:'ObjectProperty'),
      entity(ValueEntity, Value),
      assert_temporal_part(Entity, PropIri, ValueEntity, Interval)
  )),
  entity_assert(Entity, Descr).

entity_assert(_, []).

%% entity_retract(+Entity).
%
% Assert entity description in RDF triple store as new individual.
%
% @param Entity IRI of matching entity
%
entity_retract(Entity) :-
  % Retract without recursion
  forall( owl_has(Entity, knowrob:temporalParts, TemporalPart),
          rdf_retractall(TemporalPart, _, _)),
  rdf_retractall(Entity, _, _).


%% with_owl_description(+Description, ?Individual, +Goal).
%
% Ensures entity description is asserted and binds the name to
% Individual before goal is called.
% Temporary assertions are retracted in a cleanup goal.
%
% @param Description Entity description or individual
% @param Individual Entity individual
% Goal The goal with OWL entity asserted
%
with_owl_description(Description, Individual, Goal) :-
  atom(Description)
  -> (
    Individual = Description,
    call( Goal )
  ) ; (
    is_list(Description),
    setup_call_cleanup(
      entity_assert(Individual, Description),
      call( Goal ),
      entity_retract(Individual)
    )
  ).

%% entity_write(+Descr) is nondet.
%
% Write entity to current output stream.
%
% @param Descr An entity description
%
entity_write(Descr) :- entity_write(Descr, '').

entity_write([A,Type|Tail], Spaces) :-
  member(A, [a,an]),
  writef('[%w, %w,\n', [A,Type]),
  atom_concat(Spaces, '  ', SpacesNext),
  entity_write(Tail, SpacesNext),
  write(']\n').

entity_write([[Key,Val|ValDescr]|Tail], Spaces) :-
  (is_list(Val)
  -> (
    Val=[A,Type|ValTail],
    writef('%w[%w, [%w, %w\n', [Spaces,Key,A,Type]),
    atom_concat(Spaces, '  ', SpacesNext),
    entity_write(ValTail, SpacesNext),
    writef('%w]],\n', [Spaces])
  ) ; (
    writef('%w%w,\n', [Spaces,[Key,Val|ValDescr]])
  )),
  entity_write(Tail, Spaces).

entity_write([Key|Tail], Spaces) :-
  writef('%w, %w\n', [Spaces,Key]),
  entity_write(Tail, Spaces).

entity_write([], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% helper


interval_operator(during, interval_during)   :- !.
interval_operator(before, interval_before)   :- !.
interval_operator(after,  interval_after)    :- !.

entity_interval_axioms([Operator,Value|Axioms], [[Operator,Interval_val]|Tail]) :-
  interval_operator(Operator, _), !,
  entity(Interval, Value),
  interval(Interval, Interval_val),
  entity_interval_axioms(Axioms, Tail).
entity_interval_axioms([_|Axioms], Tail) :-
  entity_interval_axioms(Axioms, Tail).
entity_interval_axioms([], []).

entity_intersection_interval([[during,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_intersect(I1, I_Tail, Intersection).
entity_intersection_interval([[after,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_end(I1, End),
  interval_intersect([End], I_Tail, Intersection).
entity_intersection_interval([[before,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_start(I1, Begin),
  interval_intersect([0.0,Begin], I_Tail, Intersection).
entity_intersection_interval([], []).

interval_intersect([], I, I).
interval_intersect(I, [], I).
interval_intersect([Begin0,End0], [Begin1,End1], [Begin,End]) :-
  Begin is max(Begin0, Begin1),
  End is min(End0, End1).
interval_intersect([Begin0], [Begin1,End], [Begin,End]) :-
  Begin is max(Begin0, Begin1).
interval_intersect([Begin0,End], [Begin1], [Begin,End]) :-
  Begin is max(Begin0, Begin1).


%% Fluent properties that match temporal relation.
entity_temporally_holds(_, []).
entity_temporally_holds(FluentInterval, [TemporalRelation,IntervalDescr|Tail]) :-
  (  interval_operator(TemporalRelation,Operator)
  -> (
    interval(IntervalDescr, IntervalDescr_val),
    call(Operator, FluentInterval, IntervalDescr_val)
  ) ; true ),
  entity_temporally_holds(FluentInterval, [IntervalDescr|Tail]).


entity_properties([['http://www.w3.org/1999/02/22-rdf-syntax-ns#type',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,_]|Tail], DescrTail) :-
  rdf_has(PropIri, rdf:type, owl:'AnnotationProperty'),
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#comment',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#subClassOf',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://knowrob.org/kb/knowrob.owl#temporalParts',Fluent]|Tail], Descr) :-
  findall([Key,Value,during,[an,interval,IntervalDescr]], (
    temporal_part_has(Fluent, PropIri, PropValue, IntervalDescr),
    ( rdf_equal(PropIri, rdf:type)
    -> (
      Key=type,
      entity_iri(PropValue, Value, camelcase)
    ) ; (
      entity_properties([[PropIri,PropValue]], [[Key,Value]])
    ))
  ), FluentDescr),
  entity_properties(Tail, DescrTail),
  append(FluentDescr, DescrTail, Descr), !.

entity_properties([[inverse_of(_),_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,TimeIri]|Tail],
                  [[Key, [a,timepoint,Time]]|DescrTail]) :-
  once((nonvar(TimeIri);nonvar(Time))),
  rdfs_individual_of(TimeIri, knowrob:'TimePoint'),
  entity_iri(PropIri, Key, lower_camelcase),
  time_term(TimeIri,Time),
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,IntervalIri]|Tail],
                  [[Key, [an,interval,Interval]]|DescrTail]) :-
  once((nonvar(IntervalIri);nonvar(Interval))),
  interval(IntervalIri,Interval),
  entity_iri(PropIri, Key, lower_camelcase),
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,PropValue]|Tail], [[Key,Value]|DescrTail]) :-
  entity_iri(PropIri, Key, lower_camelcase),
  % match rdf value with description value
  (  rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
  -> (var(Value) -> strip_literal_type(PropValue, Value) ; rdf_global_term(Value, PropValue))
  ;  entity(PropValue, Value)
  ),
  entity_properties(Tail, DescrTail).

entity_properties([], []).


entity_axioms([[P_descr,O_desc|_]|Descr], AxiomIri, [[P,O]|Axioms]) :-
  entity_iri(P, P_descr, lower_camelcase),
  rdfs_subproperty_of(P, AxiomIri),
  entity(O, O_desc),
  entity_axioms(Descr, AxiomIri, Axioms).

entity_axioms([], _, []).


entity_generate(Entity, [a,timepoint], _, [a,timepoint,Time]) :-
  rdfs_individual_of(Entity, knowrob:'TimePoint'),
  time_term(Entity,Time), !.

entity_generate(Entity, [an,interval], _, [an,interval,Interval]) :-
  rdfs_individual_of(Entity, knowrob:'TemporalThing'),
  interval(Entity,Interval), !.

entity_generate(Pose, [a, pose], _, [a, pose, [X,Y,Z], [QW,QX,QY,QZ]]) :-
  rdf_has_prolog(Pose, knowrob:translation, [X,Y,Z]),
  rdf_has_prolog(Pose, knowrob:quaternion, [QW,QX,QY,QZ]), !.

entity_generate(Entity, [A,TypeBase], TypeBaseIri, [A,TypeBase|[[type,TypeName]|PropDescr]]) :-
  (( rdf_has(Entity, rdf:type, Type),
     Type \= TypeBaseIri,
     rdf_reachable(Type, rdfs:subClassOf, TypeBaseIri) )
  -> TypeIri = Type
  ;  TypeIri = TypeBaseIri ),
  entity_iri(TypeIri, TypeName, camelcase),
  findall([PropIri,PropValue], rdf(Entity, PropIri, PropValue), Props),
  entity_properties(Props, PropDescr), !.


%% Match [a|an, ?entity_type]
entity_head(Entity, _, Descr, TypeIri) :-
  var(Entity),
  
  findall(TypeIri, (
    entity_has(Descr, type, TypeDescr),
    \+ TypeDescr = restriction(_,_),
    once(
    entity_iri(TypeIri, TypeDescr, camelcase) ;
    rdf_global_term(TypeDescr, TypeIri) )
  ), Types),
  
  ( Types=[]
  -> true
  ; (
    findall(E, (
      owl_individual_of_all(Types, E),
      \+ owl_individual_of(E, knowrob:'TemporalPart')
    ), Entities),
    % avoid redundant results of owl_individual_of
    list_to_set(Entities, EntitiesUnique),
    member(Entity, EntitiesUnique)
  )).

entity_head(Entity, [A,Type], _, TypeIri) :-
  nonvar(Entity),
  findall([A,Type,TypeIri], (
    ( entity_type([A,Type], TypeIri), rdfs_individual_of(Entity, TypeIri) );
    [A,Type,TypeIri]=[a,thing,'http://www.w3.org/2002/07/owl#Thing']
  ), Types),
  member([A,Type,TypeIri], Types),
  % pick only the most special types
  forall( member([_,BType,BTypeIri], Types), (
    BType = Type ; (
      not(rdf_reachable(BTypeIri, rdfs:subClassOf, TypeIri))
    )
  )).


%% Read entity namespace
entity_ns(Entity, NamespaceUri, EntityName) :-
  (  Entity =.. [':', NS, EntityName]
  ->  (
      rdf_current_ns(NS, NamespaceUri)
  ) ; ( % fallback to knowrob namespace
      EntityName = Entity,
      rdf_current_ns(knowrob, NamespaceUri)
  )), !.

entity_ns(Entity, NamespaceUri, EntityName) :-
  atom(Entity),
  rdf_split_url(NamespaceUri, EntityName, Entity).


%% Converts between IRI representation and description representation
entity_iri(Iri, Descr, Formatter) :-
  var(Descr),
  rdf_split_url(NamespaceUri, Name, Iri),
  rdf_current_ns(Namespace, NamespaceUri),
  call(Formatter, NameUnderscore, Name),
  (  Namespace=knowrob
  -> Descr=NameUnderscore
  ;  Descr=Namespace:NameUnderscore
  ).

entity_iri(Iri, Description, Formatter) :-
  var(Iri),
  entity_ns(Description, NS, NameUnderscore),
  call(Formatter, NameUnderscore, Name),
  atom_concat(NS, Name, Iri).

entity_iri(Entity, Type) :-
  rdf_has(Entity, rdf:type, Type),
  rdf_split_url(Url, _, Type),
  rdf_current_ns(NS, Url),
  NS \= owl.

entity_iri(Entity, Type) :-
  rdfs_individual_of(Entity,Type).

match_facet(some_values_from(A1), some_values_from(A2)) :-
  rdf_global_term(A1,A1_glob),
  rdf_global_term(A2,A2_glob),
  rdfs_subclass_of(A1_glob,A2_glob), !.
match_facet(all_values_from(A1), all_values_from(A2)) :-
  rdf_global_term(A1,A1_glob),
  rdf_global_term(A2,A2_glob),
  rdfs_subclass_of(A1_glob,A2_glob), !.
