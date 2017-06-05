/** <module> Predicates for temporal reasoning in KnowRob

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

@author Daniel Beßler
@license BSD
*/

:- module(knowrob_temporal,
    [
      owl_individual_of_during/3,
      owl_individual_of_during/2,
      holds/1,
      holds/2,
      holds/3,
      holds/4,
      occurs/1,
      occurs/2,
      occurs/3,
      interval/2,
      interval_start/2,
      interval_end/2,
      interval_before/2,
      interval_after/2,
      interval_meets/2,
      interval_starts/2,
      interval_finishes/2,
      interval_overlaps/2,
      interval_during/2,
      assert_temporal_part/3,
      assert_temporal_part/4,
      assert_temporal_part_end/4,
      assert_temporal_part_end/3,
      current_temporal_part/2,
      temporal_part/3,
      temporal_part_has/3,
      temporal_part_has/4
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta owl_individual_of_during(r,r,?),
            owl_individual_of_during(r,r),
            holds(t),
            holds(t,?),
            holds(r,r,t),
            holds(r,r,t,?),
            occurs(r),
            occurs(r,?),
            occurs(r,?,r),
            interval(?,?),
            interval_start(?,?),
            interval_end(?,?),
            interval_before(?,?),
            interval_after(?,?),
            interval_meets(?,?),
            interval_starts(?,?),
            interval_finishes(?,?),
            interval_overlaps(?,?),
            interval_during(?,?),
            current_temporal_part(r,r),
            assert_temporal_part(r,r,r,r),
            assert_temporal_part(r,r,r),
            assert_temporal_part_end(r,r,r,r),
            assert_temporal_part_end(r,r,r),
            temporal_part(r,r,t),
            temporal_part_has(r,r,r),
            temporal_part_has(r,r,r,t).

% define holds as meta-predicate and allow the definitions
% to be in different source files
:- meta_predicate holds(0, ?, ?, ?, ?).
:- multifile holds/4.


owl_individual_of_during(Resource, Description) :-
  current_time(Now),
  owl_individual_of_during(Resource, Description, Now).
owl_individual_of_during(Resource, Thing, _) :-
  rdf_equal(Thing, owl:'Thing'), %!, MT 16032011
  (   atom(Resource)
  ->  true
  ;   rdf_subject(Resource)
  ).
owl_individual_of_during(_Resource, Nothing, _) :-
  rdf_equal(Nothing, owl:'Nothing'), %!, MT 16032011
  fail.
owl_individual_of_during(Resource, Description, _) :- % RDFS
  rdfs_individual_of(Resource, Description).
owl_individual_of_during(Resource, Class, Interval) :-
  nonvar(Resource), var(Class),
  holds(Resource, rdf:type, Class, Interval).
owl_individual_of_during(Resource, Class, Interval) :-
  nonvar(Resource), nonvar(Class), 
  once(( owl_subclass_of(C, Class),
         holds(Resource, rdf:type, C, Interval) )).
owl_individual_of_during(Resource, Class, Interval) :-
  var(Resource), nonvar(Class), 
  owl_subclass_of(C, Class),
  holds(Resource, rdf:type, C, Interval).
owl_individual_of_during(Resource, Class, Interval) :-
  nonvar(Class), % MT 03122014 -- does not allow generic classification of instances any more, but avoids search through all equivalents of all classes whenever Class is unbound
  rdfs_individual_of(Class, owl:'Class'),
  (   rdf_has(Class, owl:equivalentClass, EQ)
  ->  owl_individual_of_during(Resource, EQ, Interval)
  ;   rdfs_individual_of(Class, owl:'Restriction')
  ->  owl_satisfies_restriction_during(Resource, Class, Interval)
  ;   owl_individual_of_description(Resource, Class),
      findall(SC, rdf_has(Class, rdfs:subClassOf, SC), SuperClasses),
      owl_individual_of_all_during(SuperClasses, Resource, Interval)
  ).
owl_individual_of_during(Resource, Description, _) :- % RDFS
  owl_individual_from_range(Resource, Description).

owl_individual_of_all_during([], _, _).
owl_individual_of_all_during([C|T], Resource, I) :-
	owl_individual_of_during(Resource, C, I),
	owl_individual_of_all_during(T, Resource, I).


owl_satisfies_restriction_during(Resource, Restriction, Interval) :-
  rdf_has(Restriction, owl:onProperty, Property),
  (   rdf_has(Restriction, owl:hasValue, Value)
  ->  holds(Resource, Property, Value, Interval)
  ;   rdf_has(Restriction, owl:allValuesFrom, Class)
  ->  setof(V, holds(Resource, Property, V, Interval), Vs),
      owl_individual_of(Vs, Class)
  ;   rdf_has(Restriction, owl:someValuesFrom, Class)
  ->  holds(Resource, Property, Value, Interval),
      owl_individual_of(Value, Class)
  ;   rdf_subject(Resource)
  ),
  owl_satisfies_cardinality_during(Resource, Restriction, Interval).

owl_satisfies_cardinality_during(Resource, Restriction, Interval) :-
  rdf_has(Restriction, owl:onProperty, Property),
  owl_satisfies_cardinality_during(Resource, Property, Restriction, Interval).
owl_satisfies_cardinality_during(Resource, Property, Restriction, Interval) :-
  rdf_has(Restriction, owl:cardinality, literal(Atom)), !,
  non_negative_int(Atom, Card),
  findall(V, holds(Resource, Property, V, Interval), Vs0),
  sort(Vs0, Vs),			% remove duplicates
  length(Vs, Card).
owl_satisfies_cardinality_during(Resource, Property, Restriction, Interval) :-
  rdf_has(Restriction, owl:minCardinality, literal(MinAtom)),
  non_negative_int(MinAtom, Min), !,
  findall(V, holds(Resource, Property, V, Interval), Vs0),
  sort(Vs0, Vs),			% remove duplicates
  length(Vs, Count),
  Count >= Min,
  (   rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
      atom_number(MaxAtom, Max)
  ->  Count =< Max
  ;   true
  ).
owl_satisfies_cardinality_during(Resource, Property, Restriction, Interval) :-
  rdf_has(Restriction, owl:maxCardinality, literal(MaxAtom)),
  non_negative_int(MaxAtom, Max), !,
  findall(V, holds(Resource, Property, V, Interval), Vs0),
  sort(Vs0, Vs),			% remove duplicates
  length(Vs, Count),
  Count =< Max.
owl_satisfies_cardinality_during(Resource, _, _, _) :-
  rdf_subject(Resource).

%% holds(+Term, ?T)
%% holds(+Term)
%
% True iff @Term holds during @T. Where @T is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Term Must be of the form: "PROPERTY(SUBJECT, OBJECT)".
%             For example: `Term = knowrob:insideOf(example:'DinnerPlate_fdigh245', example:'Drawer_bsdgwe8trg')`.
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers
%          representing a time interval.
%
holds(Term) :-
  % TODO: just call owl_has instead
  current_time(T),
  holds(Term,T).

holds(Term, I) :-
  var(Term),
  holds(S, P, O, I),
  Term =.. [':', P, (S,O)].

holds(Term, I) :-
  nonvar(Term),
  (  Term =.. [':', Namespace, Tail]
  -> (
     Tail =.. [P,S,O],
     % unpack namespace
     rdf_current_ns(Namespace, NamespaceUri),
     atom_concat(NamespaceUri, P, PropertyUri),
     holds(S, PropertyUri, O, I)
  ) ; (
     Term =.. [P,S,O],
     holds(S, P, O, I)
  )), !.

holds(S, P, O) :-
  % TODO: just call owl_has instead
  current_time(T),
  holds(S, P, O, [T,T]).

holds(S, P, O, I) :-
  ( atom(S) ; var(S) ),
  (  atom(P)
  -> rdf_triple(P, S, O)
  ;  owl_has(S,P,O)
% FIXME: inspect does not infer class properties of individuals!
%   e.g., that a fridge instance is storage place for perishable
%  ; inspect(S,P,O)
  ),
  \+ rdfs_individual_of(S, knowrob:'TemporalPart'),
  \+ rdfs_individual_of(O, knowrob:'TemporalPart'),
  (  var(I)
  -> I = [0.0]
  ;  true
  ).
  
holds(S, P, O, I) :-
  ( atom(S) ; var(S) ),
  temporal_part_has(S, P, O, TimeInterval),
  time_term(TimeInterval, Interval),
  (  var(I)
  -> I = Interval
  ;  interval_during(I, Interval)
  ).

/*  
holds(S, P, O, I) :-
  (var(S)
  -> (
    class_properties(S, P, O)%,
    %rdfs_individual_of(S, Cls)
  ) ; (
    % FIXME(daniel): this terribly slows down inference. Tried with comp_spatial tests
    %rdfs_individual_of(S, Cls),
    %class_properties(S, P, O)
    class_properties(S, P, O)
  )),
  (  var(I)
  -> I = [0.0]
  ;  true
  ).*/


%% occurs(?Evt) is nondet.
%% occurs(?Evt,?T) is nondet.
%
% True iff @Evt occurs during @T. Where @T is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Evt Identifier of the event
% @param T   Timepoint or time interval
% 
occurs(Evt) :-
  current_time(T),
  occurs(Evt, T).

% Read event instance from RDF triple store
occurs(Evt, I) :-
  occurs(Evt, I, knowrob:'Event').

occurs(Evt, I, Type) :-
  rdfs_individual_of(Evt, Type),
  interval(Evt, EvtI),
  (  ground(I)
  -> interval_during(I, EvtI)
  ;  I = EvtI ).

%% TODO: make occurs a multifile predicate



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% methods for asserting fluent relations

%% current_temporal_part(?Obj,?TemporalPart) is nondet.
%
% true for any OWL individual `Obj` that has a temporal part `TemporalPart`
% whose relation still holds at the moment.
%
% @param Obj             The object with temporal relations
% @param TemporalPart    A temporal part of the object
% 
current_temporal_part(Obj,TemporalPart) :-
  temporal_part(Obj, TemporalPart, TemporalExtend),
  \+ rdf_has(TemporalExtend, knowrob:endTime, _).

%% temporal_part(?Obj,?TemporalPart,?TemporalExtend) is nondet.
%
% true for any OWL individual `Obj` that has a temporal part `TemporalPart`
% whose temporal extend is described by `TemporalExtend`.
%
% @param Obj             The object with temporal relations
% @param TemporalPart    A temporal part of the object
% @param TemporalExtend  The temporal extend of the temporal part
% 
temporal_part(Obj,TemporalPart,TemporalExtend) :-
  rdf_has(Obj, knowrob:'temporalParts', TemporalPart),
  rdf_has(TemporalPart, knowrob:'temporalExtend', TemporalExtend).

create_temporal_part(S, P, TemporalPart) :-
  rdf_instance_from_class(knowrob:'TemporalPart', TemporalPart),
  rdf_assert(S, knowrob:'temporalParts', TemporalPart),
  rdf_assert(TemporalPart, knowrob:'temporalProperty', P).


%% assert_temporal_part(+S,+P,+O) is nondet.
%% assert_temporal_part(+S,+P,+O,+I) is nondet.
%
% Asserts a temporal relation between `S` and `O` using the relation
% predicate `P` that holds during the time interval `I`. `O` is either an OWL individual,
% a term `nontemporal(Iri)` or a typed data value.
%
% @param S    The subject of the relation
% @param P    The relation predicate
% @param O    The object of the relation or a typed data value
% @param I    The time interval during which the relation holds
% 
assert_temporal_part(S, P, O) :-
  current_time(Now),
  assert_temporal_part(S, P, O, Now).
assert_temporal_part(S, P, O, I) :-
  create_temporal_part(S, P, TemporalPart),
  assert_temporal_part_value(TemporalPart, P, O),
  assert_temporal_part_extend(TemporalPart, I),
  % Also assert nontemporal if temporal part is not finished by now.
  % This allows nontemporal reasoners to work with descriptions generated here.
  ignore((
    current_temporal_part(S,TemporalPart),
    assert_nontemporal_value(S,P,O)
  )).

% TODO: setting data valued property should be part of knowrob_owl
assert_nontemporal_value(S, P, Value) :-
  atomic(Value),
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  (  rdf_phas(P, rdfs:range, Range)
  -> rdf_assert(S, P, literal(type(Range,Value)))
  ;  rdf_assert(S, P, literal(Value))
  ).
assert_nontemporal_value(S, P, nontemporal(Value)) :-
  rdf_assert(S, P, Value), !.
assert_nontemporal_value(S, P, Value) :-
  compound(Value),
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  rdf_assert(S, P, Value).
assert_nontemporal_value(S, P, O) :-
  rdf_assert(S, P, O).

% TODO: setting data valued property should be part of knowrob_owl
assert_temporal_part_value(TemporalPart, P, Value) :-
  atomic(Value),
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  (  rdf_phas(P, rdfs:range, Range)
  -> rdf_assert(TemporalPart, P, literal(type(Range,Value)))
  ;  rdf_assert(TemporalPart, P, literal(Value))
  ).
assert_temporal_part_value(TemporalPart_S, P, nontemporal(Value)) :-
  rdf_assert(TemporalPart_S, P, Value), !.
assert_temporal_part_value(TemporalPart, P, Value) :-
  compound(Value), % FIXME: weak check
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  rdf_assert(TemporalPart, P, Value).
assert_temporal_part_value(TemporalPart_S, P, Value) :-
  create_temporal_part(Value, P, TemporalPart_O), !,
  rdf_assert(TemporalPart_S, P, TemporalPart_O),
  rdf_assert(Value, knowrob:'temporalParts', TemporalPart_O).

% TODO: this findall/forall looks ugly
assert_temporal_part_extend(TemporalPart, I) :-
  create_interval(I, Interval),
  findall(X, (
      rdf_has(TemporalPart,_,X),
      rdfs_individual_of(X, knowrob:'TemporalPart')
  ), Parts),
  forall( member(Part, [TemporalPart|Parts]),
          rdf_assert(Part, knowrob:'temporalExtend', Interval)
  ).
retract_temporal_extend(TemporalPart) :-
  findall(X, (
      rdf_has(TemporalPart,_,X),
      rdfs_individual_of(X, knowrob:'TemporalPart')
  ), Parts),
  forall( member(Part, [TemporalPart|Parts]),
          rdf_retractall(Part, knowrob:'temporalExtend', _)
  ).

%% assert_temporal_part_end(+S,?P,?O) is nondet.
%% assert_temporal_part_end(+S,?P,?O,+End) is nondet.
%
% Assert that a given relation `P(S,O)` stopped holding starting from
% specified time instant `End` (or current time if `End` is not specified).
% The assertion specifies the `endTime` of the temporal extend of the 
% temporal part that describes this relation.
% Additionally any non-temporal representation of this relation will be retracted
% (so that standard reasoners won't yield this relation anymore).
%
% @param S    The subject of the relation
% @param P    The relation predicate
% @param O    The object of the relation or a data-typed value
% @param End  The time instant at which the relation stopped holding
% 
assert_temporal_part_end(S, P, O) :-
  current_time(Now),
  assert_temporal_part_end(S, P, O, Now).
assert_temporal_part_end(S, P, O, End) :-
  atom(End),
  atom_number(End, End_num),
  assert_temporal_part_end(S, P, O, End_num).
assert_temporal_part_end(S, P, O, End) :-
  number(End),
  current_temporal_part(S,S_temporal),
  rdf_has(S_temporal, knowrob:'temporalProperty', P),
  rdf_has(S_temporal, knowrob:'temporalExtend', I),
  temporal_part_value(S_temporal, P, O),
  % update temporal extend
  rdf_has(I, knowrob:'startTime', BeginIri),
  time_term(BeginIri, Begin),
  retract_temporal_extend(S_temporal),
  assert_temporal_part_extend(S_temporal, [Begin,End]),
  % remove the nontemporal property if exists
  ignore(( temporal_part_value(S, P, O),
           rdf_retractall(S, P, O) )).
assert_temporal_part_end(S, P, O, End) :-
  number(End),
  % Make relation temporal if the relation was not described temporally before
  temporal_part_value(S, P, O),
  rdf_retractall(S, P, O),
  assert_temporal_part(S, P, O, [0.0,End]).

%% temporal_part_value(?S, ?P, ?O)
temporal_part_value(S, P, O) :-
  var(O),
  rdf_has(S, P, X),
  (  rdf_has(P, rdf:type, owl:'DatatypeProperty')
  -> strip_literal_type(X, O)
  ;  once(( rdf_has(O,knowrob:'temporalParts',X); O=X ))
  ).
temporal_part_value(S, P, nontemporal(O)) :-
  temporal_part_value(S, P, O), !.
temporal_part_value(S, P, O) :-
  nonvar(O),
  strip_literal_type(O, O_stripped),
  rdf_has(S, P, X),
  (  rdf_has(P, rdf:type, owl:'DatatypeProperty')
  -> strip_literal_type(X, O_stripped)
  ;  once(( rdf_has(O,knowrob:'temporalParts',X); O=X ))
  ).


%% temporal_part_has(?Subject, ?Predicate, ?Object, ?Interval)
%
% True if this relation is specified via knowrob:TemporalPart
% or can be deduced using OWL inference rules.
% 
temporal_part_has(S, P, O) :- temporal_part_has(S, P, O, _). % FIXME: redundant results!

temporal_part_has(S, P, O, I) :-
  rdfs_individual_of(S, knowrob:'SpatialThing-Localized'),
  rdf_has(S, knowrob:'temporalParts', TemporalPart),
  temporal_part_has(TemporalPart, P, O, I).

temporal_part_has(S, P, O, I) :-
  atom(S),
  rdf_has(S, knowrob:'temporalProperty', P),
  
  rdf_has(S, knowrob:'temporalExtend', Ext),
  (var(I)
  -> I=Ext 
  ;  interval_during(I, Ext)),
  
  rdf_has(S, P, S_O),
  
  (rdf_equal(P, rdf:type)
  -> \+ rdf_equal(S_O, owl:'NamedIndividual'),
     \+ rdf_equal(S_O, knowrob:'TemporalPart')
  ;  true),
  
  (rdfs_individual_of(S_O, knowrob:'TemporalPart')
  -> once(owl_has(O, knowrob:temporalParts, S_O))
  ; (
    ( ground(O) -> (
      strip_literal_type(S_O,S_O_stripped),
      strip_literal_type(O,O_stripped),
      O_stripped = S_O_stripped ) ;
    ( O = S_O ))
  )).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Allen's interval algebra

%% interval(I,[ST,ET]) is semidet.
%
% The interval of I
%
% @param I Time point, interval or temporally extended entity
% @param [ST,ET] Start and end time of the interval
% 
interval(I0, I1) :- is_list(I0), !, I1 = I0.
interval(Time, [Time,Time]) :- number(Time), !.
interval(TimePoint, [Time,Time]) :-
  atom(TimePoint),
  rdfs_individual_of(TimePoint, knowrob:'TimePoint'),
  time_term(TimePoint, Time), !.
interval(I, Interval) :-
  atom(I),
  rdf_has(I, knowrob:'temporalExtend', Ext),
  interval(Ext, Interval), !.
interval(I, Interval) :-
  atom(I),
  rdf_has(I, knowrob:'startTime', T0),
  time_term(T0, T0_val),
  (  rdf_has(I, knowrob:'endTime', T1)
  -> (time_term(T1, T1_val), Interval=[T0_val,T1_val])
  ;  Interval=[T0_val] ), !.
interval(I, Interval) :-
  var(I),
  rdfs_individual_of(I, knowrob:'Event'),
  rdf_has(I, knowrob:'startTime', T0),
  time_term(T0, T0_val),
  (  rdf_has(I, knowrob:'endTime', T1)
  -> (time_term(T1, T1_val), Interval=[T0_val,T1_val])
  ;  Interval=[T0_val] ).
interval(I, Interval) :-
  var(I),
  rdfs_individual_of(I, knowrob:'Event'),
  rdf_has(I, knowrob:'temporalExtend', Ext),
  interval(Ext, Interval).

%% interval_start(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_start(I, Start_val) :-
  interval(I, Val),
  once(( Val=[Start] ; Val=[Start,_] )),
  time_term(Start, Start_val).

%% interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_end(I, End_val) :-
  interval(I, [_,End]),
  time_term(End, End_val).

%% interval_before(I0,I1) is semidet.
%
%  Interval I0 takes place before I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_before(I0, I1) :-
  interval_end(I0, End0),
  interval_start(I1, Begin1),
  End0 < Begin1.

%% interval_after(I0,I1) is semidet.
%
% Interval I0 takes place after I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_after(I0, I1) :-
  interval_start(I0, Begin0),
  interval_end(I1, End1),
  Begin0 > End1.

%% interval_meets(I0,I1) is semidet.
%
% Intervals I0 and I1 meet, i.e. the end time of I0 is equal to the start time of I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_meets(I0, I1) :-
  interval_end(I0, Time),
  interval_start(I1, Time).

%% interval_starts(I0,I1) is semidet.
%
% Interval I0 starts interval I1, i.e. both have the same start time, but I0 finishes earlier
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_starts(I0, I1) :-
  interval(I0, [T_start,End0]),
  interval(I1, I1_val),
  once(( I1_val = [T_start,End1], End0 < End1 ) ;
         I1_val = [T_start] ).

%% interval_finishes(I0,I1) is semidet.
%
% Interval I0 finishes interval I1, i.e. both have the same end time, but I0 starts later
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_finishes(I0, I1) :-
  interval(I0, [Begin0,T_end]),
  interval(I1, [Begin1,T_end]),
  Begin0 > Begin1.

%% interval_overlaps(I0,I1) is semidet.
%
% Interval I0  overlaps temporally with I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_overlaps(I0, I1) :-
  interval(I0, I0_val),
  interval(I1, I1_val),
  interval_start(I0_val, Begin0),
  interval_start(I1_val, Begin1),
  Begin0 < Begin1, % begins before the start of
  (interval_end(I0_val, End0) -> End0 > Begin1 ; true),
  (interval_end(I1_val, End1) -> (
    nonvar(End0), End0 < End1
  ) ; true).

%  ( interval(I0, [Begin0,End0]);
%    interval(I0, [Begin0]) ),
%  (( interval(I1, [Begin1,End1]),
%     ( nonvar(End0), (End0 < End1) ));   % ends before the end of
%     interval(I1, [Begin1]) ),
%  (var(End0) ; (End0 > Begin1)),         % ends after the start of
%  (Begin0 < Begin1).                   % begins before the start of

%% interval_during(I0,I1) is semidet.
%
% Interval I0 is inside interval I1, i.e. it starts later and finishes earlier than I1.
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_during(Time0, I1) :-
  number(Time0), !,
  interval(I1, I1_val),
  interval_start(I1_val, Begin1),
  Begin1 =< Time0,
  (interval_end(I1_val, End1) -> Time0 =< End1 ; true).

interval_during(I0, I1) :-
  interval(I0, I0_val),
  interval(I1, I1_val),
  interval_start(I0_val, Begin0),
  interval_start(I1_val, Begin1),
  Begin1 =< Begin0,
  (interval_end(I1_val, End1)
  -> (
    interval_end(I0_val, End0),
    End0 =< End1
  ) ; true).
