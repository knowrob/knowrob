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

:- module(knowrob_temporal,
    [
      current_time/1,  % -CurrentTime
      holds/1,   % ?Predicate(?Subject,?Object)
      holds/2,   % ?Predicate(?Subject,?Object), +Interval
      holds/3,   % ?Subject, ?Predicate, ?Object
      holds/4,   % ?Subject, ?Predicate, ?Object, +Interval
      occurs/1,  % ?Event
      occurs/2,  % ?Event, ?Time      
      occurs/3,  % ?Event, ?Time, ?Type
      owl_individual_of_during/3,  % ?Resource, ?Description, ?Interval
      owl_individual_of_during/2,  % ?Resource, ?Description
      owl_has_during/4,            % ?Subject, ?Predicate, ?Object, ?Interval
      rdfs_instance_of_during/3,   % ?Resource, ?Description, ?Interval
      rdfs_has_during/4,           % ?Subject, ?Predicate, ?Object, ?Interval
      rdf_triple_during/4,         % ?Predicate, ?Subject, ?Object, ?Interval
      assert_temporal_part/3,
      assert_temporal_part/4,
      assert_temporal_part/5,
      assert_temporal_part_end/5,
      assert_temporal_part_end/4,
      assert_temporal_part_end/3,
      current_temporal_part/2,
      temporal_part/3,
      temporal_part_has/3,
      temporal_part_has/4,
      time_term/2,
      time_between/3,
      time_between/2,
      time_later_then/2,
      time_earlier_then/2,
      interval/2,
      interval_start/2,
      interval_end/2,
      interval_before/2,
      interval_after/2,
      interval_meets/2,
      interval_starts/2,
      interval_finishes/2,
      interval_overlaps/2,
      interval_during/2
    ]).
/** <module> Predicates for temporal reasoning in KnowRob

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta rdf_triple_during(r,r,t),
            rdfs_instance_of_during(r,r,t),
            owl_individual_of_during(r,r),
            owl_individual_of_during(r,r,?),
            owl_has_during(r,r,t,?),
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
            assert_temporal_part(r,r,r,r,+),
            assert_temporal_part(r,r,r,r),
            assert_temporal_part(r,r,r),
            assert_temporal_part_end(r,r,r,r,+),
            assert_temporal_part_end(r,r,r,r),
            assert_temporal_part_end(r,r,r),
            temporal_part(r,r,t),
            temporal_part_has(r,r,r),
            temporal_part_has(r,r,r,t).

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

		 /*******************************
		 *	 high-level predicates		*
		 *******************************/

%% holds(+Term).
%% holds(+Term, ?Interval).
%% holds(?S, ?P, ?O).
%% holds(?S, ?P, ?O, ?Interval).
%
% True iff relation `P(S,O)` holds during Interval, and with Term=P(S,O).
% Where Interval is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Term Must be of the form: PROPERTY(SUBJECT, OBJECT).
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers representing a time interval.
%
holds(Term) :-
  current_time(T),
  holds(Term, [T,T]).

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
  )).

holds(S, P, O) :-
  current_time(T),
  holds(S, P, O, [T,T]).

holds(S, P, O, T) :-
  atom(T), !,
  interval(T,I),
  owl_has_during(S, P, O, I).

holds(S, P, O, T) :-
  number(T), !,
  owl_has_during(S, P, O, [T,T]).

holds(S, P, O, I) :-
  owl_has_during(S, P, O, I).

%% occurs(?Evt) is nondet.
%% occurs(?Evt,?Time) is nondet.
%% occurs(?Evt,?Time,?Type) is nondet.
%
% True iff Evt occurs during Time. Where Time is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Evt Identifier of the event
% @param Time Timepoint or time interval
% @param Type The event type iri
% 
occurs(Evt) :-
  current_time(CurrentTime),
  occurs(Evt, CurrentTime, knowrob:'Event').

occurs(Evt, Time) :-
  occurs(Evt, Time, knowrob:'Event').

occurs(Evt, Interval, Type) :-
  rdfs_individual_of(Evt, Type),
  interval(Evt, EvtI),
  (  ground(Interval)
  -> interval_during(Interval, EvtI)
  ;  Interval = EvtI ).

		 /*******************************
		 *	 	  OWL expansion			*
		 *******************************/

owl_temporal_db(I, db(rdfs_has_during(I),
                      rdfs_instance_of_during(I))).

%% owl_individual_of_during(?Resource,?Description) is nondet.
%% owl_individual_of_during(?Resource,?Description,+Interval) is nondet.
%
% True if Resource satisfies Description during Interval
% (or at the moment if not specified)
% according the the OWL-Description entailment rules,
% with additional computable and temporal semantics.
%
% @param Resource OWL resource identifier
% @param Description OWL class description
% @param Interval The interval during which the Resource satisfies Description
% 
owl_individual_of_during(Resource, Description) :-
  current_time(Now),
  owl_individual_of_during(Resource, Description, [Now,Now]).

owl_individual_of_during(Resource, Description, Interval) :-
  owl_temporal_db(Interval, DB),
  ( ground([Resource,Description,Interval]) ->
    once(owl_individual_of(Resource, Description, DB)) ; 
    owl_individual_of(Resource, Description, DB) ).

%% owl_has_during(?Subject, ?Predicate, ?Object, ?Interval).
%
% True if this relation is specified, or can be deduced using OWL
% inference rules, computable semantics, or temporal semantics.
%
% @param Subject OWL resource iri
% @param Predicate Property iri
% @param Object OWL resource iri or datatype value
% @param Interval The interval during which the relation holds
%
owl_has_during(S, P, O, Interval) :-
  owl_temporal_db(Interval, DB),
  ( ground([S,P,O,Interval]) ->
    once(owl_has(S, P, O, DB)) ; 
    owl_has(S, P, O, DB) ).

		 /*******************************
		 *	   Temporal semantics		*
		 *******************************/

%% rdfs_instance_of_during(?Resource, ?RDF_Type, Interval) is nondet.
%
% True if RDF_Type is the type of Resource during Interval
% according to RDFS, computable, or temporal semantics.
%
% @param Resource RDF resource iri
% @param RDF_Type RDF type iri
% @param Interval The interval during which RDF_Type is the type of Resource
% 
rdfs_instance_of_during(Resource, RDF_Type, Interval) :-
  var(Resource),
  rdfs_individual_of(X, RDF_Type),
  ( rdfs_individual_of(X, knowrob:'TemporalPart') -> (
    temporal_part_during(X, Interval),
    rdf_has(Resource, knowrob:temporalParts, X)
  ) ; (
    rdfs_instance_of_during1(X, RDF_Type, Interval),
    Resource = X
  )).

rdfs_instance_of_during(Resource, RDF_Type, Interval) :-
  nonvar(Resource),
  rdfs_individual_of(Resource, RDF_Type),
  rdfs_instance_of_during1(Resource, RDF_Type, Interval).

rdfs_instance_of_during(Resource, RDF_Type, Interval) :-
  nonvar(Resource),
  rdf_has(Resource, knowrob:temporalParts, X),
  rdfs_individual_of(X, RDF_Type),
  temporal_part_during(X, Interval).

% computable rdf:type
rdfs_instance_of_during(Resource, RDF_Type, Interval) :-
  rdf_equal(rdf:type, Property),
  ( nonvar(RDF_Type) ->
    rdfs_subclass_of(Type, RDF_Type) ;
    Type = RDF_Type ),
  rdfs_computable_triple_during(Property, Resource, Type, Interval).

rdfs_instance_of_during1(Resource, RDF_Type, Interval) :-
  \+ ( % TODO: this is a bit ugly
    rdf_has(Resource, knowrob:temporalParts, X),
    rdfs_individual_of(X, RDF_Type)
  ),
  ( nonvar(Interval) ->
    true ;
    owl_instance_from_class(knowrob:'TimeInterval', [begin=0.0], Interval)
  ).

temporal_part_during(TemporalPart, Interval) :-
  rdf_has(TemporalPart, knowrob:temporalExtend, TemporalExtend),
  ( nonvar(Interval) ->
    interval_during(Interval, TemporalExtend) ;
    Interval=TemporalExtend
  ).

%% rdfs_has_during(?Subject, ?Predicate, ?Object, ?Interval) is nondet.
%
% True if the RDF triple store contains (Subject Predicate Object),
% or if the triple can be derived from computable predicates,
% or temporal semantics.
%
% @param Property The property iri
% @param Frame RDF resource iri
% @param Value RDF resource iri or datatype value
% @param Interval The interval during which the triple holds
%
rdfs_has_during(S, P, O, I) :- rdf_triple_during(P, S, O, I).

%% rdf_triple_during(?Property, ?Frame, ?Value, ?Interval) is nondet.
%
% True if the RDF triple store contains (Frame Property Value),
% or if the triple can be derived from computable predicates,
% or temporal semantics.
%
% @param Property The property iri
% @param Frame RDF resource iri
% @param Value RDF resource iri or datatype value
% @param Interval The interval during which the triple holds
%
rdf_triple_during(P, S, O, Interval) :-
  var(Interval),
  ( temporal_part_has(S,P,O,_,TemporalExtend) *->
    time_term(TemporalExtend, Interval) ; (
    % assume non temporal property if no temporal part exists
    rdf_has(S,P,O),
    Interval=[0.0]
  )).

rdf_triple_during(P, S, O, Interval) :-
  nonvar(Interval),
  ( temporal_part_has(S,P,O,Interval,_) *->
    true ;
    % assume non temporal property if no temporal part exists
    rdf_has(S,P,O)
  ).

rdf_triple_during(Property, Frame, Value, Interval) :-
  ground(Property),
  rdfs_subproperty_of(SubProperty, Property),
  rdfs_computable_triple_during(SubProperty, Frame, Value, Interval).
  
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

temporal_part_during(Obj,TemporalPart,TemporalExtend) :-
  temporal_part(Obj,TemporalPart,I),
  interval_during(TemporalExtend, I).

create_temporal_part(S, P, TemporalPart) :-
  create_temporal_part(S, P, TemporalPart, user).
create_temporal_part(S, P, TemporalPart, Graph) :-
  rdf_instance_from_class(knowrob:'TemporalPart', TemporalPart),
  rdf_assert(S, knowrob:'temporalParts', TemporalPart, Graph),
  rdf_assert(TemporalPart, knowrob:'temporalProperty', P, Graph).


%% assert_temporal_part(+S,+P,+O) is nondet.
%% assert_temporal_part(+S,+P,+O,+I) is nondet.
%% assert_temporal_part(+S,+P,+O,+I,+Graph) is nondet.
%
% Asserts a temporal relation between `S` and `O` using the relation
% predicate `P` that holds during the time interval `I`. `O` is either an OWL individual,
% a term `nontemporal(Iri)` or a typed data value.
%
% @param S    The subject of the relation
% @param P    The relation predicate
% @param O    The object of the relation or a typed data value
% @param I    The time interval during which the relation holds
% @param Graph  The RDF graph name
% 
assert_temporal_part(S, P, O) :-
  current_time(Now),
  assert_temporal_part(S, P, O, Now).
assert_temporal_part(S, P, O, I) :-
  assert_temporal_part(S, P, O, I, user).
assert_temporal_part(S, P, O, _, Graph) :-
  nonvar(S),
  once(owl_individual_of(S, knowrob:'TemporalThing')),
  assert_nontemporal_value(S,P,O,Graph), !.
assert_temporal_part(S, P, O, I, Graph) :-
  create_temporal_part(S, P, TemporalPart, Graph),
  assert_temporal_part_value(TemporalPart, P, O, Graph),
  assert_temporal_part_extend(TemporalPart, I, Graph),
  % Also assert nontemporal if temporal part is not finished by now.
  % This allows nontemporal reasoners to work with descriptions generated here.
  ignore((
    current_temporal_part(S,TemporalPart),
    assert_nontemporal_value(S,P,O, Graph)
  )).

assert_nontemporal_value(S, P, O) :-
  assert_nontemporal_value(S, P, O, user).
assert_nontemporal_value(S, P, nontemporal(Value), Graph) :-
  rdf_assert(S, P, Value, Graph), !.
assert_nontemporal_value(S, P, Value, Graph) :-
  rdf_assert_prolog(S, P, Value, Graph).

assert_temporal_part_value(TemporalPart, P, Value) :-
  assert_temporal_part_value(TemporalPart, P, Value, user).
assert_temporal_part_value(TemporalPart, P, Value, Graph) :-
  rdf_has(P, rdf:type, owl:'DatatypeProperty'), !,
  rdf_assert_prolog(TemporalPart, P, Value, Graph).
assert_temporal_part_value(TemporalPart_S, P, nontemporal(Value), Graph) :-
  rdf_assert(TemporalPart_S, P, Value, Graph), !.
assert_temporal_part_value(TemporalPart_S, P, Value, Graph) :-
  create_temporal_part(Value, P, TemporalPart_O), !,
  rdf_assert(TemporalPart_S, P, TemporalPart_O, Graph),
  rdf_assert(Value, knowrob:'temporalParts', TemporalPart_O, Graph).

% TODO: this findall/forall looks ugly
assert_temporal_part_extend(TemporalPart, I) :-
  assert_temporal_part_extend(TemporalPart, I, user).
assert_temporal_part_extend(TemporalPart, I, Graph) :-
  owl_instance_from_class(knowrob:'TimeInterval', I, Interval),
  findall(X, (
      rdf_has(TemporalPart,_,X),
      rdfs_individual_of(X, knowrob:'TemporalPart')
  ), Parts),
  forall( member(Part, [TemporalPart|Parts]),
          rdf_assert(Part, knowrob:'temporalExtend', Interval, Graph)
  ).
retract_temporal_extend(TemporalPart, _Graph) :-
  findall(X, (
      rdf_has(TemporalPart,_,X),
      rdfs_individual_of(X, knowrob:'TemporalPart')
  ), Parts),
  forall( member(Part, [TemporalPart|Parts]),
          rdf_retractall(Part, knowrob:'temporalExtend', _)
  ).

%% assert_temporal_part_end(+S,?P,?O) is nondet.
%% assert_temporal_part_end(+S,?P,?O,+End) is nondet.
%% assert_temporal_part_end(+S,?P,?O,+End,+Graph) is nondet.
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
% @param Graph  The RDF graph name
% 
assert_temporal_part_end(S, P, O) :-
  current_time(Now),
  assert_temporal_part_end(S, P, O, Now).
assert_temporal_part_end(S, P, O, End) :-
  assert_temporal_part_end(S, P, O, End, user).
assert_temporal_part_end(S, P, O, End, Graph) :-
  atom(End),
  atom_number(End, End_num),
  assert_temporal_part_end(S, P, O, End_num, Graph).
assert_temporal_part_end(S, P, O, End, Graph) :-
  number(End),
  current_temporal_part(S,S_temporal),
  rdf_has(S_temporal, knowrob:'temporalProperty', P),
  rdf_has(S_temporal, knowrob:'temporalExtend', I),
  temporal_part_value(S_temporal, P, O),
  % update temporal extend
  rdf_has(I, knowrob:'startTime', BeginIri),
  time_term(BeginIri, Begin),
  retract_temporal_extend(S_temporal, Graph),
  assert_temporal_part_extend(S_temporal, [Begin,End], Graph),
  % remove the nontemporal property if exists
  ignore(( temporal_part_value(S, P, O),
           rdf_retractall(S, P, O) )).
assert_temporal_part_end(S, _, _, _, _Graph) :-
  nonvar(S),
  once(owl_individual_of(S, knowrob:'TemporalThing')), !.
assert_temporal_part_end(S, P, O, End, Graph) :-
  number(End),
  % Make relation temporal if the relation was not described temporally before
  temporal_part_value(S, P, O),
  rdf_retractall(S, P, O),
  assert_temporal_part(S, P, O, [0.0,End], Graph).

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


%% temporal_part_has(?Subject, ?Predicate, ?Object).
%% temporal_part_has(?Subject, ?Predicate, ?Object, ?Interval).
%
% True if this relation is specified via knowrob:TemporalPart
% or can be deduced using OWL inference rules.
% 
temporal_part_has(S, P, O) :- temporal_part_has(S, P, O, _).

temporal_part_has(S, P, O, I) :-
  temporal_part_has(S, P, O, I, _).

temporal_part_has(S, P, O, I, Lifespan) :-
  rdf_has(S, knowrob:'temporalParts', TemporalPart),
  temporal_part_has(TemporalPart, P, O, I, Lifespan).

temporal_part_has(S, P, O, Interval, Lifespan) :-
  rdf_has(S, knowrob:'temporalProperty', P),
  % find the lifespan of the property assertion
  rdf_has(S, knowrob:'temporalExtend', Ext),
  interval(Ext, Lifespan),
  ( ground(Interval) ->
    interval_during(Interval, Lifespan) ;
    Interval=Lifespan ),
  % read from the triple store
  rdf_has(S, P, S_O),
  % ignore some specific type statements in case of
  % temporal type assertions
  (rdf_equal(P, rdf:type)
  -> \+ rdf_equal(S_O, owl:'NamedIndividual'),
     \+ rdf_equal(S_O, knowrob:'TemporalPart')
  ;  true),
  % format the property value
  (rdfs_individual_of(S_O, knowrob:'TemporalPart')
  -> once(owl_has(O, knowrob:temporalParts, S_O))
  ; (
    ( ground(O) -> (
      strip_literal_type(S_O,S_O_stripped),
      strip_literal_type(O,O_stripped),
      O_stripped = S_O_stripped ) ;
    ( O = S_O ))
  )).

		 /*******************************
		 *		Time instants			*
		 *******************************/

%% current_time(-CurrentTime).
%
% Unifies CurrentTime with the current time in seconds.
%
current_time(CurrentTime) :-
  set_prolog_flag(float_format, '%.12g'),
  get_time(CurrentTime).

%% time_term(+Timepoint, -TimeTerm).
%
% TimeTerm is a numerical representation of Timepoint.
%
time_term(Timepoint, Timepoint) :-
  number(Timepoint), !.

time_term([Begin,End], [Begin,End]) :-
  number(Begin), number(End), !.

time_term([Begin], [Begin]) :-
  number(Begin), !.

time_term(Timeinterval, Interval) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  rdf_has(Timeinterval, knowrob:'startTime', Timepoint0),
  time_term(Timepoint0, Begin),
  (  rdf_has(Timeinterval, knowrob:'endTime', Timepoint1)
  -> (time_term(Timepoint1, End), Interval=[Begin,End])
  ;  Interval=[Begin]
  ), !.

time_term(Timepoint, Time) :-
  atom(Timepoint),
  (  rdf_split_url(_, TimePointLocal, Timepoint),
     atom_concat('timepoint_', TimeAtom, TimePointLocal)
  -> term_to_atom(Time, TimeAtom)
  ;  (  atom_concat('timepoint_', TimepointAtom, Timepoint)
     -> term_to_atom(Time, TimepointAtom)
     ;  term_to_atom(Time, Timepoint)
     )
  ).

%% time_between(+T, +T0, +T1).
%% time_between(+T, +Interval).
%
% True iff T0 <= T <= T1, and
% Interval=[T0,T1].
%
time_between(Timeinterval, T0, T1) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  time_term(Timeinterval , Interval),
  time_between(Interval, T0, T1), !.

time_between([T2,T3], T0, T1) :-
  time_between(T2, T0, T1),
  time_between(T3, T0, T1), !.

time_between(T, T0, T1) :-
  not(is_list(T)), 
  time_earlier_then(T0, T),
  time_earlier_then(T, T1).

time_between(T, Timeinterval) :-
  atom(Timeinterval),
  rdfs_individual_of(Timeinterval, knowrob:'TimeInterval'),
  time_term(Timeinterval , Interval),
  time_between(T, Interval), !.

time_between(T, [Begin,End]) :-
  time_between(T, Begin, End).

time_between(T, [Begin]) :-
  time_later_then(T, [Begin]).


%% time_later_then(+T0, +T1).
%
% True iff T0 >= T1
%
time_later_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_later_then_(T0_term, T1_term), !.
time_later_then_([_], [_])     :- false.
time_later_then_([T0], [_,T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [_,T1]) :- T1 =< T0, !.
time_later_then_(T0, T1) :- number(T0), number(T1), T1 =< T0, !.

%% time_earlier_then(+T0, +T1).
%
% True iff T0 <= T1
%
time_earlier_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_earlier_then_(T0_term, T1_term), !.
time_earlier_then_([_], [_])     :- false.
time_earlier_then_([T0], [_,T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [_,T1]) :- T0 =< T1, !.
time_earlier_then_(T0, T1) :- number(T0), number(T1), T0 =< T1, !.

		 /*******************************
		 *	Allen's interval algebra	*
		 *******************************/

%% interval(+In,-Out) is semidet.
%
% True if Out is the interval of In.
%
% @param In Time point, interval or temporally extended entity
% @param Out Start and end time of the interval
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
  ignore(interval_end(I0_val, End0)),
  ignore(interval_end(I1_val, End1)),
  ((var(End0), var(End1));
   (var(End0) -> Begin0 < End1;
   (var(End1) -> Begin1 < End0;
   (Begin0=<End1, End0>=Begin1)
   ))).

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
