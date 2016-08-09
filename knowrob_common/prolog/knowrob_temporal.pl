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
      holds/1,
      holds/2,
      occurs/1,
      occurs/2,
      occurs/3,
      assert_fluent_begin/3,
      assert_fluent_end/3,
      assert_fluent_end/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

% define holds as meta-predicate and allow the definitions
% to be in different parts of the source file
:- meta_predicate holds(0, ?, ?)
                  occurs(0, ?, ?, ?).

:- multifile holds/2
             occurs/3.

:- rdf_meta holds(t),
            holds(t,?)
            occurs(?),
            occurs(?,?),
            occurs(?,?,?),
            assert_fluent_begin(r,r,r),
            assert_fluent_end(r,r,r),
            assert_fluent_end(r,r)..

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
  current_time(T),
  holds(Term,T).

holds(Term, T) :-
  (  Term =.. [':', Namespace, Tail]
  -> (
     Tail =.. [Property,Subject,Object],
     % unpack namespace
     rdf_current_ns(Namespace, NamespaceUri),
     atom_concat(NamespaceUri, Property, PropertyUri),
     holds(PropertyUri, Subject, Object, T)
  ) ; (
     Term =.. [Property,Subject,Object],
     holds(Property, Subject, Object, T)
  )).

holds(Property, Subject, Object, T) :-
  rdf_triple(Property, Subject, Object),
  not( rdfs_individual_of(Subject, knowrob:'TemporalPart') ),
  (  var(T)
  -> T = [0.0,inf]
  ;  true
  ), !.
  
holds(Property, Subject, Object, T) :-
  rdfs_individual_of(Property, owl:'DatatypeProperty'),
  % query temporal part that declares '@Property = @Object'
  rdf_has(SubjectPart, knowrob:'temporalPartOf', Subject),
  once(rdf_has(Property, SubjectPart, Object)),
  % match time interval with @T
  rdf_has(SubjectPart, knowrob:'temporalExtend', TimeInterval),
  time_term(TimeInterval, [T0,T1]),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ), !.
  
holds(Property, Subject, Object, T) :-
  rdfs_individual_of(Property, owl:'ObjectProperty'),
  % query temporal part that declares '@Property = ObjectPart'
  % where ObjectPart is a temporal part of @Object
  rdf_has(SubjectPart, knowrob:'temporalPartOf', Subject),
  once(rdf_has(Property, SubjectPart, ObjectPart)),
  rdf_has(ObjectPart, knowrob:'temporalPartOf', Object),
  % match time interval with @T
  rdf_has(SubjectPart, knowrob:'temporalExtend', TimeInterval),
  time_term(TimeInterval, [T0,T1]),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ), !.

%% occurs(?Evt) is nondet.
%% occurs(?Evt,?T) is nondet.
%% occurs(?Evt,?T,?EvtType) is nondet.
%
% True iff @Evt occurs during @T. Where @T is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
% @EvtType specifies the type of the event, it must be a subclass of knowrob:'Event'.
%
% @param Evt Identifier of the event
% @param T   Timepoint or time interval
% @param EvtType Identifier of the event type
% 
occurs(Evt) :-
  current_time(T),
  occurs(Evt, T, knowrob:'Event').

occurs(Evt, T) :-
  occurs(Evt, T, knowrob:'Event').

occurs(Evt, T, EvtTyp) :-
  number(T), current_time(Now),
  occurs(Evt, [T,Now], EvtTyp), !.

occurs(Evt, T, EvtTyp) :-
  atom(T), time_term(T,T_val), current_time(Now),
  occurs(Evt, [T_val,Now], EvtTyp), !.

% Read event instance from RDF triple store
occurs(Evt, T, EvtTyp) :-
  rdfs_subclass_of(EvtType, knowrob:'Event'),
  rdfs_individual_of(Evt, EvtTyp),
  rdf_has(Evt, knowrob:'startTime', T0),
  (rdf_has(Evt, knowrob:'endTime', T1) ; current_time(T1)),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ).

% Compute event instance
occurs(Evt, T, EvtTyp) :-
  var(Evt),
  rdfs_subclass_of(EvtType, knowrob:'Event'),
  rdfs_computable_prolog_instance_of(Evt, Type),
  occurs(Evt, T, EvtTyp).

%% NOTE(daniel): Define computable occurs in external files like this:
%% knowrob_temporal:occurs(Evt, [T0,T1], knowrob:'MyEvent') :-
%%   var(Evt), compute_my_event(T0,T1,Evt).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% methods for asserting fluent relations


assert_fluent_begin(Subject, Predicate, Object) :-
  %TODO: what if fluent exist? -> noop
  % Create open interval (i.e., without end time specified)
  current_time(Now),
  create_timepoint(Now, IntervalStart),
  rdf_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', Interval),
  rdf_assert(Interval, 'http://knowrob.org/kb/knowrob.owl#startTime', IntervalStart),
  % Create temporal parts
  rdf_instance_from_class('http://knowrob.org/kb/knowrob.owl#TemporalPart', SubjectPart),
  rdf_assert(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalPartOf', Subject),
  rdf_assert(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalExtend', Interval),
  rdf_instance_from_class('http://knowrob.org/kb/knowrob.owl#TemporalPart', ObjectPart),
  rdf_assert(ObjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalPartOf', Object),
  rdf_assert(ObjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalExtend', Interval),
  % Link tempoiral parts via fluent property
  rdf_assert(SubjectPart, Predicate, ObjectPart).

assert_fluent_end(Subject, Predicate, Object) :-
  rdf_has(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalPartOf', Subject),
  rdf_has(ObjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalPartOf', Object),
  rdf_has(SubjectPart, Predicate, ObjectPart),
  rdf_has(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalExtend', Interval),
  not( rdf_has(Interval, 'http://knowrob.org/kb/knowrob.owl#endTime', _) ),
  current_time(Now),
  create_timepoint(Now, IntervalEnd),
  rdf_assert(Interval, 'http://knowrob.org/kb/knowrob.owl#endTime', IntervalEnd).

assert_fluent_end(Subject, Predicate) :-
  current_time(Now),
  forall((
    rdf_has(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalPartOf', Subject),
    rdf_has(SubjectPart, Predicate, _)
  ), (
    rdf_has(SubjectPart, 'http://knowrob.org/kb/knowrob.owl#temporalExtend', Interval),
    not( rdf_has(Interval, 'http://knowrob.org/kb/knowrob.owl#endTime', _) ),
    create_timepoint(Now, IntervalEnd),
    rdf_assert(Interval, 'http://knowrob.org/kb/knowrob.owl#endTime', IntervalEnd)
  )).

