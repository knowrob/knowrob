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
      assert_fluent_begin/3,
      assert_fluent_end/3,
      assert_fluent_end/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta holds(t),
            holds(t,?),
            occurs(?),
            occurs(?,?),
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
            assert_fluent_begin(r,r,r),
            assert_fluent_end(r,r,r),
            assert_fluent_end(r,r).

% define holds as meta-predicate and allow the definitions
% to be in different source files
:- meta_predicate holds(0, ?, ?),
                  occurs(0, ?, ?).
:- multifile holds/2,
             occurs/2.

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
%
% True iff @Evt occurs during @T. Where @T is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Evt Identifier of the event or a term of the form $Type($Instance)
% @param T   Timepoint or time interval
% 
occurs(Evt) :-
  current_time(T),
  occurs(Evt, T).

occurs(Evt, T) :-
  not(is_list(T)),
  time_term(T,T_val),
  (  is_list(T_val)
  -> occurs(Evt, T_val)
  ;  (
       current_time(Now),
       occurs(Evt, [T_val,Now])
     )
  ), !.

% Read event instance from RDF triple store
occurs(Evt, T) :-
  entity(Evt, Inst, EvtType),
  rdfs_subclass_of(EvtType, knowrob:'Event'),
  rdfs_individual_of(Inst, EvtType),
  rdf_has(Inst, knowrob:'startTime', T0),
  (rdf_has(Inst, knowrob:'endTime', T1) ; current_time(T1)),
  (  var(T)
  -> T = [T0,T1]
  ;  time_between(T, T0, T1)
  ).

% Compute event instance
occurs(Evt, T) :-
  entity(Evt, Inst, EvtType),
  var(Inst),
  rdfs_subclass_of(EvtType, knowrob:'Event'),
  rdfs_computable_prolog_instance_of(Inst, EvtType),
  occurs(Inst, T).

%% NOTE(daniel): Define computable occurs in external files like this:
%% knowrob_temporal:occurs(knowrob:'MyEvent'(Evt), [T0,T1]) :-
%%   var(Evt), compute_my_event(Evt,[T0,T1]).



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
interval(T, [T,T]) :- number(T).
interval([T0,T1], [T0,T1]).
interval(I, [T0,T1]) :-
  entity(I, Inst, _),
  rdf_has(Inst, knowrob:'temporalExtend', Ext),
  interval(Ext, [T0,T1]).
interval(I, [T0_val,T1_val]) :-
  entity(I, Inst, _),
  rdf_has(Inst, knowrob:'startTime', T0),
  (rdf_has(Inst, knowrob:'endTime', T1) ; current_time(T1)),
  time_term(T0, T0_val),
  time_term(T1, T1_val).

%% interval_start(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_start(I, Start) :- interval(I, [Start,_]).
%% interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_end(I, End)     :- interval(I, [_,End]).

%% interval_before(I0,I1) is semidet.
%
%  Interval I0 takes place before I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_before(I0, I1) :-
  interval(I0, [_,End0]),
  interval(I1, [Begin1,_]),
  time_earlier_then(End0,Begin1).

%% interval_after(I0,I1) is semidet.
%
% Interval I0 takes place after I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_after(I0, I1) :-
  interval(I0, [_,End0]),
  interval(I1, [Begin1,_]),
  time_later_then(Begin1,End0).

%% comp_meetsI(I0,I1) is semidet.
%
% Intervals I0 and I1 meet, i.e. the end time of I0 is equal to the start time of I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_meets(I0, I1) :-
  interval(I0, [_,T_meet]),
  interval(I1, [T_meet,_]).

%% interval_starts(I0,I1) is semidet.
%
% Interval I0 starts interval I1, i.e. both have the same start time, but I0 finishes earlier
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_starts(I0, I1) :-
  interval(I0, [T_start,End0]),
  interval(I1, [T_start,End1]),
  time_earlier_then(End0,End1).

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
  time_earlier_then(Begin1,Begin0).

%% interval_overlaps(I0,I1) is semidet.
%
% Interval I0  overlaps temporally with I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_overlaps(I0, I1) :-
  interval(I0, [Begin0,End0]),
  interval(I1, [Begin1,End1]),
  time_earlier_then(Begin0,End1),
  time_earlier_then(Begin1,End0).

%% interval_during(I0,I1) is semidet.
%
% Interval I0 is inside interval I1, i.e. it starts later and finishes earlier than I1.
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_during(I0, I1) :-
  interval(I0, [Begin0,End0]),
  interval(I1, [Begin1,End1]),
  time_earlier_then(Begin1,Begin0),
  time_earlier_then(End0,End1).
