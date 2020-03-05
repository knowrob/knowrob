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
      occurs/1,  % ?Event
      occurs/2,  % ?Event, ?Time      
      occurs/3,  % ?Event, ?Time, ?Type
      allen_constraint/2,
      time_between/3,
      time_between/2,
      time_later_then/2,
      time_earlier_then/2,
      interval/2,
      interval_duration/2,
      interval_equal/2,
      interval_start/2,
      interval_end/2,
      interval_before/2,
      interval_after/2,
      interval_meets/2,
      interval_met_by/2,
      interval_starts/2,
      interval_started_by/2,
      interval_finishes/2,
      interval_finished_by/2,
      interval_overlaps/2,
      interval_overlapped_by/2,
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
:- use_module(library('knowrob/knowrob')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta occurs(r),
            occurs(r,?),
            occurs(r,?,r),
            allen_constraint(r,t),
            interval(t,?),
            interval_duration(t,?),
            interval_start(t,?),
            interval_end(t,?),
            interval_before(t,t),
            interval_after(t,t),
            interval_meets(t,t),
            interval_met_by(t,t),
            interval_starts(t,t),
            interval_started_by(t,t),
            interval_finishes(t,t),
            interval_finished_by(t,t),
            interval_overlaps(t,t),
            interval_overlapped_by(t,t),
            interval_during(t,t).

:- rdfs_computable
            interval_after(ease:after),
            interval_before(ease:before),
            interval_meets(ease:meets),
            interval_met_by(ease:metBy),
            interval_overlaps(ease:overlappedOn),
            interval_overlapped_by(ease:overlappedBy),
            interval_starts(ease:starts),
            interval_started_by(ease:startedBy),
            interval_during(ease:during),
            interval_finishes(ease:finishes),
            interval_finished_by(ease:finishedBy),
            interval_equal(ease:simultaneous).

		 /*******************************
		 *	 high-level predicates		*
		 *******************************/

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
  occurs(Evt, _, dul:'Event').

occurs(Evt, Interval) :-
  interval(Evt, EvtI),
  (  ground(Interval)
  -> interval_during(Interval, EvtI)
  ;  Interval = EvtI ).

occurs(Evt, Interval, Type) :-
  kb_type_of(Evt, Type),
  interval(Evt, EvtI),
  (  ground(Interval)
  -> interval_during(Interval, EvtI)
  ;  Interval = EvtI ).

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

time_term(literal(type('http://www.w3.org/2001/XMLSchema#double',X)), Timepoint) :-
  ( number(X) ->
    Timepoint = X ;
    atom_number(X, Timepoint) ), !.

time_term(literal(type('http://www.w3.org/2001/XMLSchema#float',X)), Timepoint) :-
  ( number(X) ->
    Timepoint = X ;
    atom_number(X, Timepoint) ), !.

time_term([Begin,End], [Begin_v,End_v]) :-
  time_term(Begin,Begin_v),
  time_term(End,End_v), !.

time_term([Begin], [Begin_v]) :-
  time_term(Begin,Begin_v), !.

%time_term(Event, Interval) :-
  %atom(Event),
  %kb_triple(Event, knowrob:'startTime', Timepoint0),
  %time_term(Timepoint0, Begin),
  %(  kb_triple(Event, knowrob:'endTime', Timepoint1)
  %-> (time_term(Timepoint1, End), Interval=[Begin,End])
  %;  Interval=[Begin]
  %), !.

% @deprecated
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
  interval(Timeinterval, Interval),
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
  interval(Timeinterval , Interval),
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
  interval(T0, T0_term),
  interval(T1, T1_term),
  time_later_then_(T0_term, T1_term), !.
time_later_then_([_], [_])       :- fail.
time_later_then_([T0], [_,T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [_,T1]) :- T1 =< T0, !.
time_later_then_(T0, T1) :- number(T0), number(T1), T1 =< T0, !.

%% time_earlier_then(+T0, +T1).
%
% True iff T0 <= T1
%
time_earlier_then(T0, T1) :-
  interval(T0, T0_term),
  interval(T1, T1_term),
  time_earlier_then_(T0_term, T1_term), !.
time_earlier_then_([_], [_])       :- fail.
time_earlier_then_([T0], [_,T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [_,T1]) :- T0 =< T1, !.
time_earlier_then_(T0, T1) :- number(T0), number(T1), T0 =< T1, !.

		 /*******************************
		 *	Allen's interval algebra	*
		 *******************************/

%%
allen_constraint(Resource,Constraint) :-
  rdf_has(Resource,Relation,Other),
  atom(Relation),
  %rdfs_individual_of(Other,dul:'Task'),
  rdf_has(Relation,ease:symbol,X),
  kb_rdf_data_atom(X,Symbol),
  % FIXME: bad test
  Constraint =.. [Symbol,Resource,Other].

%% interval(+In,-Out) is semidet.
%
% True if Out is the interval of In.
%
% @param In Time point, interval or temporally extended entity
% @param Out Start and end time of the interval
% 
interval(I, I) :- is_list(I), !.
interval(Time, [Time,Time]) :- number(Time), !.
interval(Event, Interval) :-
  atom(Event),
  rdf_has(Event,dul:hasTimeInterval,X),!,
  interval(X, Interval).
interval(I, Interval) :-
  atom(I),
  ( rdf_has(I, knowrob:'startTime', T0) ;
    rdf_has(I, ease:'hasIntervalBegin', T0) ),
  time_term(T0, T0_val),
  (( rdf_has(I, knowrob:'endTime', T1) ;
     rdf_has(I, ease:'hasIntervalEnd', T1) )
  -> (time_term(T1, T1_val), Interval=[T0_val,T1_val])
  ;  Interval=[T0_val] ), !.
interval(Event, Interval) :-
  var(Event),
  rdfs_individual_of(Event, dul:'Event'),
  interval(Event,Interval).
interval(Situation, Interval) :-
  atom(Situation),
  rdfs_individual_of(Situation, dul:'Situation'),
  rdf_has(Situation,dul:includesTime,X),
  interval(X,Interval), !.
% @deprecated
interval(TimePoint, [Time,Time]) :-
  atom(TimePoint),
  rdfs_individual_of(TimePoint, knowrob:'TimePoint'),
  time_term(TimePoint, Time), !.

%% interval_equal(?I1,?I2) is semidet.
%
% Interval I1 is equal to I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_equal(I1,I2) :-
   interval(I1,[ST,ET]),
   interval(I2,[ST,ET]).

%%  interval_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
% @tbd Duration should be literal(type(qudt:'MinuteTime', Duration))
%
interval_duration(Event, Duration) :-
  interval(Event, [ST,ET]),
  Duration is (ET-ST).

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

%% interval_met_by(I1,I2) is semidet.
%
% Intervals I1 and I2 meet, i.e. the end time of I2 is equal to the start time of I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_met_by(I1,I2) :-
   interval_meets(I2,I1).

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

%% interval_started_by(I1,I2) is semidet.
%
% Interval I2 starts interval I1, i.e. both have the same start time, but I2 finishes earlier
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_started_by(I1,I2) :-
   interval_starts(I2,I1).

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

%% interval_finished_by(I1,I2) is semidet.
%
% Interval I2 finishes interval I1, i.e. both have the same end time, but I2 starts later
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_finished_by(I1,I2):-
   interval_finishes(I2,I1).

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
  Begin0 =< Begin1,
  End0 =< End1.

%% interval_overlapped_by(I1,I2) is semidet.
%
% Interval I2  overlaps temporally with I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_overlapped_by(I1,I2) :-
   interval_overlaps(I2,I1).

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
