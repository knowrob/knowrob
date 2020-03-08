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

:- module('knowrob/model/TimeInterval',
    [
      interval/2,
      interval_duration/2,
      interval_equal/2,
      interval_start/2,
      interval_end/2
    ]).
/** <module> Predicates for temporal reasoning in KnowRob

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/lang/ask')).
:- use_module(library('knowrob/lang/tell')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta interval(t,?),
            interval_duration(t,?),
            interval_start(t,?),
            interval_end(t,?).

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
