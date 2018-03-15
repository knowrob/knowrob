/*
  Contains all computables that calculate temporal relations between events
  to allow for temporal reasoning.

  Copyright (C) 2009-13 Moritz Tenorth, Lars Kunze; 2016 Daniel Beßler
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
:- module(comp_temporal,
    [
     comp_temporallySubsumes/2,
     comp_duration/2,
     comp_equalI/2,
     comp_meetsInvI/2,
     comp_overlapsInvI/2,
     comp_startsInvI/2,
     comp_duringInvI/2,
     comp_finishesInvI/2
    ]).
/** <module> Computable properties for spatial reasoning

@author Moritz Tenorth
@author Lars Kunze
@author Daniel Beßler
@license BSD

*/
:-  rdf_meta
    comp_temporallySubsumes(r, r),
    comp_after(r, r),
    comp_duration(r, r),
    comp_equalI(r, r),
    comp_afterI(r, r),
    comp_beforeI(r, r),
    comp_meetsI(r, r),
    comp_meetsInvI(r, r),
    comp_overlapsI(r, r),
    comp_overlapsInvI(r, r),
    comp_startsI(r, r),
    comp_startsInvI(r, r),
    comp_duringI(r, r),
    comp_duringInvI(r, r),
    comp_finishesI(r, r),
    comp_finishesInvI(r, r).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(comp_temporal, 'http://knowrob.org/kb/comp_temporal.owl#', [keep(true)]).


%% comp_temporallySubsumes(?Long, ?Short) is nondet.
%
% Check if the time segment Long contains the segment or time point Short.
%
% @param Long Identifier of the longer time segment
% @param Short Identifier of the contained time segment or time point
%
comp_temporallySubsumes(Long, Short) :-
  interval(Long, [Long_ST,Long_ET]),
  interval(Short, [Short_ST,Short_ET]),
  % compare the start and end times
  (Short_ST=<Short_ET),
  (Long_ST=<Short_ST), (Short_ST=<Long_ET),
  (Long_ST=<Short_ET), (Short_ET=<Long_ET).

%%  comp_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
% @tbd Duration should be literal(type(qudt:'MinuteTime', Duration))
%
comp_duration(Event, Duration) :-
  interval(Event, [ST,ET]),
  Duration is (ET-ST).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Allen's 13 temporal relations for intervals

%% comp_equalI(?I1,?I2) is semidet.
%
% Interval I1 is equal to I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_equalI(I1,I2) :-
   interval(I1,[ST,ET]),
   interval(I2,[ST,ET]).

%% comp_overlapsInvI(I1,I2) is semidet.
%
% Interval I2  overlaps temporally with I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_overlapsInvI(I1,I2) :-
   interval_overlaps(I2,I1).

%% comp_meetsInvI(I1,I2) is semidet.
%
% Intervals I1 and I2 meet, i.e. the end time of I2 is equal to the start time of I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_meetsInvI(I1,I2) :-
   interval_meets(I2,I1).

%% comp_duringInvI(I1,I2) is semidet.
%
% Interval I2 is inside interval I1, i.e. it starts later and finishes earlier than I1.
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_duringInvI(I1,I2) :-
   interval_during(I2,I1).

%% comp_startsInvI(I1,I2) is semidet.
%
% Interval I2 starts interval I1, i.e. both have the same start time, but I2 finishes earlier
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_startsInvI(I1,I2) :-
   interval_starts(I2,I1).

%% comp_finishesInvI(I1,I2) is semidet.
%
% Interval I2 finishes interval I1, i.e. both have the same end time, but I2 starts later
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_finishesInvI(I1,I2):-
   interval_finishes(I2,I1).
