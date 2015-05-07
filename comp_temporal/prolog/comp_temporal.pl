/** <module> Predicates for spatial reasoning

  Contains all computables that calculate temporal relations between events
  to allow for temporal reasoning.

  Now extended with predicates for reasoning on other relations over time (esp.
  holds/holds_tt)

  Copyright (C) 2009-13 Moritz Tenorth, Lars Kunze
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

@author Moritz Tenorth, Lars Kunze
@license BSD

*/
:- module(comp_temporal,
    [
     comp_temporallySubsumes/2,
     comp_after/2,
     comp_duration/2,
     comp_equalI/2,
     comp_afterI/2,
     comp_beforeI/2,
     comp_meetsI/2,
     comp_meetsInvI/2,
     comp_overlapsI/2,
     comp_overlapsInvI/2,
     comp_startsI/2,
     comp_startsInvI/2,
     comp_duringI/2,
     comp_duringInvI/2,
     comp_finishesI/2,
     comp_finishesInvI/2,
     time_point_value/2,
     start_time_value/2,
     end_time_value/2
    ]).

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
    comp_finishesInvI(r, r),
    time_point_value(r,-),
    start_time_value(r,-),
    end_time_value(r,-).


:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).

:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_temporal, 'http://knowrob.org/kb/comp_temporal.owl#', [keep(true)]).


%% comp_temporallySubsumes(?Long, ?Short) is nondet.
%
% Check if the time segment Long contains the segment or time point Short.
%
% @param Long Identifier of the longer time segment
% @param Short Identifier of the contained time segment or time point
%
comp_temporallySubsumes(Long, Short) :-

  % case: both temporally extended, i.e. start and end set

  % read times for the longer event
  rdf_triple(knowrob:startTime, Long, Ls),
  rdf_triple(knowrob:endTime, Long, Le),

  % convert time stamps to numbers
  rdf_split_url(_, LsLocal, Ls),
  atom_concat('timepoint_', LsAtom, LsLocal),
  term_to_atom(Lstart, LsAtom),

  rdf_split_url(_, LeLocal, Le),
  atom_concat('timepoint_', LeAtom, LeLocal),
  term_to_atom(Lend, LeAtom),

  % read times for the shorter event
  rdf_triple(knowrob:startTime, Short, Ss),
  rdf_triple(knowrob:endTime, Short, Se),

  % convert time stamps to numbers
  rdf_split_url(_, SsLocal, Ss),
  atom_concat('timepoint_', SsAtom, SsLocal),
  term_to_atom(Sstart, SsAtom),

  rdf_split_url(_, SeLocal, Se),
  atom_concat('timepoint_', SeAtom, SeLocal),
  term_to_atom(Send, SeAtom),

  % compare the start and end times
  (Sstart=<Send),
  (Lstart=<Sstart), (Sstart=<Lend),
  (Lstart=<Send),   (Send=<Lend).


%%  comp_after(Pre, After) is nondet.
%
% Check if the time point Pre is before the time point After
%
% @param Pre Identifier of the earlier time point
% @param After Identifier of the later time point
%
comp_after(Pre, After) :-
  rdf_has(Pre,   rdf:type, knowrob:'TimePoint'),
  rdf_has(After, rdf:type, knowrob:'TimePoint'),

  rdf_split_url(_, PreLocal, Pre),
  atom_concat('timepoint_', PreAtom, PreLocal),
  term_to_atom(P, PreAtom),

  rdf_split_url(_, AfterLocal, After),
  atom_concat('timepoint_', AfterAtom, AfterLocal),
  term_to_atom(A, AfterAtom),

  P<A.

%%  comp_time_point_start_time(Pre, After) is nondet.
%
% Start time of a time point is the time point itself
%
% @param T Identifier of a time point
%
comp_time_point_start_time(T, T) :-
  rdf_has(T, rdf:type, knowrob:'TimePoint').

%%  comp_time_point_end_time(Pre, After) is nondet.
%
% End time of a time point is the time point itself
%
% @param T Identifier of a time point
%
comp_time_point_end_time(T, T) :-
  rdf_has(T, rdf:type, knowrob:'TimePoint').

%%  comp_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
comp_duration(Event, Duration) :-
  rdf_has(Event, knowrob:startTime, Es),
  rdf_has(Event, knowrob:endTime, Ee),

  rdf_split_url(_, StartLocal, Es),
  atom_concat('timepoint_', StartAtom, StartLocal),
  term_to_atom(Start, StartAtom),

  rdf_split_url(_, EndLocal, Ee),
  atom_concat('timepoint_', EndAtom, EndLocal),
  term_to_atom(End, EndAtom),

  Duration is (End-Start).


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
   start_time_value(I1,ST),
   end_time_value(I1,ET),
   start_time_value(I2,ST),
   end_time_value(I2,ET).
   %I1 \= I2.


   
%% comp_beforeI(I1,I2) is semidet.
%
%  Interval I1 takes place before Y
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_beforeI(I1,I2) :-
   end_time_value(I1,ET),
   start_time_value(I2,ST),
   %I1 \= I2,
   ET < ST.

   

%% comp_afterI(I1,I2) is semidet.
%
% Interval I1 takes place after I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_afterI(I1,I2) :-
   comp_beforeI(I2,I1).


   
%% comp_overlapsI(I1,I2) is semidet.
%
% Interval I1  overlaps temporally with I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_overlapsI(I1,I2) :-
   start_time_value(I1,ST1),
   end_time_value(I1,ET1),
   start_time_value(I2,ST2),
   end_time_value(I2,ET2),
   (ST1 < ST2),
   (ET1 > ST2),
   (ET1 < ET2).


   
%% comp_overlapsInvI(I1,I2) is semidet.
%
% Interval I2  overlaps temporally with I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_overlapsInvI(I1,I2) :-
   comp_overlapsI(I2,I1).


   
%% comp_meetsI(I1,I2) is semidet.
%
% Intervals I1 and I2 meet, i.e. the end time of I1 is equal to the start time of I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_meetsI(I1,I2) :-
   end_time_value(I1,T),
   start_time_value(I2,T).


   
%% comp_meetsInvI(I1,I2) is semidet.
%
% Intervals I1 and I2 meet, i.e. the end time of I2 is equal to the start time of I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_meetsInvI(I1,I2) :-
   comp_meetsI(I2,I1).


   
%% comp_duringI(I1,I2) is semidet.
%
% Interval I1 is inside interval I2, i.e. it starts later and finishes earlier than I2.
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_duringI(I1,I2) :-
   start_time_value(I1,ST1),
   end_time_value(I1,ET1),
   start_time_value(I2,ST2),
   end_time_value(I2,ET2),
   ST1 > ST2,
   ET1 < ET2.


   
%% comp_duringInvI(I1,I2) is semidet.
%
% Interval I2 is inside interval I1, i.e. it starts later and finishes earlier than I1.
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_duringInvI(I1,I2) :-
   comp_duringI(I2,I1).


   
%% comp_startsI(I1,I2) is semidet.
%
% Interval I1 starts interval I2, i.e. both have the same start time, but I1 finishes earlier
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_startsI(I1,I2) :-
   start_time_value(I1,ST1),
   end_time_value(I1,ET1),
   start_time_value(I2,ST2),
   end_time_value(I2,ET2),
   ST1 = ST2,
   ET1 < ET2.


   
%% comp_startsInvI(I1,I2) is semidet.
%
% Interval I2 starts interval I1, i.e. both have the same start time, but I2 finishes earlier
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_startsInvI(I1,I2) :-
   comp_startsI(I2,I1).


   
%% comp_finishesI(I1,I2) is semidet.
%
% Interval I1 finishes interval I2, i.e. both have the same end time, but I1 starts later
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_finishesI(I1,I2) :-
   start_time_value(I1,ST1),
   end_time_value(I1,ET1),
   start_time_value(I2,ST2),
   end_time_value(I2,ET2),
   ST1 > ST2,
   ET1 = ET2.


   
%% comp_finishesInvI(I1,I2) is semidet.
%
% Interval I2 finishes interval I1, i.e. both have the same end time, but I2 starts later
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
comp_finishesInvI(I1,I2):-
   comp_finishesI(I2,I1).


   

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% helper predicates

%% time_point_value(TP, Value) is semidet.
%
% Extracts the numeric time stamp value from a knowrob:TimePoint
%
% @param TP    Instance of a knowrob:TimePoint
% @param Value Numeric value of the time stamp of TP
% 
time_point_value(TP, Value) :-
  rdf_split_url(_, StartLocal, TP),
  atom_concat('timepoint_', StartAtom, StartLocal),
  term_to_atom(Value, StartAtom).


  
%% start_time_value(I,Value) is semidet.
%
% Extracts the numeric value of the knowrob:startTime of a knowrob:TimeInterval
%
% @param I     Instance of a knowrob:TimeInterval
% @param Value Numeric value of the startTime time stamp of I
% 
start_time_value(I,Value) :-
  rdf_has(I, knowrob:startTime, TP),
  time_point_value(TP, Value).


  
%% end_time_value(I, Value) is semidet.
%
% Extracts the numeric value of the knowrob:endTime of a knowrob:TimeInterval
%
% @param I     Instance of a knowrob:TimeInterval
% @param Value Numeric value of the endTime time stamp of I
% 
end_time_value(I, Value) :-
  ( rdf_has(I, knowrob:endTime, TP),
    time_point_value(TP, Value), ! ) ;
  start_time_value(I,Value).



