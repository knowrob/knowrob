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

:- module('knowrob/reasoning/temporal/allen',
    [
      allen_constraint/2,
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
/** <module> Allen calculus implementation.

@author Daniel Beßler
@license BSD
*/
% TODO: allow reasoning only using qualitative realtions
%   (no interval data). ESGs can do it for example.

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('knowrob/triples/computable')).
:- use_module(library('knowrob/model/TimeInterval')).
:- use_module(library('knowrob/comp/rdf_data')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta interval_before(t,t),
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
            interval_before(ease:before),
            interval_after(ease:after),
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

%%
allen_constraint(Resource,Constraint) :-
  rdf_has(Resource,Relation,Other),
  atom(Relation),
  %rdfs_individual_of(Other,dul:'Task'),
  rdf_has(Relation,ease:symbol,X),
  kb_rdf_data_atom(X,Symbol),
  % FIXME: bad test
  Constraint =.. [Symbol,Resource,Other].

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
