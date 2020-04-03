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

:- module('knowrob/lang/occurs',
    [
      occurs/1,  % ?Event
      occurs/2,  % ?Event, ?Time      
      occurs/3   % ?Event, ?Time, ?Type
    ]).
/** <module> Predicates for temporal reasoning in KnowRob

@author Daniel Beßler
@license BSD
*/

:- use_module(library('knowrob/model/TimeInterval'), [
    interval/2
]).
:- use_module(library('knowrob/reasoning/temporal/allen'), [
    interval_during/2
]).
:- use_module(library('knowrob/lang/ask'), [
    kb_type_of/2
]).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta occurs(r),
            occurs(r,?),
            occurs(r,?,r).

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
