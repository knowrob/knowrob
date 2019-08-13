/*
  Copyright (C) 2019 Daniel Beßler
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

:- module(knowrob_events,
    [
      event_create/2,
      event_create/3,
      event_begin_time/2,
      event_end_time/2,
      event_set_begin_time/2,
      event_set_end_time/2,
      event_participant/3
    ]).
/** <module> Methods for reasoning about event descriptions

@author Moritz Tenorth
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).

:- rdf_meta
      event_create(r,r),
      event_create(r,r,+),
      event_begin_time(r,?),
      event_end_time(r,?),
      event_set_begin_time(r,?),
      event_set_end_time(r,?),
      event_participant(r,r,r).

%%
event_create(EvtType,Evt) :-
  event_create(EvtType,Evt,events).

event_create(EvtType,Evt,Graph) :-
  rdfs_instance_from_class(EvtType,Graph,Evt),
  rdfs_instance_from_class(dul:'TimeInterval',Graph,I),
  rdf_assert(Evt,dul:hasTimeInterval,I,Graph).

%%
event_begin_time(Evt,Stamp) :-
  interval(Evt,[Stamp|_]).

%%
event_end_time(Evt,Stamp) :-
  interval(Evt,[_,Stamp]).

%%
event_set_begin_time(Evt,Stamp) :-
  rdf(Evt,dul:hasTimeInterval,I,G),
  rdf_retractall(I,ease:hasIntervalBegin,_),
  rdf_assert_prolog(I,ease:hasIntervalBegin,Stamp,G).

%%
event_set_end_time(Evt,Stamp) :-
  rdf(Evt,dul:hasTimeInterval,I,G),
  rdf_retractall(I,ease:hasIntervalEnd,_),
  rdf_assert_prolog(I,ease:hasIntervalEnd,Stamp,G).

%%
event_participant(Action,Participant,Class) :-
  rdf_has(Action,dul:hasParticipant,Participant),
  rdfs_individual_of(Participant,Class).
