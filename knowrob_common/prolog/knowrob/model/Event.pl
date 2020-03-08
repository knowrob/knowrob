
:- module('knowrob/model/Event',
    [
      kb_is_event/1,
      event_create/3,
      event_time/3,
      event_begin_time/2,
      event_end_time/2,
      event_set_begin_time/1,
      event_set_begin_time/2,
      event_set_end_time/1,
      event_set_end_time/2,
      event_participant/3
    ]).
/** <module> Interface to RDF model of events.

*Event* is defined as any physical, social, or mental process, event, or state.

More theoretically, events can be classified in different ways, possibly based on 'aspect' (e.g. stative, continuous, accomplishement, achievement, etc.), on 'agentivity' (e.g. intentional, natural, etc.), or on 'typical participants' (e.g. human, physical, abstract, food, etc.).
Here no special direction is taken, and the following explains why: events are related to observable situations, and they can have different views at a same time.
If a position has to be suggested here anyway, the participant-based classification of events seems the most stable and appropriate for many modelling problems.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- use_module(library('knowrob/lang/ask'),  [ kb_triple/3, kb_type_of/2 ]).
:- use_module(library('knowrob/lang/tell'), [ kb_assert/3, kb_retract/3 ]).

:- rdf_meta
      kb_is_event(r),
      event_create(r,r,+),
      event_time(r,?,?),
      event_begin_time(r,?),
      event_end_time(r,?),
      event_set_begin_time(r,?),
      event_set_end_time(r,?),
      event_set_begin_time(r),
      event_set_end_time(r),
      event_participant(r,r,r).

		 /*******************************
		 *	dul:'Event'		*
		 *******************************/

%% kb_is_event(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Event'.
%
% @param Entity An entity IRI.
%
kb_is_event(Entity) :-
  kb_type_of(Entity,dul:'Event'),!.

%% event_create(+EvtType,-Evt,+Graph) is semidet.
%
% Creates an event individual. Note that actions, processes,
% and states are all sub-classes of event in this model.
%
% @param EvtType A sub-class of dul:'Event'.
% @param Evt An individual of type EvtType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
event_create(EvtType,Evt,Graph) :-
  kb_create(EvtType,Evt,_{graph:Graph}),
  kb_create(dul:'TimeInterval',I,_{graph:Graph}),
  kb_assert(Evt,dul:hasTimeInterval,I,_{graph:Graph}).

%% event_time(+Evt,?BeginTime,?EndTime) is det.
%
% Maps an event to its begin and end time.
% BeginTime/EndTime remains unchanged in case no
% begin/end time is defined for the given event.
%
% @param Evt An individual of type dul:'Event'.
% @param BeginTime The timestamp indicating the begin of the event.
% @param EndTime The timestamp indicating the end of the event.
%
event_time(Evt,BeginTime,EndTime) :-
  ignore(event_begin_time(Evt,BeginTime)),
  ignore(event_end_time(Evt,EndTime)).

%% event_begin_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its begin time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the begin of the event.
%
event_begin_time(Evt,Stamp) :-
  interval_start(Evt,Stamp).

%% event_end_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its end time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the end of the event.
%
event_end_time(Evt,Stamp) :-
  interval_end(Evt,Stamp).

%% event_set_begin_time(+Evt,?Stamp) is det.
%% event_set_begin_time(+Evt) is det.
%
% Associates an event to its begin time.
% event_set_begin_time/1 sets the begin time to the current time.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the begin of the event.
%
event_set_begin_time(Evt) :-
  get_time(Stamp),
  event_set_begin_time(Evt,Stamp).

event_set_begin_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalBegin,_),
  kb_assert(I,ease:hasIntervalBegin,Stamp).


%% event_set_end_time(+Evt,?Stamp) is det.
%% event_set_end_time(+Evt) is det.
%
% Associates an event to its end time.
% event_set_end_time/1 sets the end time to the current time.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the end of the event.
%
event_set_end_time(Evt) :-
  get_time(Stamp),
  event_set_end_time(Evt,Stamp).

event_set_end_time(Evt,Stamp) :-
  kb_triple(Evt,dul:hasTimeInterval,I),!,
  kb_retract(I,ease:hasIntervalEnd,_),
  kb_assert(I,ease:hasIntervalEnd,Stamp).

%% event_participant(+Evt,?Participant,?Class) is semidet.
%
% Relates an event to a tuple of an object partipating in
% the event and its type.
%
% @param Evt An individual of type dul:'Event'.
% @param Participant An individual of type dul:'Object'.
% @param Class The most specific type of Participant.
%
event_participant(Evt,Participant,Class) :-
  kb_triple(Evt,dul:hasParticipant,Participant),
  kb_type_of(Participant,Class).
