
:- module(model_DUL_Event,
    [
      is_event(r),
      is_action(r),
      is_process(r),
      event_time(r,?,?),
      event_is_active(r),
      event_is_active(r,+),
      event_begin_time(r,?),
      event_end_time(r,?),
      event_set_begin_time(r),
      event_set_begin_time(r,?),
      event_set_end_time(r),
      event_set_end_time(r,?),
      event_add(r,r),
      event_participant(r,r,r),
      action_has_task(r,r),
      action_set_task(r,r)
    ]).
:- rdf_module.
/** <module> DUL notion of Event.

In DUL, Object is defined as:
  "Any physical, social, or mental process, event, or state."

@author Daniel Beßler
@license BSD
*/

%% is_event(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Event'.
%
% @param Entity An entity IRI.
%
is_event(Entity) :-
  ask( Entity rdf:type dul:'Event' ).

%% is_action(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Action'.
%
% @param Entity An entity IRI.
%
is_action(Entity) :-
  ask( Entity rdf:type dul:'Action' ).

%% is_process(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Process'.
%
% @param Entity An entity IRI.
%
is_process(Entity) :-
  ask( Entity rdf:type dul:'Process' ).

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
  time_interval_data(Evt,BeginTime,EndTime).

%% event_begin_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its begin time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the begin of the event.
%
event_begin_time(Evt,Stamp) :-
  time_interval_start(Evt,Stamp).

%% event_end_time(+Evt,?Stamp) is semidet.
%
% Maps an event to its end time, if any.
%
% @param Evt An individual of type dul:'Event'.
% @param Stamp The timestamp indicating the end of the event.
%
event_end_time(Evt,Stamp) :-
  time_interval_end(Evt,Stamp).

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
  time_interval_tell(Evt,Stamp,_).

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
  time_interval_tell(Evt,_,Stamp).

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
  ask( Evt         dul:hasParticipant Participant ),
  ask( Participant rdf:type           Class ).

%% event_add(+Act,+Filler) is semidet.
%
% Asserts that some object or region was involved
% in an action.
% This does not assert what role the object played,
% or what the parameter associated to this region is.
%
% @param Act An individual of type dul:'Action'.
% @param Filler An individual of type dul:'Object' or dul:'Region'.
%
event_add(Act,X) :-
  is_reqion(X),!,
  tell( Act dul:hasRegion X ).

event_add(Act,X) :-
  is_object(X),!,
  tell( Act dul:hasParticipant X ).

event_add(_Action,X) :-
  print_message(warning,
    model_error(not_a_region_or_object(X))),
  fail.

%% action_has_task(?Act,?Tsk) is semidet.
%
% Relates an action to the task that it executes.
% Note that this relations may not be known, e.g. in case
% of observing another agent performing an action.
% In such a case this predicate fails.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_has_task(Act,Tsk) :-
  ask( Act dul:executesTask Tsk ).

%% action_set_task(+Act,+Tsk) is semidet.
%
% Asserts the task that is executed by an action.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_set_task(Act,Tsk) :-
  tell( Act dul:executesTask Tsk ).
