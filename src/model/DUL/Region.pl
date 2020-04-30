:- module(model_DUL_Region,
    [ is_region(r),
      is_parameter(r),
      is_time_interval(r),
      is_space_region(r),
      is_amount(r),
      is_physical_attribute(r),
      is_social_attribute(r),
      has_region(r,r),
      has_parameter(r,r),
      has_parameter(r,r,r),
      has_parameter_range(r,r,r),
      has_assignment(r,r),
      has_data_value(r,?),
      %%
      time_interval_tell(r,+,+),
      time_interval_data(r,?,?),
      time_interval_duration(r,?),
      time_interval_equal(r,r),
      time_interval_start(r,?),
      time_interval_end(r,?)
    ]).
/** <module> DUL notion of Region.

In DUL, Region is defined as:
  "Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity."

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS')
        [ has_type/2 ]).
:- use_module('Event'
        [ is_event/1 ]).
:- use_module('Situation'
        [ is_situation/1 ]).

%% is_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
is_region(Entity) ?+>
  has_type(Entity, dul:'Region').

%% is_parameter(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Parameter'.
%
% @param Entity An entity IRI.
%
is_parameter(Entity) ?+>
  has_type(Entity, dul:'Parameter').

%% is_space_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SpaceRegion'.
%
% @param Entity An entity IRI.
%
is_space_region(Entity) ?+>
  has_type(Entity, dul:'SpaceRegion').

%% is_amount(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Amount'.
%
% @param Entity An entity IRI.
%
is_amount(Entity) ?+>
  has_type(Entity, dul:'Amount').

%% is_physical_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PhysicalAttribute'.
%
% @param Entity An entity IRI.
%
is_physical_attribute(Entity) ?+>
  has_type(Entity, dul:'PhysicalAttribute').

%% is_social_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SocialAttribute'.
%
% @param Entity An entity IRI.
%
is_social_attribute(Entity) ?+>
  has_type(Entity, dul:'SocialAttribute').

%% is_time_interval(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'TimeInterval'.
%
% @param Entity An entity IRI.
%
is_time_interval(Entity) ?+>
  has_type(Entity, dul:'TimeInterval').

%%
%
has_region(Entity,Region) ?+>
  holds(Entity, dul:hasRegion, Region).

%%
%
has_parameter(Entity,Param) ?+>
  holds(Entity,dul:hasParameter,Param).

%%
%
has_parameter(Entity,Param,ParamType) ?>
  holds(Entity,dul:hasParameter,Param),
  has_object_type(Param,ParamType).

%%
%
has_parameter_range(Entity,Parameter,Range) ?>
  holds(Entity,dul:hasParameter,Param),
  holds(Param,dul:classifies,only(Range)).

%%
%
has_assignment(Parameter,Region) ?+>
  holds(Parameter, dul:classifies, Region).

%%
%
has_data_value(Region,DataValue) ?+>
  holds(Entity, dul:hasDataValue, DataValue).

%% time_interval_data(+In,-Out) is semidet.
%
% True if Out is the interval of In.
%
% @param In Time point, interval or temporally extended entity
% @param Out Start and end time of the interval
% 
time_interval_data([Begin, End], Begin, End) :-
  !.
time_interval_data(Instant, Instant, Instant) :-
  number(TimeInstant),
  !.
time_interval_data(Entity, Begin, End) :-
  get_time_interval_(Entity,Interval),
  % FIXME: needs to move to EASE
  % TODO: rather use interval term and only one assertion for intervals!
  ignore(holds(Interval, ease:hasIntervalBegin, Begin)),
  ignore(holds(Interval, ease:hasIntervalEnd, End)),
  !.

%% 
%
time_interval_tell(Entity,Begin,End) :-
  get_time_interval_(Entity,Interval),!,
  ( ground(Begin) ->
    tell(holds(Interval, ease:hasIntervalBegin, Begin)) ;
    true
  ),
  ( ground(End) ->
    tell(holds(Interval, ease:hasIntervalEnd, End)) ;
    true
  ).

%% interval_equal(?I1,?I2) is semidet.
%
% Interval I1 is equal to I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
time_interval_equal(Entity1,Entity2) :-
   time_interval_data(Entity1, Begin, End),
   time_interval_data(Entity2, Begin, End),
   ground([Begin,End]).

%% interval_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
% @tbd Duration should be literal(type(qudt:'MinuteTime', Duration))
%
time_interval_duration(Entity, Duration) :-
  time_interval_data(Entity, Begin, End),
  ground([Begin, End]),
  Duration is (End-Begin).

%% interval_start(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
time_interval_start(Entity, Begin) :-
  time_interval_data(Entity, Begin, _).

%% interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
time_interval_end(Entity, End) :-
  time_interval_data(Entity, _, End).


%%
get_time_interval_(TimeInterval,TimeInterval) :-
  is_time_interval(TimeInterval),!.

get_time_interval_(Event,TimeInterval) :-
  is_event(Event),!,
  holds(Event, dul:hasTimeInterval, TimeInterval).

get_time_interval_(Situation,TimeInterval) :-
  is_situation(Situation),!,
  % TODO: rather go over all included events to find time boundaries?
  %         then no need to include some time interval
  holds(Situation, dul:includesTime, TimeInterval).
