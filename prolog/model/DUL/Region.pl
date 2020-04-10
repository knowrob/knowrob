
:- module(model_DUL_Region,
    [
      is_region(r),
      is_time_interval(r),
      is_space_region(r),
      is_amount(r),
      is_physical_attribute(r),
      is_social_attribute(r),
      is_composite_region(r),
      region_data_value(r,?),
      %%
      time_interval_tell(r,+,+),
      time_interval_data(r,?,?),
      time_interval_duration(r,?),
      time_interval_equal(r,r),
      time_interval_start(r,?),
      time_interval_end(r,?)
    ]).
:- rdf_module.
/** <module> DUL notion of Region.

In DUL, Region is defined as:
  "Any region in a dimensional space (a dimensional space is a maximal Region), which can be used as a value for a quality of an Entity."

@author Daniel Beßler
@license BSD
*/

%% is_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
is_region(Entity) :-
  ask( Region rdf:type dul:'Region' ).

%% is_space_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SpaceRegion'.
%
% @param Entity An entity IRI.
%
is_space_region(Entity) :-
  ask( Region rdf:type dul:'SpaceRegion' ).

%% is_amount(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Amount'.
%
% @param Entity An entity IRI.
%
is_amount(Entity) :-
  ask( Region rdf:type dul:'Amount' ).

%% is_physical_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PhysicalAttribute'.
%
% @param Entity An entity IRI.
%
is_physical_attribute(Entity) :-
  ask( Region rdf:type dul:'PhysicalAttribute' ).

%% is_social_attribute(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SocialAttribute'.
%
% @param Entity An entity IRI.
%
is_social_attribute(Entity) :-
  ask( Region rdf:type dul:'SocialAttribute' ).

%% is_time_interval(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'TimeInterval'.
%
% @param Entity An entity IRI.
%
is_time_interval(Entity) :-
  ask( Entity rdf:type dul:'TimeInterval' ).

%% is_composite_region(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Region'
% with some sub-region.
%
% @param Entity An entity IRI.
%
is_composite_region(Entity) :-
  is_region( Entity ),
  ask( Entity dul:hasPart SubRegion ), !.

%%
%
region_data_value(Region,DataValue) :-
  ask( Entity dul:hasRegionDataValue DataValue ).

%% time_interval_data(+In,-Out) is semidet.
%
% True if Out is the interval of In.
%
% @param In Time point, interval or temporally extended entity
% @param Out Start and end time of the interval
% 
time_interval_data(TimeInterval, Begin, End) :-
  get_time_interval(TimeInterval,X),!,
  ignore( ask( TimeInterval ease:hasIntervalBegin Begin )),
  ignore( ask( TimeInterval ease:hasIntervalEnd End )).

%% 
%
time_interval_tell(TimeInterval,Begin,End) :-
  get_time_interval(TimeInterval,X),!,
  ( ground(Begin) ->
    tell( X ease:hasIntervalBegin Begin ) ;
    true
  ),
  ( ground(End) ->
    tell( X ease:hasIntervalEnd End ) ;
    true
  ).

%%
%
get_time_interval(TimeInterval,TimeInterval) :-
  is_time_interval(TimeInterval),!.

get_time_interval(Event,TimeInterval) :-
  is_event(Event),!,
  ask( Event dul:hasTimeInterval TimeInterval ).

get_time_interval(Situation,TimeInterval) :-
  is_situation(Situation),!,
  ask( Situation dul:includesTime TimeInterval ).
  

%% interval_equal(?I1,?I2) is semidet.
%
% Interval I1 is equal to I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
time_interval_equal(TimeInterval1,TimeInterval2) :-
   time_interval_data(TimeInterval1, Begin, End),
   time_interval_data(TimeInterval2, Begin, End).

%%  interval_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
% @tbd Duration should be literal(type(qudt:'MinuteTime', Duration))
%
time_interval_duration(TimeInterval, Duration) :-
  time_interval_data(TimeInterval, Begin, End),
  ground([Begin, End]),
  Duration is (End-Begin).

%% interval_start(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
time_interval_start(TimeInterval, Begin) :-
  time_interval_data(TimeInterval, Begin, _).

%% interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
time_interval_end(TimeInterval, End) :-
  time_interval_data(TimeInterval, _, End).
