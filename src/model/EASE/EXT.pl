
:- module(model_EASE_EXT,
    [ is_episode(r),
      %%
      time_interval_tell(r,+,+),
      time_interval_data(r,?,?),
      time_interval_duration(r,?),
      time_interval_equal(r,r),
      time_interval_start(r,?),
      time_interval_end(r,?)
      %constraint_term(r,?),
      %has_constrained_concept(r,r,r),
      %has_dependent_concept(r,r,r),
      %set_constrained_concept(r,r),
      %set_dependent_concept(r,r),
      %%%
      %building_number_of_levels(r,?),
      %building_number_of_stories(r,?), 
      %building_has_room(r,r)
     ]).

:- use_module(library('db/tripledb'),
	[ tripledb_ask/3 ]).
:- use_module(library('model/DUL/Region'),
	[ has_time_interval/2 ]).

is_episode(Entity) ?+>
  has_type(Entity, ease:'Episode').
 
		 /*******************************
		 *	    TIME INTERVALS			*
		 *******************************/

%% has_interval_data(+Term,?Since,?Until) is semidet.
%
% Associates a term to its interval data [Since,Until].
% Both are represented as Unix timestamps.
%
% @param Term language term
% @param Since begin of the interval
% @param Until end of the interval
%
has_interval_data(Term,Since,Until) ?>
	{ time_interval_data(Term,Since,Until),
	  universal_scope(US)
	},
	fact_scope(US).

has_interval_data(Term,Since,Until) +>
	{ has_time_interval(Term,Interval) },
	triple(Interval, ease:hasIntervalBegin, Since),
	triple(Interval, ease:hasIntervalEnd, Until).

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
  number(Instant),
  !.

time_interval_data(Entity, Begin, End) :-
  ( is_time_interval(Entity)
  -> Interval=Entity
  ;  has_time_interval(Entity,Interval)
  ),
  ignore(tripledb_ask(Interval, ease:hasIntervalBegin, Begin)),
  ignore(tripledb_ask(Interval, ease:hasIntervalEnd, End)),
  !.

%% 
%
time_interval_tell(Entity,Begin,End) :-
  has_time_interval(Entity,Interval),!,
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

		 /*******************************
		 *	    CONSTRAINTS		*
		 *******************************/

% TODO: e.g. Keep object that has role A
%                close to the one having role B
%                in the scope of some action

%% constraint_term(+Constraint,?Term) is semidet
%
%
%constraint_term(Constraint,Term) :-
  %ask( Constraint rdf:type   C_type ),
  %ask( C_type     rdfs:label C_label ),!,
  %%%
  %has_constrained_concept(Constraint,R0,_),
  %has_dependent_concept(Constraint,R1,_),
  %Term =.. [C_label,R0,R1].

%% has_constrained_concept(+Constraint,?Role0,?Role) is semidet
%
%
%has_constrained_concept(Constraint,Role0,Role) :-
  %ask( Constraint knowrob:constrains Role0 ),
  %ask( Role0      rdf:type           Role ).

%% has_dependent_concept(+Constraint,?Role0,?Role) is semidet
%
%
%has_dependent_concept(Constraint,Role0,Role) :-
  %ask( Constraint knowrob:dependsOnConcept Role0 ),
  %ask( Role0      rdf:type                 Role ).

%% set_constrained_concept(+Concept0,+Concept1) is semidet
%
%
%set_constrained_concept(Concept0,Concept1) :-
  %tell( Concept0 knowrob:constrains Concept1 ).

%% set_dependent_concept(+Concept0,+Concept1) is semidet
%
%
%set_dependent_concept(Concept0,Concept1) :-
  %tell( Concept0 knowrob:dependsOnConcept Concept1 ).

		 /*******************************
		 *	    BUILDING		*
		 *******************************/

%% building_number_of_levels(?Map:iri, ?N:int) is semidet
%
% True if N is the number of levels in Map.
%
% @param Map The semantic map 
% @param N The number of levels
%
%building_number_of_levels(Map, N):-
  %setof(L, ask( Map knowrob:'hasLevels' L ), Ls),
  %length(Ls, N).

%% building_number_of_stories(?Map:iri, ?N:int) is semidet
%
% True if N is the number of stories in Map.
%
% @param Map The semantic map 
% @param N The number of stories
%
%building_number_of_stories(Map, N) :-
  %setof(L, (
    %ask( Map knowrob:'hasLevels', L ),
    %ask( L   rdf:type             knowrob:'AboveGroundLevelInAConstruction' )
  %), Ls),
  %length(Ls, N).

%% building_has_room(+Map:iri, ?Room:iri) is semidet
%
% True if Room is a Room of some level in Map.
%
% @param Map The semantic map 
% @param Room The room in Map
%
%building_has_room(Map,Room) ?+>
  %holds(Map,knowrob:'hasLevels',Level),
  %holds(Level,knowrob:'hasRooms',Room).
