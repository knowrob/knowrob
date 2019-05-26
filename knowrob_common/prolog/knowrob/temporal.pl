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

:- module(knowrob_temporal,
    [
      current_time/1,  % -CurrentTime
      holds/1,   % ?Predicate(?Subject,?Object)
      holds/2,   % ?Predicate(?Subject,?Object), +Interval
      holds/3,   % ?Subject, ?Predicate, ?Object
      holds/4,   % ?Subject, ?Predicate, ?Object, +Interval
      occurs/1,  % ?Event
      occurs/2,  % ?Event, ?Time      
      occurs/3,  % ?Event, ?Time, ?Type
      allen_constraint/4,
      owl_individual_of_during/3,  % ?Resource, ?Description, ?Interval
      owl_individual_of_during/2,  % ?Resource, ?Description
      owl_has_during/4,            % ?Subject, ?Predicate, ?Object, ?Interval
      rdfs_instance_of_during/3,   % ?Resource, ?Description, ?Interval
      rdfs_has_during/4,           % ?Subject, ?Predicate, ?Object, ?Interval
      rdf_triple_during/4,         % ?Predicate, ?Subject, ?Object, ?Interval
      time_term/2,
      time_between/3,
      time_between/2,
      time_later_then/2,
      time_earlier_then/2,
      interval/2,
      interval_start/2,
      interval_end/2,
      interval_before/2,
      interval_after/2,
      interval_meets/2,
      interval_starts/2,
      interval_finishes/2,
      interval_overlaps/2,
      interval_during/2,
      owl_temporal_db/2
    ]).
/** <module> Predicates for temporal reasoning in KnowRob

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).

% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:- rdf_meta rdf_triple_during(r,r,t),
            rdfs_instance_of_during(r,r,t),
            owl_individual_of_during(r,r),
            owl_individual_of_during(r,r,?),
            owl_has_during(r,r,t,?),
            rdfs_has_during(r,r,t,?),
            holds(t),
            holds(t,?),
            holds(r,r,t),
            holds(r,r,t,?),
            occurs(r),
            occurs(r,?),
            occurs(r,?,r),
            allen_constraint(r,r,r,t),
            interval(?,?),
            interval_start(?,?),
            interval_end(?,?),
            interval_before(?,?),
            interval_after(?,?),
            interval_meets(?,?),
            interval_starts(?,?),
            interval_finishes(?,?),
            interval_overlaps(?,?),
            interval_during(?,?).

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease, 'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).

		 /*******************************
		 *	 high-level predicates		*
		 *******************************/

%% holds(+Term).
%% holds(+Term, ?Interval).
%% holds(?S, ?P, ?O).
%% holds(?S, ?P, ?O, ?Interval).
%
% True iff relation `P(S,O)` holds during Interval, and with Term=P(S,O).
% Where Interval is a TimeInterval or TimePoint individual,
% a number or a list of two numbers representing a time interval.
%
% @param Term Must be of the form: PROPERTY(SUBJECT, OBJECT).
% @param T Can be TimeInterval or TimePoint individual, a number or a list of two numbers representing a time interval.
%
holds(Term) :-
  current_time(T),
  holds(Term, [T,T]).

holds(Term, I) :-
  var(Term),
  holds(S, P, O, I),
  Term =.. [':', P, (S,O)].

holds(Term, I) :-
  nonvar(Term),
  (  Term =.. [':', Namespace, Tail]
  -> (
     Tail =.. [P,S,O],
     % unpack namespace
     rdf_current_ns(Namespace, NamespaceUri),
     atom_concat(NamespaceUri, P, PropertyUri),
     holds(S, PropertyUri, O, I)
  ) ; (
     Term =.. [P,S,O],
     holds(S, P, O, I)
  )).

holds(S, P, O) :-
  current_time(T),
  holds(S, P, O, [T,T]).

holds(S, P, O, T) :-
  atom(T), !,
  interval(T,I),
  owl_has_during(S, P, O, I).

holds(S, P, O, T) :-
  number(T), !,
  owl_has_during(S, P, O, [T,T]).

holds(S, P, O, I) :-
  owl_has_during(S, P, O, I).

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
  current_time(CurrentTime),
  occurs(Evt, CurrentTime, dul:'Event').

occurs(Evt, Interval) :-
  interval(Evt, EvtI),
  (  ground(Interval)
  -> interval_during(Interval, EvtI)
  ;  Interval = EvtI ).

occurs(Evt, Interval, Type) :-
  rdfs_individual_of(Evt, Type),
  interval(Evt, EvtI),
  (  ground(Interval)
  -> interval_during(Interval, EvtI)
  ;  Interval = EvtI ).

%%
allen_constraint(S,P,O,Term) :-
  rdf_has_prolog(P,ease:symbol,Sym),
  Term=..[Sym,S,O].

		 /*******************************
		 *	 	  OWL expansion			*
		 *******************************/

owl_temporal_db(I, db(rdfs_has_during(I),
                      rdfs_instance_of_during(I))).

%% owl_individual_of_during(?Resource,?Description) is nondet.
%% owl_individual_of_during(?Resource,?Description,+Interval) is nondet.
%
% True if Resource satisfies Description during Interval
% (or at the moment if not specified)
% according the the OWL-Description entailment rules,
% with additional computable and temporal semantics.
%
% @param Resource OWL resource identifier
% @param Description OWL class description
% @param Interval The interval during which the Resource satisfies Description
% 
owl_individual_of_during(Resource, Description) :-
  current_time(Now),
  owl_individual_of_during(Resource, Description, [Now,Now]).

owl_individual_of_during(Resource, Description, Interval) :-
  owl_temporal_db(Interval, DB),
  ( ground([Resource,Description,Interval]) ->
    once(owl_individual_of(Resource, Description, DB)) ; 
    owl_individual_of(Resource, Description, DB) ).

%% owl_has_during(?Subject, ?Predicate, ?Object, ?Interval).
%
% True if this relation is specified, or can be deduced using OWL
% inference rules, computable semantics, or temporal semantics.
%
% @param Subject OWL resource iri
% @param Predicate Property iri
% @param Object OWL resource iri or datatype value
% @param Interval The interval during which the relation holds
%
owl_has_during(S, P, O, Interval) :-
  owl_temporal_db(Interval, DB),
  ( ground([S,P,O,Interval]) ->
    once(owl_has(S, P, O, DB)) ; 
    owl_has(S, P, O, DB) ).

		 /*******************************
		 *	   Temporal semantics		*
		 *******************************/

%% rdfs_instance_of_during(?Resource, ?RDF_Type, Interval) is nondet.
%
% True if RDF_Type is the type of Resource during Interval.
%
% @param Resource RDF resource iri
% @param RDF_Type RDF type iri
% @param Interval The interval during which RDF_Type is the type of Resource
% 
rdfs_instance_of_during(Resource, RDF_Type, Interval) :-
  rdf_equal(rdf:type, Property),
  ( nonvar(RDF_Type) ->
    rdfs_subclass_of(Type, RDF_Type) ;
    Type = RDF_Type ),
  rdfs_computable_triple_during(Property, Resource, Type, Interval).

%% rdfs_has_during(?Subject, ?Predicate, ?Object, ?Interval) is nondet.
%
% True if the RDF triple store contains (Subject Predicate Object),
% or if the triple can be derived from computable predicates,
% or temporal semantics.
%
% @param Property The property iri
% @param Frame RDF resource iri
% @param Value RDF resource iri or datatype value
% @param Interval The interval during which the triple holds
%
rdfs_has_during(S, P, O, I) :- rdf_triple_during(P, S, O, I).

%% rdf_triple_during(?Property, ?Frame, ?Value, ?Interval) is nondet.
%
% True if the RDF triple store contains (Frame Property Value),
% or if the triple can be derived from computable predicates,
% or temporal semantics.
%
% @param Property The property iri
% @param Frame RDF resource iri
% @param Value RDF resource iri or datatype value
% @param Interval The interval during which the triple holds
%
rdf_triple_during(Property, Frame, Value, Interval) :-
  ground(Property),
  rdfs_computable_triple_during(Property, Frame, Value, Interval).

		 /*******************************
		 *		Time instants			*
		 *******************************/

%% current_time(-CurrentTime).
%
% Unifies CurrentTime with the current time in seconds.
%
current_time(CurrentTime) :-
  set_prolog_flag(float_format, '%.12g'),
  get_time(CurrentTime).

%% time_term(+Timepoint, -TimeTerm).
%
% TimeTerm is a numerical representation of Timepoint.
%
time_term(Timepoint, Timepoint) :-
  number(Timepoint), !.

time_term(literal(type('http://www.w3.org/2001/XMLSchema#double',X)), Timepoint) :-
  ( number(X) ->
    Timepoint = X ;
    atom_number(X, Timepoint) ), !.

time_term(literal(type('http://www.w3.org/2001/XMLSchema#float',X)), Timepoint) :-
  ( number(X) ->
    Timepoint = X ;
    atom_number(X, Timepoint) ), !.

time_term([Begin,End], [Begin_v,End_v]) :-
  time_term(Begin,Begin_v),
  time_term(End,End_v), !.

time_term([Begin], [Begin_v]) :-
  time_term(Begin,Begin_v), !.

time_term(Event, Interval) :-
  atom(Event),
  rdf_has(Event, knowrob:'startTime', Timepoint0),
  time_term(Timepoint0, Begin),
  (  rdf_has(Event, knowrob:'endTime', Timepoint1)
  -> (time_term(Timepoint1, End), Interval=[Begin,End])
  ;  Interval=[Begin]
  ), !.

% @deprecated
time_term(Timepoint, Time) :-
  atom(Timepoint),
  (  rdf_split_url(_, TimePointLocal, Timepoint),
     atom_concat('timepoint_', TimeAtom, TimePointLocal)
  -> term_to_atom(Time, TimeAtom)
  ;  (  atom_concat('timepoint_', TimepointAtom, Timepoint)
     -> term_to_atom(Time, TimepointAtom)
     ;  term_to_atom(Time, Timepoint)
     )
  ).

%% time_between(+T, +T0, +T1).
%% time_between(+T, +Interval).
%
% True iff T0 <= T <= T1, and
% Interval=[T0,T1].
%
time_between(Timeinterval, T0, T1) :-
  atom(Timeinterval),
  time_term(Timeinterval, Interval),
  time_between(Interval, T0, T1), !.

time_between([T2,T3], T0, T1) :-
  time_between(T2, T0, T1),
  time_between(T3, T0, T1), !.

time_between(T, T0, T1) :-
  not(is_list(T)), 
  time_earlier_then(T0, T),
  time_earlier_then(T, T1).

time_between(T, Timeinterval) :-
  atom(Timeinterval),
  time_term(Timeinterval , Interval),
  time_between(T, Interval), !.

time_between(T, [Begin,End]) :-
  time_between(T, Begin, End).

time_between(T, [Begin]) :-
  time_later_then(T, [Begin]).


%% time_later_then(+T0, +T1).
%
% True iff T0 >= T1
%
time_later_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_later_then_(T0_term, T1_term), !.
time_later_then_([_], [_])       :- fail.
time_later_then_([T0], [_,T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [T1])   :- T1 =< T0, !.
time_later_then_([_,T0], [_,T1]) :- T1 =< T0, !.
time_later_then_(T0, T1) :- number(T0), number(T1), T1 =< T0, !.

%% time_earlier_then(+T0, +T1).
%
% True iff T0 <= T1
%
time_earlier_then(T0, T1) :-
  time_term(T0, T0_term),
  time_term(T1, T1_term),
  time_earlier_then_(T0_term, T1_term), !.
time_earlier_then_([_], [_])       :- fail.
time_earlier_then_([T0], [_,T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [T1])   :- T0 =< T1, !.
time_earlier_then_([_,T0], [_,T1]) :- T0 =< T1, !.
time_earlier_then_(T0, T1) :- number(T0), number(T1), T0 =< T1, !.

		 /*******************************
		 *	Allen's interval algebra	*
		 *******************************/

%% interval(+In,-Out) is semidet.
%
% True if Out is the interval of In.
%
% @param In Time point, interval or temporally extended entity
% @param Out Start and end time of the interval
% 
interval(I, I) :- is_list(I), !.
interval(Time, [Time,Time]) :- number(Time), !.
interval(I, Interval) :-
  atom(I),
  rdf_has(I, knowrob:'startTime', T0),
  time_term(T0, T0_val),
  (  rdf_has(I, knowrob:'endTime', T1)
  -> (time_term(T1, T1_val), Interval=[T0_val,T1_val])
  ;  Interval=[T0_val] ), !.
interval(I, Interval) :-
  var(I),
  rdfs_individual_of(I, dul:'Event'),
  interval(I,Interval).
% @deprecated
interval(TimePoint, [Time,Time]) :-
  atom(TimePoint),
  rdfs_individual_of(TimePoint, knowrob:'TimePoint'),
  time_term(TimePoint, Time), !.

%% interval_start(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_start(I, Start_val) :-
  interval(I, Val),
  once(( Val=[Start] ; Val=[Start,_] )),
  time_term(Start, Start_val).

%% interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
interval_end(I, End_val) :-
  interval(I, [_,End]),
  time_term(End, End_val).

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

%% interval_overlaps(I0,I1) is semidet.
%
% Interval I0  overlaps temporally with I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_overlaps(I0, I1) :-
  interval(I0, I0_val),
  interval(I1, I1_val),
  interval_start(I0_val, Begin0),
  interval_start(I1_val, Begin1),
  ignore(interval_end(I0_val, End0)),
  ignore(interval_end(I1_val, End1)),
  ((var(End0), var(End1));
   (var(End0) -> Begin0 < End1;
   (var(End1) -> Begin1 < End0;
   (Begin0=<End1, End0>=Begin1)
   ))).

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
