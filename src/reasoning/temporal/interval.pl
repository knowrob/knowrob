:- module(time_interval,
    [ interval_constraint(r,t),
      interval_query(t),
      interval_equal(r,r)          => ease:simultaneous,
      interval_before(r,r)         => ease:before,
      interval_after(r,r)          => ease:after,
      interval_meets(r,r)          => ease:meets,
      interval_met_by(r,r)         => ease:metBy,
      interval_starts(r,r)         => ease:starts,
      interval_started_by(r,r)     => ease:startedBy,
      interval_finishes(r,r)       => ease:finishes,
      interval_finished_by(r,r)    => ease:finishedBy,
      interval_overlaps(r,r)       => ease:overlappedOn,
      interval_overlapped_by(r,r)  => ease:overlappedBy,
      interval_during(r,r)         => ease:during
    ]).
/** <module> Allen calculus implementation using Event Endpoint Graphs (ESGs).

@author Daniel Beßler
@license BSD
*/

% TODO: scope is ignored here entirely.
%        - time scope does not make sense
%        - what about confidence etc?

%%
%
allen_symbol(=).
allen_symbol(<).
allen_symbol(>).
allen_symbol(m).
allen_symbol(mi).
allen_symbol(o).
allen_symbol(oi).
allen_symbol(s).
allen_symbol(si).
allen_symbol(f).
allen_symbol(fi).
allen_symbol(d).

%%
%
interval_constraint(Resource,Constraint) :-
  interval_constraint(Resource,Constraint,_).

interval_constraint(Resource,Constraint,Other) :-
  % TODO: this seems a bit hacky
  tripledb_ask(Resource,Relation,Other),
  tripledb_ask(Relation,ease:symbol,Symbol),
  once(allen_symbol(Symbol)),
  Constraint =.. [Symbol,Resource,Other].

%%
%
interval_query(  =(A,B) ) :- interval_equals(A,B).
interval_query(  <(A,B) ) :- interval_before(A,B).
interval_query(  >(A,B) ) :- interval_before(B,A).
interval_query(  m(A,B) ) :- interval_meets(A,B).
interval_query( mi(A,B) ) :- interval_meets(B,A).
interval_query(  o(A,B) ) :- interval_overlaps(A,B).
interval_query( oi(A,B) ) :- interval_overlaps(B,A).
interval_query(  s(A,B) ) :- interval_starts(A,B).
interval_query( si(A,B) ) :- interval_starts(B,A).
interval_query(  f(A,B) ) :- interval_finishes(A,B).
interval_query( fi(A,B) ) :- interval_finishes(B,A).
interval_query(  d(A,B) ) :- interval_during(A,B).

%% interval_before(I0,I1) is semidet.
%
%  Interval I0 takes place before I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_before(I0, I1, _Scope) :-
  time_interval_data(I0,  _, E0),
  time_interval_data(I1, B1, _),
  ground([E0,B1]),
  E0 < B1.

interval_before(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, <(+(I0),-(I1))).

%% interval_after(I0,I1) is semidet.
%
% Interval I0 takes place after I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_after(I0,I1,Scope) :-
  interval_before(I1,I0,Scope).

%% interval_meets(I0,I1) is semidet.
%
% Intervals I0 and I1 meet, i.e. the end time of I0 is equal to the start time of I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_meets(I0, I1, _Scope) :-
  time_interval_data(I0,  _, E0),
  time_interval_data(I1, B1, _),
  ground([E0,B1]),
  E0 is B1.

interval_meets(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, =(+(I0),-(I1))).

%% interval_met_by(I1,I2) is semidet.
%
% Intervals I1 and I2 meet, i.e. the end time of I2 is equal to the start time of I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_met_by(I1,I2,Scope) :-
   interval_meets(I2,I1,Scope).

%% interval_starts(I0,I1) is semidet.
%
% Interval I0 starts interval I1, i.e. both have the same start time, but I0 finishes earlier
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_starts(I0, I1, _Scope) :-
  time_interval_data(I0, B0, E0),
  time_interval_data(I1, B1, E1),
  ground([B0,B1,E0,E1]),
  B0 is B1, E0 < E1.

interval_starts(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, =(-(I0),-(I1))),
  esg_query_(ESG, <(+(I0),+(I1))).

%% interval_started_by(I1,I2) is semidet.
%
% Interval I2 starts interval I1, i.e. both have the same start time, but I2 finishes earlier
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_started_by(I1,I2,Scope) :-
   interval_starts(I2,I1,Scope).

%% interval_finishes(I0,I1) is semidet.
%
% Interval I0 finishes interval I1, i.e. both have the same end time, but I0 starts later
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_finishes(I0, I1, _Scope) :-
  time_interval_data(I0, B0, E0),
  time_interval_data(I1, B1, E1),
  ground([B0,B1,E0,E1]),
  B0 > B1, E0 is E1.

interval_finishes(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, =(+(I0),+(I1))),
  esg_query_(ESG, <(-(I1),-(I0))).

%% interval_finished_by(I1,I2) is semidet.
%
% Interval I2 finishes interval I1, i.e. both have the same end time, but I2 starts later
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_finished_by(I1,I2,Scope):-
   interval_finishes(I2,I1,Scope).

%% interval_overlaps(I0,I1) is semidet.
%
% Interval I0  overlaps temporally with I1
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
% 
interval_overlaps(I0, I1, _Scope) :-
  time_interval_data(I0, B0, E0),
  time_interval_data(I1, B1, E1),
  ground([B0,B1,E0,E1]),
  B0 < B1, B1 < E0, E0 < E1.

interval_overlaps(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, <(-(I0),-(I1))),
  esg_query_(ESG, <(-(I1),+(I0))),
  esg_query_(ESG, <(+(I0),+(I1))).

%% interval_overlapped_by(I1,I2) is semidet.
%
% Interval I2  overlaps temporally with I1
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
interval_overlapped_by(I1,I2,Scope) :-
   interval_overlaps(I2,I1,Scope).

%% interval_during(I0,I1) is semidet.
%
% Interval I0 is inside interval I1, i.e. it starts later and finishes earlier than I1.
%
% @param I0 Time point, interval or temporally extended entity
% @param I1 Time point, interval or temporally extended entity
%
interval_during(I0, I1, _Scope) :-
  time_interval_data(I0, B0, E0),
  time_interval_data(I1, B1, E1),
  ground([B0,B1,E0,E1]),
  B1 =< B0,
  E0 =< E1.

interval_during(I0, I1, _Scope) :-
  get_esg_(I0,ESG),
  get_esg_(I1,ESG),
  esg_query_(ESG, <(-(I1),-(I0))),
  esg_query_(ESG, <(+(I0),+(I1))).

%%
% Answer an event endpoint query using an ESG.
%
esg_query_(ESG, <(E0,E1)) :-
  % <(E0,E1) holds iff there is any path from
  % E0 to E1 in the ESG.
  % TODO: think about ways how pathfinding can be optimized
  esg_endpoint_node(ESG,E0,N0),
  esg_endpoint_node(ESG,E1,N1),
  N0 \= N1,
  esg_path(ESG,N0,N1,_),!.

esg_query_(ESG, =(E0,E1)) :-
  % =(E0,E1) holds iff E0 and E1 are part of the same ESG node.
  % FIXME: being part of the same node really always means *equal*?
  %        I think rather that either of them could occur first,
  %        but not necessary at the same time.
  %        - ESG might need to distinguish between unknown and equal!
  %esg_endpoint_node(ESG,E0,N),
  %esg_endpoint_node(ESG,E1,N),
  fail.

%%
%
get_esg_(Evt,ESG) :-
  % FIXME: once generated ESGs are never updated.
  %           each new allen axiom inferred/asserted should notify us to update ESGs
  %           - use notify to trigger update?
  esg_cache_(Evt,ESG) -> true ;
  esg_gen_(Evt,ESG).

esg_gen_(Entity,ESG) :-
  var(ESG),
  esg_unique_id(ESG),
  esg_gen_2_(Entity,ESG,[]).

esg_gen_2_(Entity,_ESG,Visited) :-
  member(Entity,Visited),!.
  
esg_gen_2_(Entity,ESG,Visited) :-
  push_event_endpoints(ESG,Entity),
  forall(
    ( interval_constraint(Entity,Constraint,Other) ),
    ( esg_gen_2_(Other,ESG,[Entity|Visited]),
      push_allen_constraint(ESG,Constraint) )
  ),
  assertz(esg_cache_(Entity,ESG)).
