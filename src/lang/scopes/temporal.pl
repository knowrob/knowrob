:- module(temporal_scope,
    [ during(r,r),
      since(r,r),
      until(r,r),
      time_scope_data/2,
      time_subscope_of/2,
      time_scope_satisfies/2,
      time_scope_intersect/3,
      time_scope_merge/3,
      time_scope/5,
      op(1000, yfx, during),
      op(1000, yfx, since),
      op(1000, yfx, until)
    ]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

% TODO: currently interval data is needed for this to work.
%         it would be good if this could be handled qualitatively
%         as long as data is unknown (see esg.pl in temporal reasoning).
%         possibly use esg.pl here.

:- use_module(library('model/DUL/Region'),
    [ time_interval_data/3,
      time_interval_start/2
    ]).

%% during(+Statement,?Time) is nondet.
%
% True for statements that hold during the whole
% duration of some time interval.
%
during(Query,[Since,Until]) ?>
  { var(Interval), ! },
  `call`(Query),
  `fact-scope`(FScope),
  { time_scope_data(FScope,[Since,Until]) }.

during(Query,Time) ?>
  { number(Time), ! },
  { time_scope(Time,=<,Time,>=,Scope) },
  `call`(Query,Scope).
  
during(Query,Interval) ?>
  { time_interval_data(Interval,Since,Until) },
  { ground([Since,Until]) },
  { time_scope(Since,=<,Until,>=,Scope) },
  `call`(Fact,Scope).

during(Fact,Interval) +>
  { time_interval_data(Interval,Since,Until) },
  { ground([Since,Until]) },
  { time_scope(Since,_,Until,_,Scope) },
  `call`(Fact,Scope).

%% since(+Statement,?Time) is nondet.
%
% True for statements that hold (at least) since some time
% instant.
%
since(Query,Since) ?>
  { var(Since), ! },
  `call`(Query),
  `fact-scope`(FScope),
  { time_scope_data(FScope,[Since,_]) }.

since(Query,Time) ?>
  { number(Time), ! },
  { time_scope(Time,=<,_,_,Scope) },
  `call`(Query,Scope).

since(Query,Interval) ?>
  { time_interval_start(Interval,Since) },
  { ground(Since) },
  { time_scope(Since,=<,_,_,Scope) },
  `call`(Query,Scope).

since(Fact,Interval) +>
  { time_interval_start(Interval,Since) },
  { ground(Since) },
  { time_scope(Since,_,_,_,Scope) },
  % TODO: if there is a fact active at that time, use that until time instead.
  % 1. fact holds since <= Since
  %       -> nothing to do
  % 2. fact does not hold at Since
  %       -> add new fact without until
  `call`(Fact,Scope).

%% until(+Statement,?Time) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
%
until(Query,Until) ?>
  { var(Until), ! },
  `call`(Query),
  `fact-scope`(FScope),
  { time_scope_data(FScope,[_,Until]) }.

until(Query,Time) ?>
  { number(Time), ! },
  { time_scope(_,_,Time,>=,Scope) },
  `call`(Query,Scope).

until(Query,Interval) ?>
  { time_interval_start(Interval,Until) },
  { ground(Until) },
  { time_scope(_,_,Until,>=,Scope) },
  `call`(Query,Scope).

until(Fact,Interval) +>
  { time_interval_start(Interval,Until) },
  { ground(Until) },
  { time_scope(_,_,Until,>=,Scope) },
  % TODO: use this to stop facts active at time
  %         if none, also set since to Until
  % 1. fact holds until >= Until
  %       -> nothing to do
  % 2. fact does not hold at Since
  %       -> add new fact without until
  % FIXME: it is not allowed to leave since var, I think
  `call`(Fact,Scope).

%% time_scope_data(+Scope,?Interval) is det.
%
% Read since/until pair of temporal scope.
%
time_scope_data(Scope,[Since,Until]) :-
  ( get_dict(time,Scope,X) ; X=Scope ),
  ( get_dict(since,X,double(Since)) ; Since=_ ),
  ( get_dict(until,X,double(Until)) ; Until=_ ),
  !.

%%
%
%
time_scope_satisfies(FScope,QScope) :-
  time_scope_data(FScope,[FSince,FUntil]),
  ( get_dict(since,QScope,QSince) ->
    time_scope_satisfies1(QSince,FSince);
    true ),
  ( get_dict(until,QScope,QUntil) ->
    time_scope_satisfies1(QUntil,FUntil);
    true ),
  !.

time_scope_satisfies1(=(V),V)    :- !.
time_scope_satisfies1(=<(V0),V1) :- time_scope_min_(V0,V1,V1).
time_scope_satisfies1(>=(V0),V1) :- time_scope_max_(V0,V1,V1).
time_scope_satisfies1( <(V0),V1) :- time_scope_min_(V0,V1,V1).
time_scope_satisfies1( >(V0),V1) :- time_scope_max_(V0,V1,V1).

%%
% All timepoints in Sub are also included in Sup.
%
time_subscope_of(Sub,Sup) :-
  time_scope_data(Sub,[Sub0,Sub1]),
  time_scope_data(Sup,[Sup0,Sup1]),
  time_scope_min_(Sub0,Sup0,Sup0),
  time_scope_max_(Sub1,Sup1,Sup1).

%%
% Union is the scope containing all timepoints from
% scope A and scope B, where A and B have overlapping
% time scope (merge not possible if disjoint).
%
time_scope_merge(A,B,Merged) :-
  time_scope_data(A,[Since0,Until0]),
  time_scope_data(B,[Since1,Until1]),
  time_scope_overlaps_([Since0,Until0],
                       [Since1,Until1]),
  time_scope_min_(Since0,Since1,Since),
  time_scope_max_(Until0,Until1,Until),
  ( ground(Until) ->
    Merged=_{ since: double(Since), until: double(Until) };
    Merged=_{ since: double(Since) }
  ).

%%
% Modify A such that it only contains time instants
% that are also contained in B.
%
time_scope_intersect(A,B,Intersection) :-
  time_scope_data(A,[Since0,Until0]),
  time_scope_data(B,[Since1,Until1]),
  time_scope_max_(Since0,Since1,Since), ground(Since),
  time_scope_min_(Until0,Until1,Until),
  ( var(Until) ; Since < Until ),
  ( ground(Until) ->
    Intersection=_{ since: double(Since), until: double(Until) };
    Intersection=_{ since: double(Since) }
  ).

%%
time_scope_overlaps_([S0,U0],[S1,U1]) :-
  time_scope_inside_(S0,[S1,U1]);
  time_scope_inside_(U0,[S1,U1]).

%%
time_scope_inside_(X,[S,U]) :-
  var(X),!,
  var(U).

time_scope_inside_(X,[S,U]) :-
  X>=S,
  ( ground(U) -> X=<U ; true ).

%%
time_scope_min_('Infinity',Min,Min).
time_scope_min_(Min,'Infinity',Min).
time_scope_min_(X0,X1,X1)  :- var(X0).
time_scope_min_(X0,X1,X0)  :- var(X1).
time_scope_min_(X0,X1,Min) :- Min is min(X0,X1).

%%
time_scope_max_('Infinity',_,'Infinity').
time_scope_max_(_,'Infinity','Infinity').
time_scope_max_(X0,_,_)    :- var(X0).
time_scope_max_(_,X1,_)    :- var(X1).
time_scope_max_(X0,X1,Max) :- Max is max(X0,X1).

%%
time_scope(Since,SinceOperator,Until,_,
           _{ time: _{ since: SinceTerm }}) :-
  var(Until),!,
  get_scope1_(Since,SinceOperator,SinceTerm).

time_scope(Since,_,Until,UntilOperator,
           _{ time: _{ until: UntilTerm }}) :-
  var(Since),!,
  get_scope1_(Until,UntilOperator,UntilTerm).

time_scope(Since,SinceOperator,Until,UntilOperator,
           _{ time: _{ since: SinceTerm, until: UntilTerm }}) :-
  get_scope1_(Since,SinceOperator,SinceTerm),
  get_scope1_(Until,UntilOperator,UntilTerm).

get_scope1_(Value,Operator,Term) :-
  ( var(Operator) ->
    Term=double(Value);
    Term=..[Operator,double(Value)]
  ).
