:- module(temporal_scope,
    [ during(r,r),
      since(r,r),
      until(r,r),
      time_scope_data/2,
      op(1000, yfx, during),
      op(1000, yfx, since),
      op(1000, yfx, until)
    ]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/
% TODO: special handling for *during Situation*?
%           - create DnS structures in the background

:- use_module(library('model/DUL/Region'),
        [ time_interval_data/3, time_interval_start/2 ]).

%% during(+Statement,?Time) is nondet.
%
% True for statements that hold during the whole
% duration of some time interval.
%
during(Query,Interval) ?>
  { var(Interval), ! },
  `call`(Query),
  `scope`(_->FScope),
  { time_scope_data(FScope,Interval) }.

during(Query,Timestamp) ?>
  { number(Timestamp), ! },
  `call`(Query, _{ time: _{ since: =<(Timestamp),
                            until: >=(Timestamp) }}).
  
during(Query,Interval) ?>
  { time_interval_data(Interval,Since,Until) },
  { ground([Since,Until]) },
  `call`(Query, _{ time: _{ since: =<(Since),
                            until: >=(Until) }}).

during(Fact,Interval) +>
  { time_interval_data(Interval,Since,Until) },
  { ground([Since,Until]) },
  `call`(Fact, _{ time: _{ since: =(Since),
                           until: =(Until) }}).

%% since(+Statement,?Time) is nondet.
%
% True for statements that hold (at least) since some time
% instant.
%
since(Query,Since) ?>
  { var(Since), ! },
  `call`(Query),
  `scope`(_->FScope),
  { time_scope_data(FScope,[Since,_]) }.

since(Query,Since) ?>
  { number(Since), ! },
  `call`(Query, _{ time: _{ since: =<(Since) }}).

since(Query,Interval) ?>
  { time_interval_start(Interval,Since) },
  { ground(Since) },
  `call`(Query, _{ time: _{ since: =<(Since) }}).

since(Fact,Interval) +>
  { time_interval_start(Interval,Since) },
  { ground(Since) },
  `call`(Fact, _{ time: _{ since: =(Since) }}).

%% until(+Statement,?Time) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
%
until(Query,Until) ?>
  { var(Until), ! },
  `call`(Query),
  `scope`(_->FScope),
  { time_scope_data(FScope,[_,Until]) }.

until(Query,Until) ?>
  { number(Until), ! },
  `call`(Query, _{ time: _{ until: >=(Until) }}).

until(Query,Interval) ?>
  { time_interval_start(Interval,Until) },
  { ground(Until) },
  `call`(Query, _{ time: _{ until: >=(Until) }}).

until(Fact,Interval) +>
  { time_interval_start(Interval,Until) },
  { ground(Until) },
  `call`(Fact, _{ time: _{ until: =(Until) }}).

%% time_scope_data(+Scope,?Interval) is det.
%
% Read since/until pair of temporal scope.
%
time_scope_data(X,Scope->_) :-
  time_scope_data(X,Scope).

time_scope_data([Since,Until],Scope) :-
  get_dict(time,Scope,X),
  ( get_dict(since,X,Since) ; Since=_ ),
  ( get_dict(until,X,Until) ; Until=_ ),
  !.

time_scope_data([Since,Until],X) :-
  ( get_dict(since,X,Since) ; Since=_ ),
  ( get_dict(until,X,Until) ; Until=_ ).
