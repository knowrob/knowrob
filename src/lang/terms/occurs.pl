:- module(lang_occurs,
    [ occurs(r),             % ?Event
      op(1000, xf, occurs)
    ]).
/** <module> The *occurs* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/DUL/Region'),
        [ time_interval_start/2, time_interval_end/2 ]).
:- use_module(library('model/DUL/Event'),
        [ has_time_interval/2 ]).
:- use_module(library('lang/scopes/temporal'),
        [ time_scope_data/2 ]).

%% occurs(?Event) is nondet.
%
% True for all occurences (events).
%
occurs(Evt) +?>
  `unscope`(time,TimeScope),
  is_event(Evt),
  { time_scope_data(TimeScope,[Since,Until]) },
  { has_time_interval(Evt,IntervalEntity) },
  occurs_since(IntervalEntity,Since),
  occurs_until(IntervalEntity,Until),
  `scope`(time,TimeScope).

%%
occurs_since(Interval,Since) ?>
  { var(Since), ! },
  { time_interval_start(Interval,Since) }.

occurs_since(Interval,Since) ?>
  { time_interval_start(Interval,X) },
  { X =< Since }.

occurs_since(Interval,Since) +>
  holds(Interval,ease:hasIntervalBegin,Since).

%%
occurs_until(Interval,Until) ?>
  { var(Until), ! },
  { time_interval_end(Interval,Until) }.

occurs_until(Interval,Until) ?>
  { time_interval_end(Interval,X) },
  { X >= Until }.

occurs_until(Interval,Until) +>
  holds(Interval,ease:hasIntervalEnd,Until).
