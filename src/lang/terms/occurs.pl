:- module(lang_occurs,
    [ occurs(r),             % ?Event
      op(1000, xf, occurs)
    ]).
/** <module> The *occurs* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/DUL/Region'),
    [ time_interval_start/2,
      time_interval_end/2
    ]).
:- use_module(library('model/DUL/Event'),
    [ has_time_interval/2
    ]).
:- use_module('../scopes/temporal.pl',
    [ time_scope_data/2
    ]).

%% occurs(?Event) is nondet.
%
% True for all occurences (events).
%
occurs(Evt) ?>
  `query-scope`(Query_Scope),
  { get_dict(time,Query_Scope,Query_TimeScope),
    time_interval_data(Evt,Since,Until),
    time_scope(Since,_,Until,Event_TimeScope),
    time_scope_satisfies(Event_TimeScope,Query_TimeScope)
  }.

occurs(Evt) +>
  `unscope`(time,TimeScope),
  is_event(Evt),
  occurs1(Evt,TimeScope),
  `scope`(time,TimeScope).

%%
occurs1(Evt,TimeScope) +> { var(TimeScope),! }.
occurs1(Evt,TimeScope) +>
  { time_scope_data(TimeScope,[Since,Until]) },
  { has_time_interval(Evt,Interval) },
  occurs_since_(Interval,Since),
  occurs_until_(Interval,Until).

%%
occurs_since_(Interval,Since) +> { var(Since), ! }.
occurs_since_(Interval,Since) +> holds(Interval,ease:hasIntervalBegin,Since).
  
%%
occurs_until_(Interval,Until) +> { var(Until), ! }.
occurs_until_(Interval,Until) +> holds(Interval,ease:hasIntervalEnd,Until).
