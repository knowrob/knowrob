:- module(lang_occurs,
    [ occurs(r) % ?Event
    ]).
/** <module> The occurs predicate.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xf, occurs).

:- use_module(library('db/scope'),
    [ universal_scope/1
    ]).
:- use_module('../scopes/temporal.pl',
    [ time_scope/3,
      time_scope_data/2,
      time_subscope_of/2
    ]).

%% occurs(?Event) is nondet.
%
% True for all occurences (events).
%
% @param Event an event instance.
%
occurs(Evt) ?>
  has_interval_data(Evt,Since,Until),
  { ground([Since,Until]) },
  query_scope(QScope),
  { time_scope(=(Since),=(Until),OccursScope),
    subscope_of(OccursScope,QScope)
  }.

occurs(Evt) +>
  fact_scope(FScope),
  { get_dict(time,FScope,TimeScope),
    universal_scope(US)
  },
  call(
    [ is_event(Evt),
      occurs1(Evt,TimeScope)
    ],
    [scope(US)]
  ).

%%
occurs1(_,TimeScope)   +> { var(TimeScope),! }.
occurs1(Evt,TimeScope) +>
  { time_scope_data(TimeScope,[Since,Until]) },
  has_interval_data(Evt,Since,Until).
