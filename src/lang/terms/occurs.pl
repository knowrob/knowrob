:- module(lang_occurs,
    [ occurs(r) % ?Event
    ]).
/** <module> The occurs predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('comm/notify'), [ notify_synchronize/1 ]).

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
  { notify_synchronize(event(Evt)) },
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

     /*******************************
     *        UNIT TESTS            *
     *******************************/

:- begin_tripledb_tests(
    lang_occurs,
    'package://knowrob/owl/test/events.owl',
    [ namespace('http://knowrob.org/kb/test_events.owl#')
    ]).

test('tell and ask event occurs during a time interval') :-
  assert_true(occurs(test:'Short4') during [1377777009, 1377777011]),
  assert_true(occurs(test:'Short1') during test:'Time_Short1'),
  assert_false(occurs(test:'Event6') during test:'Time_Long'),
  assert_true(tell(occurs(test:'Event6') during test:'Time_Long')),
  assert_true(occurs(test:'Event6') during test:'Time_Long').

test('tell and ask an event occurs during an event') :-
  assert_true(tell(holds(test:'Event6',dul:'hasTimeInterval', test:'Time_Long'))),
  assert_true(tell(occurs(test:'Event0') during test:'Event6')),
  assert_true(occurs(test:'Event0') during test:'Event6').

:- end_tripledb_tests(lang_occurs).
