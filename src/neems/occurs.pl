:- module(mongolog_occurs, [ occurs(r) ]).
/** <module> The occurs predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module('SOMA').

%% occurs(?Event) is nondet.
%
% True for all occurences (events).
%
% @param Event an event instance.
%
occurs(Evt) ?>
	% query event interval
	call_with_context(
		event_interval(Evt, EventBegin, EventEnd),
		[query_scope(dict{ time: dict{
			min: dict{ min: double(0),          max: double(0) },
			max: dict{ min: double('Infinity'), max: double('Infinity') }
		}})]),
	% read time interval from compile context
	context(query_scope(QScope)),
	pragma(mongolog_time_scope(QScope, Since0, Until0)),
	pragma(mng_strip_operator(Since0,_,Since1)),
	pragma(mng_strip_operator(Until0,_,Until1)),
	% only succeed if time intervals intersect each other
	max(EventBegin,Since1) =< min(EventEnd,Until1).

occurs(Evt) +>
	% read time scope provided by e.g. during/2 as in `occurs(Evt) during [Since,Until]`
	% from compile context.
	% TODO: this will be [0,inf] if not provided? is this a problem?
	context(query_scope(QScope)),
	pragma(mongolog_time_scope(QScope, Since0, Until0)),
	pragma(mng_strip_operator(Since0,_,Since1)),
	pragma(mng_strip_operator(Until0,_,Until1)),
	% call with universal scope
	call_with_context(
		[ is_event(Evt),
		  event_interval(Evt, Since1, Until1)
		],
		[query_scope(dict{ time: dict{
			min: dict{ min: double(0),          max: double(0) },
			max: dict{ min: double('Infinity'), max: double('Infinity') }
		}})]
	).

%%
during(Query, Event) ?+>
	atom(Event),
	pragma(time_scope(=<(Since), >=(Until), Scope)),
	ask(event_interval(Event, Since, Until)),
	call_with_context(Query, [query_scope(Scope)]).

since(Query, Event) ?+>
	atom(Event),
	ask(event_interval(Event, Time, _)),
	call_with_context(Query, [query_scope(dict{
		time: dict{ min: dict{ max: Time } }
	})]).

until(Query, Event) ?+>
	atom(Event),
	ask(event_interval(Event, Time, _)),
	call_with_context(Query, [query_scope(dict{
		time: dict{ max: dict{ min: Time } }
	})]).

     /*******************************
     *        UNIT TESTS            *
     *******************************/

:- use_module(library('mongolog/mongolog_test')).
:- begin_mongolog_tests(mongolog_occurs, 'owl/test/events.owl').

:- rdf_register_prefix(test, 'http://knowrob.org/kb/test_events.owl#', [force(true)]).

test('during(occurs(+),+Interval)') :-
	assert_true(occurs(test:'Short4') during [1377777009, 1377777011]),
	assert_true(occurs(test:'Short4') during [1377777010, 1377777011]),
	assert_true(occurs(test:'Short4') during [1377777001, 1377777009]),
	assert_true(occurs(test:'Short4') during [1377777011, 1377777019]),
	assert_false(occurs(test:'Short4') during [1377777012, 1377777019]),
	assert_false(occurs(test:'Short4') during [1377777001, 1377777004]).

test('assert(during(occurs(+),+Interval))') :-
	assert_false(occurs(a) during [8,12]),
	assert_true(mongolog_project(occurs(a) during [8,12])),
	assert_true(occurs(a) during [8,12]),
	assert_true(occurs(a) during [10,20]),
	assert_false(occurs(a) during [5,7]).

test('during(occurs(+),+Event)') :-
	assert_true(occurs(test:'Short1') during test:'Short1'),
	assert_true(occurs(test:'Long') during test:'Short1'),
	assert_true(occurs(test:'Long') during test:'Short3'),
	assert_true(occurs(test:'Long') during test:'Short4'),
	assert_false(occurs(test:'Short1') during test:'Short3'),
	assert_false(occurs(test:'Short1') during test:'Short4').

test('assert(during(occurs(+),+Event))') :-
	assert_true(mongolog_project(occurs(test:'Event6') during test:'Short1')),
	assert_true(occurs(test:'Event6') during test:'Short1').

:- end_mongolog_tests(mongolog_occurs).
