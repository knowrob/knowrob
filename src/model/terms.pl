:- module(model_terms,
    [ occurs(r),    % ?Event
	  is_a(r,r)     % +Resource, ?Type
    ]).
/** <module> The occurs predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module('OWL').
:- use_module('SOMA').

:- op(1000, xf, occurs).

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
		[scope(_{ time: _{
			since: =(double(0)),
			until: =(double('Infinity'))
		}})]),
	% read time interval from compile context
	context(scope(_{
		time: _{ since: Since0, until: Until0 }
	})),
	pragma(mng_strip_operator(Since0,_,Since1)),
	pragma(mng_strip_operator(Until0,_,Until1)),
	% only succeed if time intervals intersect each other
	max(EventBegin,Since1) =< min(EventEnd,Until1).

occurs(Evt) +>
	% read time scope provided by e.g. during/2 as in `occurs(Evt) during [Since,Until]`
	% from compile context.
	% TODO: this will be [0,inf] if not provided? is this a problem?
	context(scope(_{
		time: _{ since: Since0, until: Until0 }
	})),
	pragma(mng_strip_operator(Since0,_,Since1)),
	pragma(mng_strip_operator(Until0,_,Until1)),
	% call tell with universal scope
	call_with_context(
		[ is_event(Evt),
		  event_interval(Evt, Since1, Until1)
		],
		[scope(_{ time: _{
			since: =(double(0)),
			until: =(double('Infinity'))
		}})]
	).

%%
lang_temporal:during(Query, Event) ?+>
	atom(Event),
	pragma(time_scope(=<(Since), >=(Until), Scope)),
	ask(event_interval(Event, Since, Until)),
	call_with_context(Query, [scope(Scope)]).

lang_temporal:since(Query, Event) ?+>
	atom(Event),
	ask(event_interval(Event, Time, _)),
	call_with_context(Query, [scope(_{
		time: _{ since: =<(Time) }
	})]).

lang_temporal:until(Query, Event) ?+>
	atom(Event),
	ask(event_interval(Event, Time, _)),
	call_with_context(Query, [scope(_{
		time: _{ until: >=(Time) }
	})]).

%% is_a(+Resource,?Type) is nondet.
%
% Wrapper around instance_of, subclass_of, and subproperty_of.
% Using this is a bit slower as an additional type check
% is needed.
% For example: `Cat is_a Animal` and `Nibbler is_a Cat`.
% 
% Note that contrary to wrapped predicates, is_a/2 requires
% the Resource to be ground.
%
% @param Resource a RDF resource
% @param Type the type of the resource
%
is_a(A,B) ?>
	ground(A),
	is_individual(A),
	!,
	instance_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_class(A),
	!,
	subclass_of(A,B).

is_a(A,B) ?>
	ground(A),
	is_property(A),
	!,
	subproperty_of(A,B).


     /*******************************
     *        UNIT TESTS            *
     *******************************/

:- begin_rdf_tests(
    model_terms,
    'package://knowrob/owl/test/events.owl',
    [ namespace('http://knowrob.org/kb/test_events.owl#')
    ]).

test('occurs(+)') :-
	assert_true(occurs(test:'Short4')),
	assert_true(occurs(test:'Long')),
	assert_false(occurs(test:'xyz')).

test('during(occurs(+),+Interval)') :-
	assert_true(occurs(test:'Short4') during [1377777009, 1377777011]),
	assert_true(occurs(test:'Short4') during [1377777010, 1377777011]),
	assert_true(occurs(test:'Short4') during [1377777001, 1377777009]),
	assert_true(occurs(test:'Short4') during [1377777011, 1377777019]),
	assert_false(occurs(test:'Short4') during [1377777012, 1377777019]),
	assert_false(occurs(test:'Short4') during [1377777001, 1377777004]).

test('tell(during(occurs(+),+Interval))') :-
	assert_false(occurs(a) during [8,12]),
	assert_true(tell(occurs(a) during [8,12])),
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

test('tell(during(occurs(+),+Event))') :-
	assert_true(tell(occurs(test:'Event6') during test:'Short1')),
	assert_true(occurs(test:'Event6') during test:'Short1').

:- end_rdf_tests(model_terms).
