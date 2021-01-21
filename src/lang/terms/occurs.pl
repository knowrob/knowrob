:- module(lang_occurs,
    [ occurs(r) % ?Event
    ]).
/** <module> The occurs predicate.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xf, occurs).

%% occurs(?Event) is nondet.
%
% True for all occurences (events).
%
% @param Event an event instance.
%
occurs(Evt) ?>
	has_interval_data(Evt, Since, Until),
	% intersect fact scope with event interval
	intersect(_{
		time: _{ since: Since, until: Until }
	}).

occurs(Evt) +>
	% read time scope provided by e.g. during/2
	% as in `occurs(Evt) during [Since,Until]`
	% from compile context.
	% TODO: this will be [0,inf] if not provided? is this a problem?
	context(scope(_{
		time: _{ since: =(Since), until: =(Until) }
	})),
	% call tell with universal scope
	call(
		[ is_event(Evt),
		  has_interval_data(Evt, Since, Until))
		],
		_{ time: _{
			since: =(double(0)),
			until: =(double('Infinity'))
		}}
	).

     /*******************************
     *        UNIT TESTS            *
     *******************************/

:- begin_rdf_tests(
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

:- end_rdf_tests(lang_occurs).
