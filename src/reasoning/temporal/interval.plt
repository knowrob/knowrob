
:- begin_tests(time_interval).

:- use_module(library('db/tripledb'),
	[ tripledb_load/2 ]).
:- use_module('interval.pl').

:- tripledb_load(
	'package://knowrob/owl/test/events.owl',
	[ graph(user),
	  namespace(test_events,'http://knowrob.org/kb/test_events.owl#')
	]).

/********************************
 *	    QUANTITATIVE INPUT     	*
 ********************************/

test('d(Short1,Long)') :-
	interval_during(test_events:'Short1', test_events:'Long').
test('d(Short2,Long)') :-
	interval_during(test_events:'Short2', test_events:'Long').
test('d(Short3,Long)') :-
	interval_during(test_events:'Short3', test_events:'Long').
test('d(Short4,Long)', [fail]) :-
	interval_during(test_events:'Short4', test_events:'Long').

test('s(Short1,Long)') :-
	interval_starts(test_events:'Short1', test_events:'Long').
test('si(Long,Short1)') :-
	interval_started_by(test_events:'Long', test_events:'Short1').
test('s(Short2,Long)', [fail]) :-
	interval_starts(test_events:'Short2', test_events:'Long').

test('f(Short3,Long)') :-
	interval_finishes(test_events:'Short3', test_events:'Long').
test('fi(Long,Short3)') :-
	interval_finished_by(test_events:'Long', test_events:'Short3').
test('f(Short2,Long)', [fail]) :-
	interval_finishes(test_events:'Short2', test_events:'Long').

test('m(Short1,Short2)') :-
	interval_meets(test_events:'Short1', test_events:'Short2').
test('mi(Short2,Short1)') :-
	interval_met_by(test_events:'Short2', test_events:'Short1').
test('m(Short2,Short3)', [fail]) :-
	interval_meets(test_events:'Short2', test_events:'Short3').

test('o(Long,Short4)') :-
	interval_overlaps(test_events:'Long', test_events:'Short4').
test('oi(Short4,Long)') :-
	interval_overlapped_by(test_events:'Short4', test_events:'Long').
test('o(Short2,Long)', [fail]) :-
	interval_overlaps(test_events:'Short2', test_events:'Long').

test('<(Short2,Short3)') :-
	interval_before(test_events:'Short2', test_events:'Short3').
test('<(Short1,Short3)') :-
	interval_before(test_events:'Short1', test_events:'Short3').
test('<(Short3,Short4)', [fail]) :-
	interval_before(test_events:'Short3', test_events:'Short4').
% NOTE: below fails because the events meet each other
% TODO: I think it is a bit counter-intuitive that < does not include m.
test('<(Short1,Short2)', [fail]) :-
	interval_before(test_events:'Short1', test_events:'Short2').

test('>(Short3,Short2)') :-
	interval_after(test_events:'Short3', test_events:'Short2').
test('>(Short3,Short1)') :-
	interval_after(test_events:'Short3', test_events:'Short1').
test('>(Short4,Short3)', [fail]) :-
	interval_after(test_events:'Short2', test_events:'Short3').


/********************************
 *	    QUALITATIVE INPUT     	*
 ********************************/

test('s(Event0,Event1)') :-
	interval_starts(test_events:'Event0', test_events:'Event1').
test('s(Event1,Event2)', [fail]) :-
	interval_starts(test_events:'Event1', test_events:'Event2').

test('m(Event1,Event2)') :-
	interval_meets(test_events:'Event1', test_events:'Event2').
test('m(Event1,Event3)') :-
	interval_meets(test_events:'Event1', test_events:'Event3').
test('m(Event2,Event4)', [fail]) :-
	interval_meets(test_events:'Event2', test_events:'Event4').

test('=(Event2,Event3)') :-
	interval_equals(test_events:'Event2', test_events:'Event3').
test('=(Event1,Event0)', [fail]) :-
	interval_equals(test_events:'Event1', test_events:'Event0').

test('o(Event2,Event4)') :-
	interval_overlaps(test_events:'Event2', test_events:'Event4').
test('o(Event3,Event4)') :-
	interval_overlaps(test_events:'Event3', test_events:'Event4').
test('o(Event0,Event1)', [fail]) :-
	interval_overlaps(test_events:'Event0', test_events:'Event1').

test('d(Event5,Event4)') :-
	interval_during(test_events:'Event5', test_events:'Event4').
test('d(Event0,Event1)', [fail]) :-
	interval_during(test_events:'Event0', test_events:'Event1').

test('<(Event0,Event2)') :-
	interval_before(test_events:'Event0', test_events:'Event2').
test('<(Event0,Event3)') :-
	interval_before(test_events:'Event0', test_events:'Event3').
test('<(Event0,Event4)') :-
	interval_before(test_events:'Event0', test_events:'Event4').
test('<(Event0,Event5)') :-
	interval_before(test_events:'Event0', test_events:'Event5').
test('<(Event0,Event1)', [fail]) :-
	interval_before(test_events:'Event0', test_events:'Event1').

test('<(Event1,Event4)') :-
	interval_before(test_events:'Event1', test_events:'Event4').
test('<(Event1,Event5)') :-
	interval_before(test_events:'Event1', test_events:'Event5').
test('<(Event1,Event0)', [fail]) :-
	interval_before(test_events:'Event1', test_events:'Event0').
% NOTE: below fails because the events meet each other
test('<(Event1,Event2)', [fail]) :-
	interval_before(test_events:'Event1', test_events:'Event2').
test('<(Event1,Event3)', [fail]) :-
	interval_before(test_events:'Event1', test_events:'Event3').

/********************************
 *	    MIXED INPUT	     		*
 ********************************/

test('<(Event6,Short3)') :-
	interval_before(test_events:'Event6', test_events:'Short3').
test('<(Event7,Short3)') :-
	interval_before(test_events:'Event7', test_events:'Short3').

test('<(Event6,Short4)', [ blocked('mixing of quantitative+qualitative input not yet supported.') ]) :-
	interval_before(test_events:'Event6', test_events:'Short4').
test('<(Event7,Short4)', [ blocked('mixing of quantitative+qualitative input not yet supported.') ]) :-
	interval_before(test_events:'Event7', test_events:'Short4').

/********************************
 *	    DYNAMIC ASSERTIONS     	*
 ********************************/

% TODO: test that adding some axiom will merge ESGs

:- end_tests(time_interval).
