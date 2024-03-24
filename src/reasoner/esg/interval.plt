:- use_module(library('rdf_test')).
:- begin_rdf_tests('time_interval', 'owl/test/events.owl').

:- use_module('interval.pl').

:- use_module(library('semweb'), [ sw_register_prefix/2 ]).
:- sw_register_prefix(test, 'http://knowrob.org/kb/test_events.owl#').

/********************************
 *	    QUANTITATIVE INPUT     	*
 ********************************/

test('d(Short1,Long)') :-
	assert_true(interval_during(test:'Short1', test:'Long')).
test('d(Short2,Long)') :-
	assert_true(interval_during(test:'Short2', test:'Long')).
test('d(Short3,Long)') :-
	assert_true(interval_during(test:'Short3', test:'Long')).
test('d(Short4,Long)') :-
	assert_false(interval_during(test:'Short4', test:'Long')).

test('s(Short1,Long)') :-
	assert_true(interval_starts(test:'Short1', test:'Long')).
test('si(Long,Short1)') :-
	assert_true(interval_started_by(test:'Long', test:'Short1')).
test('s(Short2,Long)') :-
	assert_false(interval_starts(test:'Short2', test:'Long')).

test('f(Short3,Long)') :-
	assert_true(interval_finishes(test:'Short3', test:'Long')).
test('fi(Long,Short3)') :-
	assert_true(interval_finished_by(test:'Long', test:'Short3')).
test('f(Short2,Long)') :-
	assert_false(interval_finishes(test:'Short2', test:'Long')).

test('m(Short1,Short2)') :-
	assert_true(interval_meets(test:'Short1', test:'Short2')).
test('mi(Short2,Short1)') :-
	assert_true(interval_met_by(test:'Short2', test:'Short1')).
test('m(Short2,Short3)') :-
	assert_false(interval_meets(test:'Short2', test:'Short3')).

test('o(Long,Short4)') :-
	assert_true(interval_overlaps(test:'Long', test:'Short4')).
test('oi(Short4,Long)') :-
	assert_true(interval_overlapped_by(test:'Short4', test:'Long')).
test('o(Short2,Long)') :-
	assert_false(interval_overlaps(test:'Short2', test:'Long')).

test('<(Short2,Short3)') :-
	assert_true(interval_before(test:'Short2', test:'Short3')).
test('<(Short1,Short3)') :-
	assert_true(interval_before(test:'Short1', test:'Short3')).
test('<(Short3,Short4)') :-
	assert_false(interval_before(test:'Short3', test:'Short4')).
% NOTE: below fails because the events meet each other
test('<(Short1,Short2)') :-
	assert_false(interval_before(test:'Short1', test:'Short2')).

test('>(Short3,Short2)') :-
	assert_true(interval_after(test:'Short3', test:'Short2')).
test('>(Short3,Short1)') :-
	assert_true(interval_after(test:'Short3', test:'Short1')).
test('>(Short4,Short3)') :-
	assert_false(interval_after(test:'Short2', test:'Short3')).

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QUALITATIVE INPUT
%%%%%%%%%%%%%%%%%%%%%%%%%%

test('s(Event0,Event1)') :-
	assert_true(interval_starts(test:'Event0', test:'Event1')).
test('s(Event1,Event2)') :-
	assert_false(interval_starts(test:'Event1', test:'Event2')).

test('m(Event1,Event2)') :-
	assert_true(interval_meets(test:'Event1', test:'Event2')).
test('m(Event1,Event3)') :-
	assert_true(interval_meets(test:'Event1', test:'Event3')).
test('m(Event2,Event4)') :-
	assert_false(interval_meets(test:'Event2', test:'Event4')).

test('=(Event2,Event3)') :-
	assert_true(interval_equals(test:'Event2', test:'Event3')).
test('=(Event1,Event0)') :-
	assert_false(interval_equals(test:'Event1', test:'Event0')).

test('o(Event2,Event4)') :-
	assert_true(interval_overlaps(test:'Event2', test:'Event4')).
test('o(Event3,Event4)') :-
	assert_true(interval_overlaps(test:'Event3', test:'Event4')).
test('o(Event0,Event1)') :-
	assert_false(interval_overlaps(test:'Event0', test:'Event1')).

test('d(Event5,Event4)') :-
	assert_true(interval_during(test:'Event5', test:'Event4')).
test('d(Event0,Event1)') :-
	assert_false(interval_during(test:'Event0', test:'Event1')).

test('<(Event0,Event2)') :-
	assert_true(interval_before(test:'Event0', test:'Event2')).
test('<(Event0,Event3)') :-
	assert_true(interval_before(test:'Event0', test:'Event3')).
test('<(Event0,Event4)') :-
	assert_true(interval_before(test:'Event0', test:'Event4')).
test('<(Event0,Event5)') :-
	assert_true(interval_before(test:'Event0', test:'Event5')).
test('<(Event0,Event1)') :-
	assert_false(interval_before(test:'Event0', test:'Event1')).

test('<(Event1,Event4)') :-
	assert_true(interval_before(test:'Event1', test:'Event4')).
test('<(Event1,Event5)') :-
	assert_true(interval_before(test:'Event1', test:'Event5')).
test('<(Event1,Event0)') :-
	assert_false(interval_before(test:'Event1', test:'Event0')).
% NOTE: below fails because the events meet each other
test('<(Event1,Event2)') :-
	assert_false(interval_before(test:'Event1', test:'Event2')).
test('<(Event1,Event3)') :-
	assert_false(interval_before(test:'Event1', test:'Event3')).

%%%%%%%%%%%%%%%%%%%
%% MIXED INPUT
%%%%%%%%%%%%%%%%%%%

test('<(Event6,Short3)') :-
	assert_true(interval_before(test:'Event6', test:'Short3')).
test('<(Event7,Short3)') :-
	assert_true(interval_before(test:'Event7', test:'Short3')).

test('<(Event6,Short4)', [ fixme('support mixing of quantitative+qualitative input') ]) :-
	interval_before(test:'Event6', test:'Short4').
test('<(Event7,Short4)', [ fixme('support mixing of quantitative+qualitative input') ]) :-
	interval_before(test:'Event7', test:'Short4').

:- end_tests(time_interval).
