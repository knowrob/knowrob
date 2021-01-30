:- module(lang_temporal,
	[ during(t,r),
	  since(t,r),
	  until(t,r)
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('lang/scope'),
		[ time_scope/3 ]).

:- op(800, yfx, user:during).
:- op(800, yfx, user:since).
:- op(800, yfx, user:until).

%% during(+Statement,?Interval) is nondet.
%
% True iff Statement holds during the whole
% duration of a time interval.
% during/2 is defined as an operator such that
% queries can be written as `Statement during Interval`.
% The Interval is represented as 2-element list `[Since,Until]`
% where Since and Until are the interval boundaries (unix timestamp, double).
% Note that it is currently not allowed to call this predicate
% with one of the boundaries grounded and the other not.
% Either both boundaries must be ground or both variables.
% If used in *tell* expressions, during/2 will scope all
% assertions in Statement with the interval provided.
%
% @param Statement A language term.
% @param Interval A 2-element list.
%
during(Statement, [Since,Until]) ?>
	number(Since),
	number(Until),
	call(Statement, _{
		time: _{ since: =<(Since),
		         until: >=(Until) }
	}).

during(Statement, [Since,Until]) ?>
	var(Since),
	var(Until),
	% Note: goal must be called with "wildcard" scope to include all records.
	%       the default mode is to only include records that are still true.
	call(Statement, _{
		time: _{ since: >=(double(0)),
		         until: =<(double('Infinity')) }
	}),
	% FIXME this is not really accurate as get('v_scope')
	% yields the accumulated scope so far.
	% but we only want the accumulated scope for Goal here.
	% SOLUTION: do the get within the *call*
	% same for since and until.
	set(Since, string('$v_scope.time.since')),
	set(Until, string('$v_scope.time.until')).

during(Statement, [Since, Until]) +>
	call(Statement, _{
		time: _{ since: =(Since),
		         until: =(Until) }
	}).

%% since(+Statement, ?Instant) is nondet.
%
% True for statements that hold (at least) since some time
% instant, _and_ until at least the current time.
% since/2 is defined as an operator such that
% queries can be written as `Statement since Instant`.
% Instant is a unix timestamp represented as floating point number.
% If used in *tell* expressions, since/2 will scope all
% assertions in Statement with an interval that begins
% at given time instant, and whose end is not known yet.
%
% @param Statement A language term.
% @param Instant A time instant.
%
since(Statement, Instant) ?>
	number(Instant),
	pragma(get_time(Now)),
	call(Statement, _{
		time: _{ since: =<(Instant),
		         until: >=(Now) }
	}).

since(Statement, Instant) ?>
	var(Instant),
	pragma(get_time(Now)),
	call(Statement, _{
		time: _{ since: >=(0),
		         until: >=(Now) }
	}),
	% FIXME: see above during/2
	set(Instant, string('$v_scope.time.since')).

since(Statement, Instant) +>
	number(Instant),
	% FIXME: until time is set to infinity here, however, the interpretation
	%        is that we don't know yet when it ends.
	%        the problem is we cannot distinguish this from records that are known
	%        to hold forever!
	call(Statement, _{
		time: _{ since: =(Instant),
		         until: =(double('Infinity')) }
	}).

%% until(+Statement,?Interval) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
until(Query, Time) ?>
	number(Time),
	call(Query, _{
		time: _{ until: >=(Time) }
	}).

until(Query, Time) ?>
	var(Time),
	call(Query, _{
		time: _{ since:  >=(double(0)), until: =<(double('Infinity')) }
	}),
	% FIXME: see above during/2
	set(Time, string('$v_scope.time.until')).

until(Query, Time) +>
	% FIXME: tell(until) should handle when statement is known until longer
	%         e.g. chnging inf to until time?
	number(Time),
	call(Query, _{
		time: _{ until: =(Time) }
	}).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_rdf_tests(
		'lang_temporal',
		'package://knowrob/owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#')
		]).


test('during can be asserted and queried') :-
	assert_true(lang_query:tell(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during [10,34]
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during [10,34]
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during [14,24]
	)).

test('during works with overlapping scope') :-
	% assert additional interval during which a statement holds that overlaps
	% with an existing interval
	assert_true(lang_query:tell(
		triple(test:'Lea', test:hasNumber, '+493455249')
		during [44,84]
	)),
	assert_true(lang_query:tell(
		triple(test:'Lea', test:hasNumber, '+493455249')
		during [24,54]
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455249')
		during [34,44]
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455249')
		during [38,80]
	)).

test('during does not yield false records') :-
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+999999999')
		during [5,20]
	)),
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455249')
		during [12,20]
	)),
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during [5,20]
	)),
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during [34,44]
	)).

test('during handles ungrounded scope') :-
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during _
	)),
	lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+493455247')
		during X
	),
	assert_equals(X,[10.0,34.0]).


test('since can be asserted and queried') :-
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 800
	)),
	assert_true(lang_query:tell(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 800
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 800
	)),
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 1000
	)).

test('since does not yield false records') :-
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 600
	)).

%test('tell the rectangle size during a time interval') :-
%	assert_false(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) during [1593302400,1593349200]),
%	assert_true(tell(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) during [1593302400,1593349200])),
%	assert_true(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) since 1593302400),
%	assert_true(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) until 1593349200),
%	assert_true(tell(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) during [1593388800,1593435600])),
%	assert_true(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) since 1593389900),
%	assert_true(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) during [1593388900,1593434600]),
%	assert_false(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) until 1594434300),
%	assert_false(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) since 1593288900),
%	assert_false(holds(test:'RectangleBig',test:'hasHeightInMeters', 15.2) during [1592434300,1592464300]).

:- end_rdf_tests('lang_temporal').
