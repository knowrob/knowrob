:- module(lang_temporal,
	[ during(t,r),
	  since(t,r),
	  until(t,r)
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

:- op(800, yfx, user:during).
:- op(800, yfx, user:since).
:- op(800, yfx, user:until).

%% during(+Statement,?Time) is nondet.
%
% True for statements that hold during the whole
% duration of some time interval.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
during(Query, [Since,Until]) ?>
	number(Since),
	number(Until),
	call(Query, _{
		time: _{ since: =<(Since), until: >=(Until) }
	}).

during(Goal, [Since,Until]) ?>
	var(Since),
	var(Until),
	% Note: goal must be called with "wildcard" scope to include all records.
	%       the default mode is to only include records that are still true.
	call(Goal, _{
		time: _{ since:  >=(double(0)), until: =<(double('Infinity')) }
	}),
	% FIXME this is not really accurate as get('v_scope')
	% yields the accumulated scope so far.
	% but we only want the accumulated scope in Query
	% here.
	% SOLUTION: do the get within the *call*
	set(Since, string('$v_scope.time.since')),
	set(Until, string('$v_scope.time.until')).

during(Query, [Since, Until]) +>
	call(Query, _{
		time: _{ since: =(Since), until: =(Until) }
	}).

%% since(+Statement,?Interval) is nondet.
%
% True for statements that hold (at least) since some time
% instant.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
since(Query, Time) ?>
	number(Time),
	call(Query, _{
		time: _{ since: =<(Time) }
	}).

since(Query, Time) ?>
	var(Time),
	call(Query),
	set(Time, string('$v_scope.time.since')).

since(Query, Time) +>
	call(Query, _{
		time: _{ since: =(Time) }
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
	call(Query),
	set(Time, string('$v_scope.time.until')).

until(Query, Time) +>
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


test('tell during') :-
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

test('tell Lea hasNumber overlapping') :-
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

test('Lea not hasNumber during') :-
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

test('Lea hasNumber during X') :-
	assert_true(ask(triple(test:'Lea', test:hasNumber, '+493455247') during _)),
	ask(triple(test:'Lea', test:hasNumber, '+493455247') during X),
	assert_equals(X,[10.0,34.0]).

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
