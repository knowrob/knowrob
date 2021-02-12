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
		[ time_scope/3, universal_scope/1 ]).

:- multifile during/2.
:- multifile since/2.
:- multifile until/2.

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
	pragma(time_scope(=<(Since), >=(Until), Scope)),
	call_with_context(Statement, [scope(Scope)]).

during(Statement, [Since,Until]) ?>
	var(Since),
	var(Until),
	% Note: goal must be called with "wildcard" scope to include all records.
	%       the default mode is to only include records that are still true.
	pragma(time_scope(
		>=(0),
		=<(double('Infinity')),
		Scope
	)),
	call_with_context(Statement, [scope(Scope)]),
	% read computed fact scope
	% FIXME: this is not really accurate as get('v_scope')
	% yields the accumulated scope so far.
	% but we only want the accumulated scope for Goal here.
	% SOLUTION: do the get within the *call*
	% same for since and until.
	set(Since, string('$v_scope.time.since')),
	set(Until, string('$v_scope.time.until')).

during(Statement, [Since, Until]) +>
	pragma(time_scope(=(Since), =(Until), Scope)),
	call_with_context(Statement, [scope(Scope)]).

%% since(+Statement, ?Instant) is nondet.
%
% True for statements that hold (at least) since some time
% instant, _and_ until at least the current time.
% since/2 is defined as an operator such that
% queries can be written as `Statement since Instant`.
% Instant is a unix timestamp represented as floating point number.
% If used in *tell* expressions, since/2 will scope all
% assertions in Statement with an interval that begins
% at given time instant, and whose end is not known.
%
% @param Statement A language term.
% @param Instant A time instant.
% @todo better handling of unknown until of interval
%
since(Statement, Instant) ?>
	number(Instant),
	% get current time
	pragma(get_time(Now)),
	% only include records that hold at least since
	% instant and at least until now
	pragma(time_scope(=<(Instant), >=(Now), Scope)),
	call_with_context(Statement, [scope(Scope)]).

since(Statement, Instant) ?>
	var(Instant),
	% get current time
	pragma(get_time(Now)),
	% only include records that still are thought to be true
	pragma(time_scope(>=(0), >=(Now), Scope)),
	call_with_context(Statement, [scope(Scope)]),
	% read computed fact scope
	% FIXME: see above during/2
	set(Instant, string('$v_scope.time.since')).

since(Statement, Instant) +>
	number(Instant),
	% FIXME: until time is set to infinity here, however, the interpretation
	%        is that we don't know yet when it ends.
	%        the problem is we cannot distinguish this from records that are known
	%        to hold forever!
	pragma(time_scope(
		=(Instant),
		=(double('Infinity')),
		Scope
	)),
	call_with_context(Statement, [scope(Scope)]).

%% until(+Statement, ?Instant) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
% until/2 is defined as an operator such that
% queries can be written as `Statement until Instant`.
% Instant is a unix timestamp represented as floating point number.
% If used in *tell* expressions, until/2 updates the existing record of
% Statement known to hold at time instant if any, else it
% will create a record whose begin time is not known.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
% @todo tell until has unclear semantic
% @todo better handling of unknown since of interval
%
until(Statement, Instant) ?>
	number(Instant),
	% only include records that hold at instant
	pragma(time_scope(=<(Instant), >=(Instant), Scope)),
	call_with_context(Statement, [scope(Scope)]).

until(Statement, Instant) ?>
	var(Instant),
	% include all records
	pragma(time_scope(
		>=(double(0)),
		=<(double('Infinity')),
		Scope
	)),
	call_with_context(Statement, [scope(Scope)]),
	% FIXME: see above during/2
	set(Instant, string('$v_scope.time.until')).

until(Statement, Instant) +>
	number(Instant),
	% FIXME: since time is set to 0 here, however, the interpretation
	%        is that we don't know yet when it starts.
	%        the problem is we cannot distinguish this from records that are known
	%        to hold since begin of time!
	% TODO: better disable tell cases for until and since?
	pragma(time_scope(=(0), =(Instant), Scope)),
	call_with_context(Statement,
		[intersect_scope, scope(Scope)]).

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


test('since ask+tell') :-
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
	)),
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		since 600
	)).

test('until ask+tell') :-
	% before tell until=inf
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		until 1000
	)),
	assert_true(lang_query:tell(
		triple(test:'Lea', test:hasNumber, '+499955247')
		until 900
	)),
	% after tell until=900
	assert_true(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		until 900
	)),
	assert_false(lang_query:ask(
		triple(test:'Lea', test:hasNumber, '+499955247')
		until 1000
	)).

:- end_rdf_tests('lang_temporal').
