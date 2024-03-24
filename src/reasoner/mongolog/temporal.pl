:- module(mongolog_temporal,
	[ during(t,r),
	  since(t,r),
	  until(t,r)
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('scope'), [ time_scope/3 ]).

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
% If used in *project* expressions, during/2 will scope all
% assertions in Statement with the interval provided.
%
% @param Statement A language term.
% @param Interval A 2-element list.
%
during(Statement, [Since,Until]) ?>
	number(Since),
	number(Until),
	pragma(time_scope(=<(Since), >=(Until), Scope)),
	call_with_context(Statement, [query_scope(Scope)]).

during(Statement, [Since,Until]) ?>
	var(Since),
	var(Until),
	% Note: goal must be called with below scope to include all records.
	%       the default mode is to only include records true now.
	pragma(time_scope(
		>=(0),
		=<(double('Infinity')),
		Scope
	)),
	call_with_context(Statement, [query_scope(Scope)]),
	% read computed fact scope
	% FIXME: this is not really accurate as get('v_scope') yields the accumulated scope so far.
	%   but we only want the accumulated scope for Goal here.
	%   SOLUTION: do the get within the *call* same for since and until.
	assign(Since, string('$v_scope.time.since')),
	assign(Until, string('$v_scope.time.until')).

during(Statement, [Since, Until]) +>
	pragma(time_scope(=(Since), =(Until), Scope)),
	call_with_context(Statement, [query_scope(Scope)]).

%% since(+Statement, ?Instant) is nondet.
%
% True for statements that hold (at least) since some time
% instant, _and_ until at least the current time.
% since/2 is defined as an operator such that
% queries can be written as `Statement since Instant`.
% Instant is a unix timestamp represented as floating point number.
% If used in *project* expressions, since/2 will scope all
% assertions in Statement with an interval that begins
% at given time instant, and whose end is not known.
%
% @param Statement A language term.
% @param Instant A time instant.
%
since(Statement, Instant) ?>
	% TODO: better handling of unknown until of interval
	number(Instant),
	% get current time
	pragma(get_time(Now)),
	% only include records that hold at least since
	% instant and at least until now
	pragma(time_scope(=<(Instant), >=(Now), Scope)),
	call_with_context(Statement, [query_scope(Scope)]).

since(Statement, Instant) ?>
	var(Instant),
	% get current time
	pragma(get_time(Now)),
	% only include records that still are thought to be true
	pragma(time_scope(>=(0), >=(Now), Scope)),
	call_with_context(Statement, [query_scope(Scope)]),
	% read computed fact scope
	assign(Instant, string('$v_scope.time.since')).

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
	call_with_context(Statement, [query_scope(Scope)]).

%% until(+Statement, ?Instant) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
% until/2 is defined as an operator such that
% queries can be written as `Statement until Instant`.
% Instant is a unix timestamp represented as floating point number.
% If used in *project* expressions, until/2 updates the existing record of
% Statement known to hold at time instant if any, else it
% will create a record whose begin time is not known.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
until(Statement, Instant) ?>
	% TODO project until has unclear semantic
	% TODO better handling of unknown since of interval
	number(Instant),
	% only include records that hold at instant
	pragma(time_scope(=<(Instant), >=(Instant), Scope)),
	call_with_context(Statement, [query_scope(Scope)]).

until(Statement, Instant) ?>
	var(Instant),
	% include all records
	pragma(time_scope(
		>=(double(0)),
		=<(double('Infinity')),
		Scope
	)),
	call_with_context(Statement, [query_scope(Scope)]),
	assign(Instant, string('$v_scope.time.until')).

until(Statement, Instant) +>
	number(Instant),
	% FIXME: since time is set to 0 here, however, the interpretation
	%        is that we don't know yet when it starts.
	%        the problem is we cannot distinguish this from records that are known
	%        to hold since begin of time!
	% TODO: better disable project cases for until and since?
	pragma(time_scope(=(0), =(Instant), Scope)),
	call_with_context(Statement,
		[intersect_scope, query_scope(Scope)]).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- use_module(library('mongolog/mongolog_test')).
:- begin_mongolog_tests('mongolog_temporal','owl/test/swrl.owl').

:- sw_register_prefix(test, 'http://knowrob.org/kb/swrl_test#').

test('during(+Triple,+Interval)') :-
	assert_true(mongolog_call(project(
		triple(test:'Lea', test:hasNumber, '+493455247') during [10,34]))),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [10,34])),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [14,24])),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+999999999') during [5,20])),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455249') during [12,20])),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [5,20])),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [34,44])).

test('during(+Triple,+Overlapping)') :-
	% assert additional interval during which a statement holds that overlaps
	% with an existing interval
	assert_true(mongolog_call(project(
		triple(test:'Lea', test:hasNumber, '+493455249') during [44,84]))),
	assert_true(mongolog_call(project(
		triple(test:'Lea', test:hasNumber, '+493455249') during [24,54]))),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455249') during [34,44])),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455249') during [38,80])),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [140,240])).

test('during(+Triple,[-Since,-Until])', fixme('variables in second argument of during')) :-
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during [_,_])),
	(	mongolog_call(
			triple(test:'Lea', test:hasNumber, '+493455247') during [Since,Until])
	->	assert_equals([Since,Until], [10.0,34.0])
	;	true
	).

test('during(+Triple,-Interval)', fixme('variables in second argument of during')) :-
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+493455247') during _)),
	(	mongolog_call(
			triple(test:'Lea', test:hasNumber, '+493455247') during X)
	->	assert_equals(X,[10.0,34.0])
	;	true
	).

test('since(+Triple,+Instant)') :-
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955247') since 800)),
	assert_true(mongolog_call(project(
		triple(test:'Lea', test:hasNumber, '+499955247') since 800))),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955247') since 800)),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955247') since 1000)),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955247') since 600)).

test('until(+Triple,+Instant)') :-
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955248') until 600)),
	assert_true(mongolog_call(project(
		triple(test:'Lea', test:hasNumber, '+499955248') until 600))),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955248') until 600)),
	assert_true(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955248') until 100)),
	assert_false(mongolog_call(
		triple(test:'Lea', test:hasNumber, '+499955248') until 1000)).

:- end_mongolog_tests('mongolog_temporal').
