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

during(Query, [Since,Until]) ?>
	var(Since),
	var(Until),
	call(Query),
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

test('tell Lea hasNumber during') :-
	tell( holds(test:'Lea', test:hasNumber, '+493455247') during [10,34] ),
	tell( holds(test:'Lea', test:hasNumber, '+493455249') during [34,64] ).

test('tell Lea hasNumber overlapping') :-
	% assert additional interval during which a statement holds that overlaps
	% with an existing interval
	tell( holds(test:'Lea', test:hasNumber, '+493455249') during [54,84] ).

test('Lea hasNumber during') :-
	assert_true(holds(test:'Lea', test:hasNumber, '+493455247') during [10,34]),
	assert_true(holds(test:'Lea', test:hasNumber, '+493455247') during [14,24]),
	assert_true(holds(test:'Lea', test:hasNumber, '+493455249') during [34,44]),
	assert_true(holds(test:'Lea', test:hasNumber, '+493455249') during [38,80]).

test('Lea hasNumber during X') :-
	findall(X,
		holds(test:'Lea', test:hasNumber, '+493455247') during X,
		Xs
	),
	assert_true(length(Xs,1)),
	assert_equals(Xs,[[10.0,34.0]]).

test('Lea not hasNumber during') :-
	assert_false(holds(test:'Lea', test:hasNumber, '+999999999') during [5,20]),
	assert_false(holds(test:'Lea', test:hasNumber, '+493455249') during [12,20]),
	assert_false(holds(test:'Lea', test:hasNumber, '+493455247') during [5,20]),
	assert_false(holds(test:'Lea', test:hasNumber, '+493455247') during [34,44]).

:- end_rdf_tests('lang_temporal').
