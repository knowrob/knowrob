:- module(lang_temporal,
	[ during(t,r),
	  since(t,r),
	  until(t,r)
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('lang/scopes/temporal'),
    [ time_scope/3,
      time_scope_data/2 ]).

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
during(Query,TimeTerm) ?>
	{ var(TimeTerm),
	  !,
	  time_scope(>=(0), =<('Infinity'), QScope)
	},
	call(Query,[scope(QScope)]),
	fact_scope(FScope),
	{ time_scope_data(FScope,TimeTerm) }.

during(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,ground(Since),ground(Until)),
	  time_scope(=<(Since),>=(Until),Scope)
	},
	call(Query,[scope(Scope)]).

during(Query,TimeTerm) +>
	option(update(union)),
	{ interval_data_(TimeTerm,ground(Since),ground(Until)),
	  time_scope(=(Since),=(Until),Scope)
	  %time_scope(=<(Since),>=(Until),Scope)
	},
	call(Query,[scope(Scope)]).

%% since(+Statement,?Interval) is nondet.
%
% True for statements that hold (at least) since some time
% instant.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
since(Query,TimeTerm) ?>
	{ var(TimeTerm),
	  !,
	  time_scope(>=(0), =<('Infinity'), QScope)
	},
	call(Query,[scope(QScope)]),
	fact_scope(FScope),
	{ time_scope_data(FScope,[TimeTerm,_]) }.

since(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,ground(Since),_),
	  time_scope(=<(Since),_,Scope)
	},
	call(Query,[scope(Scope)]).

since(Query,TimeTerm) +>
	option(update(intersect)),
	{ interval_data_(TimeTerm,ground(Since),_),
	  % TODO: it is not true that this implies it holds until end of time.
	  %       better would be to represent that it holds at least until some instant.
	  %       However, to properly handle this a notion of uncertainty might be needed.
	  time_scope(=(Since),=('Infinity'),Scope)
	  %get_time(Now), time_scope(=(Since),=(Now),Scope)
	  %time_scope(=(Since),>=(Since),Scope)
	},
	call(Query,[scope(Scope)]).

%% until(+Statement,?Interval) is nondet.
%
% True for statements that hold (at least) until some time
% instant.
%
% @param Statement A language term.
% @param Interval A time interval, instant, or event.
%
until(Query,TimeTerm) ?>
	{ var(TimeTerm),
	  !,
	  time_scope(>=(0), =<('Infinity'), QScope)
	},
	call(Query,[scope(QScope)]),
	fact_scope(FScope),
	{ time_scope_data(FScope,[_,TimeTerm]) }.

until(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,_,ground(Until)),
	  time_scope(_,>=(Until),Scope)
	},
	call(Query,[scope(Scope)]).

until(Query,TimeTerm) +>
	option(update(intersect)),
	{ interval_data_(TimeTerm,_,ground(Until)),
	  % TODO: it is not true that this implies it holds since begin of time.
	  %       better would be to represent that it holds at least since some instant.
	  %       However, to properly handle this a notion of uncertainty might be needed.
	  time_scope(=(0),=(Until),Scope)
	  %time_scope(=(Until),=(Until),Scope)
	  %time_scope(=<(Until),=(Until),UntilScope)
	},
	call(Query,[scope(Scope)]).

%%
interval_data_(Term,Since,Until) :-
	ask(has_interval_data(Term,X,Y))
	-> ( interval_data__(Term,Since,X),
	     interval_data__(Term,Until,Y) )
	;  ( throw(temporal_scope(interval_data(nodata(Term)))) ).

interval_data__(Term,ground(Y),Y) :-
	!,
	( ground(Y) -> true
	; throw(temporal_scope(interval_data(not_ground(Term))))
	).
interval_data__(_,X,X).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_tests(lang_temporal).

:- tripledb_load(
		'package://knowrob/owl/test/swrl.owl',
		[ graph(user),
		  namespace(test_swrl,'http://knowrob.org/kb/swrl_test#')
		]).

test('tell Lea hasNumber during') :-
	tell( holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during [10,34] ),
	tell( holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455249') during [34,64] ).

test('tell Lea hasNumber overlapping') :-
	% assert additional interval during which a statement holds that overlaps
	% with an existing interval
	tell( holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455249') during [54,84] ).

test('Lea hasNumber during') :-
	assert_true(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during [10,34]),
	assert_true(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during [14,24]),
	assert_true(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455249') during [34,44]),
	assert_true(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455249') during [38,80]).

test('Lea hasNumber during X') :-
	holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during X,
	assert_equals(X,[10,34]).

test('Lea not hasNumber during') :-
	assert_false(holds(test_swrl:'Lea', test_swrl:hasNumber, '+999999999') during [5,20]),
	assert_false(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455249') during [12,20]),
	assert_false(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during [5,20]),
	assert_false(holds(test_swrl:'Lea', test_swrl:hasNumber, '+493455247') during [34,44]).

test('forget Lea hasNumber') :-
	tripledb_forget(test_swrl:'Lea', test_swrl:hasNumber, _).

:- end_tests(lang_temporal).
