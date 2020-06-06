:- module(lang_temporal,
	[ during(r,r),
	  since(r,r),
	  until(r,r)
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('lang/scopes/temporal'),
    [ time_scope/3,
      time_scope_data/2 ]).

:- op(1000, yfx, user:during).
:- op(1000, yfx, user:since).
:- op(1000, yfx, user:until).

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
	  !
	},
	call(Query),
	fact_scope(FScope),
	{ time_scope_data(FScope,TimeTerm) }.

during(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,ground(Since),ground(Until)),
	  time_scope(=<(Since),>=(Until),Scope)
	},
	call(Query,[scope(Scope)]).

during(Query,TimeTerm) +>
	% Use during statements to update scope of existing facts by computing
	% the union of TimeTerm and existing scope.
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
	  !
	},
	call(Query),
	fact_scope(FScope),
	{ time_scope_data(FScope,[TimeTerm,_]) }.

since(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,ground(Since),_),
	  time_scope(=<(Since),_,Scope)
	},
	call(Query,[scope(Scope)]).

since(Query,TimeTerm) +>
	% Use since statements to update scope of existing facts by computing
	% the intersection of TimeTerm and existing scope.
	option(update(intersect)),
	{ interval_data_(TimeTerm,ground(Since),_),
	  % TODO: it is not true that this implies it holds until end of time.
	  %       better would be to represent that it holds at least until some instant.
	  %       However, to properly handle this a notion of uncertainty is needed.
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
	  !
	},
	call(Query),
	fact_scope(FScope),
	{ time_scope_data(FScope,[_,TimeTerm]) }.

until(Query,TimeTerm) ?>
	{ interval_data_(TimeTerm,_,ground(Until)),
	  time_scope(_,>=(Until),Scope)
	},
	call(Query,[scope(Scope)]).

until(Query,TimeTerm) +>
	% Use until statements to update scope of existing facts by computing
	% the intersection of TimeTerm and existing scope.
	option(update(intersect)),
	{ interval_data_(TimeTerm,_,ground(Until)),
	  % TODO: it is not true that this implies it holds since begin of time.
	  %       better would be to represent that it holds at least since some instant.
	  %       However, to properly handle this a notion of uncertainty is needed.
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

test('lang_temporal1') :-
	A=a,
	B=b,
	A=B.

:- end_tests(lang_temporal).
