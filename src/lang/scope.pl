:- module(lang_scope,
	[ universal_scope/1,
	  wildcard_scope/1,
	  current_scope/1,
	  subscope_of/2,
	  scope_intersect/3,
	  time_scope/3,
	  time_scope_data/2
    ]).
/** <module> The scope of statements being true.

@author Daniel Beßler
@license BSD
*/

%% universal_scope(-Scope) is det.
%
% The scope of facts that are universally true.
%
% @param Scope A scope dictionary.
%
universal_scope(_{
	time: _{
		since: =(double(0)),
		until: =(double('Infinity'))
	}
}).

%% current_scope(-Scope) is det.
%
% The scope of facts that are currently true.
%
% @param Scope A scope dictionary.
%
current_scope(_{
	time: _{
		since: =<(double(Now)),
		until: >=(double(Now))
	}
}) :- get_time(Now).

%% wildcard_scope(-Scope) is det.
%
% A scope that matches any fact.
%
% @param Scope A scope dictionary.
%
wildcard_scope(_{}).

%% subscope_of(+Sub,+Sup) is det.
%
% True if scope Sup contains all facts that are contained
% in scope Sub.
%
% @param Sub a scope dict.
% @param Sup a scope dict.
%
subscope_of(
		_{ time: TimeSub },
		_{ time: TimeSup }) :-
	time_subscope_of(TimeSub, TimeSup).

%%
time_subscope_of(Sub,Sup) :-
	time_scope_data(Sub,[Sub0,Sub1]),
	time_scope_data(Sup,[Sup0,Sup1]),
	get_min(Sub0,Sup0,Sup0),
	get_max(Sub1,Sup1,Sup1).

%% scope_intersect(+A,+B,-Merged) is det.
%
% Intersect two scopes. The intersected scope contains all
% facts that are contained in scope A and also in scope B.
%
% @param A a scope dict.
% @param B a scope dict.
%
scope_intersect(_{}, Scope, Scope) :-
	!.
scope_intersect(Scope, _{}, Scope) :-
	!.
scope_intersect(Scope, Scope, Scope) :-
	!.
scope_intersect(
		_{ time: TimeA },
		_{ time: TimeB },
		_{ time: Time }) :-
	time_scope_intersect(TimeA, TimeB, Time).

%%
time_scope_intersect(A, B, Intersection) :-
	time_scope_data(A, [Since0,Until0]),
	time_scope_data(B, [Since1,Until1]),
	get_max(Since0,Since1,Since), ground(Since),
	get_min(Until0,Until1,Until),
	(	var(Until)
	;	get_min(Since,Until,Since)
	),
	!,
	(	ground(Until)
	->	Intersection=_{ since: double(Since), until: double(Until) }
	;	Intersection=_{ since: double(Since) }
	).

%% time_scope(?Since,?Until,?Scope) is semidet.
%
% @param Scope A scope dict.
%
time_scope(Scope,Since,Until) :-
	(	Scope=_{ time: X }
	;	X=Scope
	),
	X=_{ since: Since, until: Until },
	!.

%% time_scope_data(+Scope,?IntervalData) is det.
%
% Read since/until pair of temporal scope.
% Note that vars are used in case since or until
% not known.
%
% @param Scope A scope dict.
% @param IntervalData A list [Since,Until].
%
time_scope_data(Scope,[Since,Until]) :-
	(	get_dict(time,Scope,X)
	;	X=Scope
	),
	time_scope_data_(since,X,Since),
	time_scope_data_(until,X,Until),
	!.

%%
time_scope_data_(Key,Dict,Val) :-
	get_dict(Key,Dict,Val0),
	!,
	mng_strip_operator(Val0, Op, double(Val1)),
	(	Op='='
	->	Val=Val1
	;	strip_operator_(Val,Op,Val1)
	).

time_scope_data_(_,_,_).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% HELPER
%%%%%%%%%%%%%%%%%%%%%%%

% get min number, supports ungrounded values and 'Infinity' value
get_min('Infinity',Min,Min) :- !.
get_min(Min,'Infinity',Min) :- !.
get_min(0,_,0) :- !.
get_min(_,0,0) :- !.
get_min(X0,X1,X1)  :- var(X0),!.
get_min(X0,X1,X0)  :- var(X1),!.
get_min(X0,X1,Min) :-
	mng_strip_operator(X0, Op0, V0),
	mng_strip_operator(X1, Op1, V1),
	operator_polarization(Op0,P0),
	operator_polarization(Op1,P1),
	( P0<P1 -> is_equal_(X0,Min)
	; P1<P0 -> is_equal_(X1,Min)
	; V0<V1 -> is_equal_(X0,Min)
	; is_equal_(X1,Min)
	).

% get max number, supports ungrounded values and 'Infinity' value
get_max('Infinity',_,'Infinity') :- !.
get_max(_,'Infinity','Infinity') :- !.
get_max(0,Max,Max) :- !.
get_max(Max,0,Max) :- !.
get_max(X0,_,_)    :- var(X0),!.
get_max(_,X1,_)    :- var(X1),!.
get_max(X0,X1,Max) :-
	mng_strip_operator(X0, Op0, V0),
	mng_strip_operator(X1, Op1, V1),
	operator_polarization(Op0,P0),
	operator_polarization(Op1,P1),
	( P0>P1 -> is_equal(X0,Max)
	; P1>P0 -> is_equal(X1,Max)
	; V0>V1 -> is_equal(X0,Max)
	; is_equal(X1,Max)
	).

%%
operator_polarization( <,-1) :- !.
operator_polarization(=<,-1) :- !.
operator_polarization( =, 0) :- !.
operator_polarization( >, 1) :- !.
operator_polarization(>=, 1) :- !.

%
is_equal(X,X) :- !.
is_equal(X,Y) :-
	ground(Y),
	mng_strip_operator(X,Op,VX),
	mng_strip_operator(Y,Op,VY),
	VX_f is float(VX),
	VX_f is float(VY).

		 /*******************************
		 *	    UNIT TESTS	     		*
		 *******************************/

:- begin_tests(temporal_scope).

test('scope_data([3,20])') :-
	time_scope(=(3), =(20), S),
	time_scope_data(S,Data),
	assertion(ground(Data)),
	assertion(Data = [3,20]).

test('scope_data([3,>=20])') :-
	time_scope(=(3), >=(20), S),
	time_scope_data(S,Data),
	assertion(ground(Data)),
	assertion(Data = [3,>=(20)]).

test('scope_data([=<3,20])') :-
	time_scope(=<(3), =(20), S),
	time_scope_data(S,Data),
	assertion(ground(Data)),
	assertion(Data = [=<(3),20]).

test('subscope_of(current,universal)') :-
	universal_scope(S1),
	current_scope(S2),
	assertion(subscope_of(S2,S1)).

test('subscope_of([15,18],[10,20])') :-
	time_scope(=(10),=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   subscope_of(S2,S1)),
	assertion(\+ subscope_of(S1,S2)).

test('subscope_of([15,18],[15,20])') :-
	time_scope(=(15),=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   subscope_of(S2,S1)),
	assertion(\+ subscope_of(S1,S2)).

test('subscope_of([15,18],[15,18])') :-
	time_scope(=(15),=(18),S1),
	time_scope(=(15),=(18),S2),
	assertion(subscope_of(S2,S1)),
	assertion(subscope_of(S1,S2)).

test('subscope_of([15,18],[10,>=20])') :-
	time_scope(=(10),>=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   subscope_of(S2,S1)),
	assertion(\+ subscope_of(S1,S2)).

test('intersect_scope([5,18],[10,20])') :-
	time_scope(=(5),  =(18), S1),
	time_scope(=(10), =(20), S2),
	scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10, 18]).

test('intersect_scope([5,10],[10,20])') :-
	time_scope(=(5),  =(10), S1),
	time_scope(=(10), =(20), S2),
	scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,10]).

test('intersect_scope([5,9],[10,20])') :-
	time_scope(=(5),  =(9),  S1),
	time_scope(=(10), =(20), S2),
	assertion(\+ scope_intersect(S1,S2,_)).

test('intersect_scope([5,18],[10,>=20])') :-
	time_scope(=(5),   =(18), S1),
	time_scope(=(10), >=(20), S2),
	scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,18]).

test('intersect_scope([5,>=18],[10,>=20])') :-
	time_scope(=(5),  >=(18), S1),
	time_scope(=(10), >=(20), S2),
	scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,(>=(18))]).

:- end_tests(temporal_scope).
