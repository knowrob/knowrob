:- module(temporal_scope,
	[ time_scope_data/2,
	  time_subscope_of/2,
	  time_scope_intersect/3,
	  time_scope_merge/3,
	  time_scope_universal/1,
	  time_scope_now/1,
	  time_scope/3,
	  time_scope/5
	]).
/** <module> Temporally scoped statements.

@author Daniel Beßler
@license BSD
*/

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
	( get_dict(time,Scope,X) ; X=Scope ),
	time_scope_data_(since,X,Since),
	time_scope_data_(until,X,Until),
	!.

%%
time_scope_data_(Key,Dict,Val) :-
	get_dict(Key,Dict,Val0),!,
	strip_operator_(Val0,Op,double(Val1)),
	( Op='='
	-> Val=Val1
	;  strip_operator_(Val,Op,Val1)
	).

time_scope_data_(_,_,_).

%% time_scope_universal(-Scope) is det.
%
% From begin of time until its end.
%
% @param Scope A scope dict.
%
time_scope_universal(Scope) :-
	time_scope(=(0), =('Infinity'), _{ time: Scope }).

%% time_scope_now(-Scope) is det.
%
% A scope that matches the current time.
%
% @param Scope A scope dict.
%
time_scope_now(Scope) :-
	get_time(Now),
	time_scope(=<(Now), >=(Now), _{ time: Scope }).

%% time_subscope_of(+A,+B) is semidet.
%
% All statements in A are also included in B.
%
% @param A A scope dict.
% @param B A scope dict.
%
time_subscope_of(Sub,Sup) :-
	time_scope_data(Sub,[Sub0,Sub1]),
	time_scope_data(Sup,[Sup0,Sup1]),
	time_scope_min_(Sub0,Sup0,Sup0),
	time_scope_max_(Sub1,Sup1,Sup1).

%% time_scope_merge(+A,+B,-Merged) is semidet.
%
% Merged is the scope containing all statements from
% scope A and scope B, where A and B overlap
% (merge not possible if disjoint).
%
% @param A A scope dict.
% @param B A scope dict.
% @param Merged A scope dict.
%
time_scope_merge(A,B,Merged) :-
	time_scope_data(A,[Since0,Until0]),
	time_scope_data(B,[Since1,Until1]),
	time_scope_overlaps_([Since0,Until0],
	                     [Since1,Until1]),
	time_scope_min_(Since0,Since1,Since),
	time_scope_max_(Until0,Until1,Until),
	( ground(Until)
	-> Merged=_{ since: double(Since), until: double(Until) }
	;  Merged=_{ since: double(Since) }
	).

%%
time_scope_overlaps_([S0,U0],[S1,U1]) :-
	time_scope_inside_(S0,[S1,U1]);
	time_scope_inside_(U0,[S1,U1]).

%%
time_scope_inside_(X,[_S,U]) :-
	var(X),!,
	var(U).

time_scope_inside_(X,[S,U]) :-
	time_scope_max_(X,S,X),
	( ground(U)
	-> time_scope_max_(X,U,U)
	;  true
	).

%% time_scope_merge(+A,+B,-Intersection) is semidet.
%
% Modify A such that it only contains time instants
% that are also contained in B.
%
% @param A A scope dict.
% @param B A scope dict.
% @param Intersection A scope dict.
%
time_scope_intersect(A,B,Intersection) :-
	time_scope_data(A,[Since0,Until0]),
	time_scope_data(B,[Since1,Until1]),
	time_scope_max_(Since0,Since1,Since), ground(Since),
	time_scope_min_(Until0,Until1,Until),
	( var(Until) ; time_scope_min_(Since,Until,Since) ),!,
	( ground(Until)
	-> Intersection=_{ since: double(Since), until: double(Until) }
	;  Intersection=_{ since: double(Since) }
	).

%%
time_scope_overlaps_query(A,B) :-
	time_scope_data(A,[Since0,Until0]),
	strip_operator_(Since0,_,Since),
	strip_operator_(Until0,_,Until),
	time_scope(=<(Until),>=(Since),TimeScope),
	get_dict(time,TimeScope,B).

%%
time_scope_min_('Infinity',Min,Min) :- !.
time_scope_min_(Min,'Infinity',Min) :- !.
time_scope_min_(0,_,0) :- !.
time_scope_min_(_,0,0) :- !.
time_scope_min_(X0,X1,X1)  :- var(X0),!.
time_scope_min_(X0,X1,X0)  :- var(X1),!.
time_scope_min_(X0,X1,Min) :-
	strip_operator_(X0,Op0,V0),
	strip_operator_(X1,Op1,V1),
	operator_polarization_(Op0,P0),
	operator_polarization_(Op1,P1),
	( P0<P1 -> Min=X0
	; P1<P0 -> Min=X1
	; V0<V1 -> Min=X0
	; Min=X1
	).

%%
time_scope_max_('Infinity',_,'Infinity') :- !.
time_scope_max_(_,'Infinity','Infinity') :- !.
time_scope_max_(0,Max,Max) :- !.
time_scope_max_(Max,0,Max) :- !.
time_scope_max_(X0,_,_)    :- var(X0),!.
time_scope_max_(_,X1,_)    :- var(X1),!.
time_scope_max_(X0,X1,Max) :-
	strip_operator_(X0,Op0,V0),
	strip_operator_(X1,Op1,V1),
	operator_polarization_(Op0,P0),
	operator_polarization_(Op1,P1),
	( P0>P1 -> Max=X0
	; P1>P0 -> Max=X1
	; V0>V1 -> Max=X0
	; Max=X1
	).

%% time_scope(+Since,+Until,-Scope) is det.
%
time_scope(SinceTerm,UntilTerm,Scope) :-
	strip_operator_(SinceTerm,SinceOperator,SinceValue),
	strip_operator_(UntilTerm,UntilOperator,UntilValue),
	time_scope(
		SinceValue,SinceOperator,
		UntilValue,UntilOperator,
		Scope).

%%
strip_operator_(>=(Value),>=,Value) :- !.
strip_operator_(=<(Value),=<,Value) :- !.
strip_operator_(>(Value),>,Value) :- !.
strip_operator_(<(Value),<,Value) :- !.
strip_operator_(=(Value),=,Value) :- !.
strip_operator_(Value,=,Value) :- !.

%%
operator_polarization_( <,-1) :- !.
operator_polarization_(=<,-1) :- !.
operator_polarization_( =, 0) :- !.
operator_polarization_( >, 1) :- !.
operator_polarization_(>=, 1) :- !.

%% time_scope(+Since,+Op1,+Until,+Op2,-Scope) is det.
%
% Create a new scope dcitionary from given arguments.
%
% @param Since The since time.
% @param Op1 The operator for since time.
% @param Until The until time.
% @param Op2 The operator for until time.
% @param Scope A scope dict.
%
time_scope(Since,SinceOperator,Until,_,
           _{ time: _{ since: SinceTerm }}) :-
	var(Until),!,
	get_scope1_(Since,SinceOperator,SinceTerm).

time_scope(Since,_,Until,UntilOperator,
           _{ time: _{ until: UntilTerm }}) :-
	var(Since),!,
	get_scope1_(Until,UntilOperator,UntilTerm).

time_scope(Since,SinceOperator,Until,UntilOperator,
           _{ time: _{ since: SinceTerm, until: UntilTerm }}) :-
	get_scope1_(Since,SinceOperator,SinceTerm),
	get_scope1_(Until,UntilOperator,UntilTerm).

get_scope1_(Value,Operator,Term) :-
	( Operator='=' %var(Operator)
	-> Term=double(Value)
	;  Term=..[Operator,double(Value)]
	).

%%
% Register temporal scope by declaring clauses in scope module.
%
scope:universal_scope(time,V)          :- time_scope_universal(V).
scope:current_scope(time,V)            :- time_scope_now(V).
scope:subscope_of(time,V0,V1)          :- time_subscope_of(V0,V1).
scope:scope_merge(time,V0,V1,V)        :- time_scope_merge(V0,V1,V).
scope:scope_intersect(time,V0,V1,V)    :- time_scope_intersect(V0,V1,V).
scope:scope_overlaps_query(time,V0,V1) :- time_scope_overlaps_query(V0,V1).

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
	time_scope_universal(S1),
	time_scope_now(S2),
	assertion(time_subscope_of(S2,S1)).

test('subscope_of([15,18],[10,20])') :-
	time_scope(=(10),=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   time_subscope_of(S2,S1)),
	assertion(\+ time_subscope_of(S1,S2)).

test('subscope_of([15,18],[15,20])') :-
	time_scope(=(15),=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   time_subscope_of(S2,S1)),
	assertion(\+ time_subscope_of(S1,S2)).

test('subscope_of([15,18],[15,18])') :-
	time_scope(=(15),=(18),S1),
	time_scope(=(15),=(18),S2),
	assertion(time_subscope_of(S2,S1)),
	assertion(time_subscope_of(S1,S2)).

test('subscope_of([15,18],[10,>=20])') :-
	time_scope(=(10),>=(20),S1),
	time_scope(=(15),=(18),S2),
	assertion(   time_subscope_of(S2,S1)),
	assertion(\+ time_subscope_of(S1,S2)).

test('merge_scope([5,18],[10,20])') :-
	time_scope(=(5),  =(18), S1),
	time_scope(=(10), =(20), S2),
	time_scope_merge(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [5, 20]).

test('merge_scope([5,10],[10,20])') :-
	time_scope(=(5),  =(10), S1),
	time_scope(=(10), =(20), S2),
	time_scope_merge(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [5,20]).

test('merge_scope([5,9],[10,20])') :-
	time_scope(=(5),  =(9),  S1),
	time_scope(=(10), =(20), S2),
	assertion(\+ time_scope_merge(S1,S2,_)).

test('merge_scope([5,10],[10,>=20])') :-
	time_scope(=(5),   =(10), S1),
	time_scope(=(10), >=(20), S2),
	time_scope_merge(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [5, (>=(20))]).

test('merge_scope([5,10],[=<10,20])') :-
	time_scope(=(5),   =(10), S1),
	time_scope(=<(10), =(20), S2),
	time_scope_merge(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [=<(10),20]).

test('intersect_scope([5,18],[10,20])') :-
	time_scope(=(5),  =(18), S1),
	time_scope(=(10), =(20), S2),
	time_scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10, 18]).

test('intersect_scope([5,10],[10,20])') :-
	time_scope(=(5),  =(10), S1),
	time_scope(=(10), =(20), S2),
	time_scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,10]).

test('intersect_scope([5,9],[10,20])') :-
	time_scope(=(5),  =(9),  S1),
	time_scope(=(10), =(20), S2),
	assertion(\+ time_scope_intersect(S1,S2,_)).

test('intersect_scope([5,18],[10,>=20])') :-
	time_scope(=(5),   =(18), S1),
	time_scope(=(10), >=(20), S2),
	time_scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,18]).

test('intersect_scope([5,>=18],[10,>=20])') :-
	time_scope(=(5),  >=(18), S1),
	time_scope(=(10), >=(20), S2),
	time_scope_intersect(S1, S2, S3),
	time_scope_data(S3,Data),
	assertion(ground(Data)),
	assertion(Data = [10,(>=(18))]).

:- end_tests(temporal_scope).
