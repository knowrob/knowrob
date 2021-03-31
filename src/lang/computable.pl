:- module(computable,
    [ computables(t),
      add_computable_predicate/2,
      add_computable_property/2,
      drop_computable_predicate/1,
      drop_computable_property/1,
      drop_computable_property/2
    ]).
/** <module> Loading of computable predicates used to compute relations and data values.

@author Daniel Beßler
@license BSD
*/

% operator used for defining computables
:- op(1150, fx, user:computables).

% computables that were added
:- dynamic computable_predicate/3.
:- dynamic computable_property/3.

%% add_computable_predicate(+Indicator, +Goal) is det.
%
add_computable_predicate(Indicator, Goal) :-
	strip_module(Goal, Module, Goal0),
	assertz(computable_predicate(Indicator, Module, Goal0)).

%% drop_computable_predicate(+Indicator) is det.
%
drop_computable_predicate(Indicator) :-
	retractall(computable_predicate(Indicator, _, _)).

%% add_computable_property(+Property, +Goal) is det.
%
add_computable_property(Property, Goal) :-
	strip_module(Goal, Module, Goal0),
	assertz(computable_property(Property, Module, Goal0)).

%% drop_computable_property(+Module) is det.
%
drop_computable_property(Module) :-
	retractall(computable_property(_, Module, _)).

%% drop_computable_property(+Indicator) is det.
%
drop_computable_property(Module, Property) :-
	retractall(computable_property(Property, Module, _)).

%% computables(+Computables) is det.
%
% Register a list of comutables.
%
% @param Computables list of computables
%
computables(Computables) :-
	comma_list(Computables,List),
	List=[(:(Module,_))|_],
	computables(Module, List).

%
computables(_, []) :- !.
computables(Module, [Computable|Rest]) :-
	Computable =.. [CompFunctor, Arity | RestArgs],
	(	RestArgs==[] -> Options=[]
	;	RestArgs=[Options]
	),
	computables(Module, CompFunctor, Arity, Options),
	computables(Module, Rest).

% computable properties
computables(Module, CompFunctor, Arity, Options) :-
	option(propery(Property), Options),
	!,
	Arity == 2,
	add_computable_property(Property, (:(Module,CompFunctor))).

% computable predicates
computables(Module, CompFunctor, Arity, Options) :-
	option(functor(LangFunctor), Options, CompFunctor),
	Indicator=(/(LangFunctor,Arity)),
	add_computable_predicate(Indicator, (:(Module,CompFunctor))).


% this clause integrated computables with the querying interface
lang_query:call_with(computable, Goal, _Options) :-
	comma_list(Goal, SubGoals),
	maplist([SubGoal,CompGoals]>>
		bagof(X, computable_goal(SubGoal,X), CompGoals),
		SubGoals, CompSubGoals0),
	flatten(CompSubGoals0, CompSubGoals),
	comma_list(CompGoal, CompSubGoals),
	call(CompGoal).

%
computable_goal(Goal,CompGoal) :-
	(	computable_property_goal(Goal,CompGoal)
	;	computable_predicate_goal(Goal,CompGoal)
	).

%
computable_predicate_goal(Goal, CompGoal) :-
	% get callable computable goal
	Goal =.. [Functor0|Args],
	length(Args,Arity),
	computable_predicate(/(Functor0,Arity), Module, Functor1),
	Goal1 =.. [Functor1|Args],
	CompGoal = (:(Module,Goal1)).

%
computable_property_goal(holds(S,P,O), CompGoal) :-
	ground(P),
	% TODO: also include super properties of computable properties
	computable_property(P, Module, Functor1),
	Goal1 =.. [Functor1,S,O],
	CompGoal = (:(Module,Goal1)).


% this clause integrated computables with the querying interface
lang_query:is_callable_with(computable, Goal) :-
	once(is_callable_as_computable(Goal)).

is_callable_as_computable(Goal) :-
	Goal =.. [Functor|Args],
	length(Args,Arity),
	computable_predicate(/(Functor,Arity),_,_).

is_callable_as_computable(holds(_,P,_)) :-
	ground(P),
	% TODO: also include super properties of computable properties
	computable_property(P,_,_).


		 /*******************************
		 *    	  UNIT TESTING     		*
		 *******************************/

test_comp_map(X,Y) :- Y is X * X.
test_comp_gen(X) :- between(1,9,X).

test_setup :-
	add_computable_predicate(comp_gen/1, computable:test_comp_gen),
	add_computable_predicate(comp_map/2, computable:test_comp_map).

test_cleanup :-
	drop_computable_predicate(comp_gen/1),
	drop_computable_predicate(comp_map/2).

:- begin_tests('computable',
		[ setup(computable:test_setup),
		  cleanup(computable:test_cleanup) ]).

test('comp_gen(-)') :-
	findall(X, ask(comp_gen(X)), Xs),
	assert_true(Xs == [1,2,3,4,5,6,7,8,9]).

test('(comp_gen(-),comp_map(+,-))') :-
	findall(Y, ask((
		comp_gen(X),
		comp_map(X,Y)
	)), AllSolutions),
	AllSolutions == [1,4,9,16,25,36,49,64,81].

:- end_tests('computable').

