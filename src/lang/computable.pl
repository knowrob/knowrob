:- module(computable,
    [ add_computable_predicate/2,
      drop_computable_predicate/1
      %computables(t)
    ]).
/** <module> Loading of computable predicates used to compute relations and data values.

@author Daniel Beßler
@license BSD
*/

:- dynamic computable_predicate/3.

%% add_computable_predicate(+Indicator, +Goal) is det.
%
add_computable_predicate(Indicator, Goal) :-
	strip_module(Goal, Module, Goal0),
	assertz(computable_predicate(Indicator, Module, Goal0)).

%% drop_computable_predicate(+Indicator) is det.
%
drop_computable_predicate(Indicator) :-
	retractall(computable_predicate(Indicator, _, _)).


		 /*******************************
		 *	 	   lang_query     		*
		 *******************************/

%
lang_query:is_callable_with(computable, Goal) :-
	Goal =.. [Functor|Args],
	length(Args,Arity),
	once(computable_predicate(/(Functor,Arity),_,_)).

%
lang_query:call_with(computable, Goal, _Options) :-
	comma_list(Goal, SubGoals),
	maplist([SubGoal,CompGoal]>>
		computable_goal(SubGoal,CompGoal),
		SubGoals, CompSubGoals),
	comma_list(CompGoal, CompSubGoals),
	call(CompGoal).

computable_goal(Goal,CompGoal) :-
	% get callable computable goal
	Goal =.. [Functor0|Args],
	length(Args,Arity),
	computable_predicate(/(Functor0,Arity), Module, Functor1),
	Goal1 =.. [Functor1|Args],
	CompGoal = (:(Module,Goal1)).

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
	Xs == [1,2,3,4,5,6,7,8,9].

test('(comp_gen(-),comp_map(+,-))') :-
	findall(Y, ask((
		comp_gen(X),
		comp_map(X,Y)
	)), AllSolutions),
	AllSolutions == [1,4,9,16,25,36,49,64,81].

:- end_tests('computable').


%:- op(1150, fx, user:computables).
%
%:- use_module(library('model/OWL'),
%    [ is_object_property/1,
%      is_data_property/1
%    ]).

%% computables(+Computables) is det.
%
% Register a list of comutable predicates.
% Each computable is represented as list
% `[Predicate,Property]` where Predicate is a Prolog
% predicate, and Property is a RDF property.
%
% @param Computables list of computables
%
%computables(Computables) :-
%  % peak module M
%  computable_list_(Computables,List),
%  List=[(:(M,_))|_],
%  %
%  findall(OC, (
%    member(OC,List),
%    is_object_computable(OC)
%  ), ObjectComputables),
%  computable_reasoner_(M,ObjectComputables),
%  %
%  findall(DC, (
%    member(DC,List),
%    is_datatype_computable(DC)
%  ), DatatypeComputables),
%  computable_obda_(M,DatatypeComputables).
%
%%%
%computable_list_(Computables,[C|Xs]) :-
%  Computables=','(C,Cs),!,
%  computable_list_(Cs,Xs).
%
%computable_list_(C,[C]).
%
%%%
%assert_scoped(M,Head,Body) :-
%  assertz((:-((:(M,Head)),Body))).
%
%		 /*******************************
%		 *	   RELATIONS	*
%		 *******************************/
%
%%%
%is_object_computable((:(_,C))) :-
%  C=..[_,Property|_],
%  is_object_property(Property).
%
%%%
%computable_reasoner_(_,[]) :- !.
%
%computable_reasoner_(M,ObjectComputables) :-
%  % assert various can_answer and infer clauses
%  forall(
%    member(C,ObjectComputables),
%    computable_reasoner2_(C)
%  ),
%  % queries can be answered in case of property is a variable
%  assert_scoped(M,
%        can_answer(holds(_,P,_)),
%        var(P)),
%  % create one more *infer* clause that calls *infer2*
%  % only if property is grounded (i.e. only yield specific
%  % properties in case property is a var)
%  assert_scoped(M,
%        infer(holds(S,P0,O),Fact,Scope),
%        ( \+ var(P0),
%          infer2(holds(S,P0,O),Fact,Scope)
%        )),
%  % register reasoner
%  register_reasoner(M).
%
%computable_reasoner2_(:(Module,ComputableTerm)) :-
%  ComputableTerm=..[Predicate,Property],
%  % TODO validate
%  atom(Property),
%  %
%  forall(
%    % FIXME: we assume here that property hierarchy never changes.
%    %         better would be to re-initialize in case the hierarchy changes.
%    transitive(subproperty_of(Property,SupProperty)),
%    ( % assert *can_answer* clause
%      assert_can_answer_(Module,SupProperty),
%      % assert *infer* clause
%      assert_infer_(Module,SupProperty,Property,Predicate)
%    )
%  ).
%
%%%
%assert_can_answer_(M,P) :-
%  Predicate=(:(M,can_answer(holds(_,P,_)))),
%  ( clause(Predicate,true) ;
%    assertz(Predicate)
%  ).
%
%%%
%assert_infer_(M,P_sup,P_specific,Predicate) :-
%  ( P_sup=P_specific -> X=infer ; X=infer2 ),
%  Head=..[X,holds(S,P_sup,O),holds(S,P_specific,O),Scope],
%  Goal=..[Predicate,S,O],
%  assert_scoped(M,Head,ask(Goal,Scope)).
%
%		 /*******************************
%		 *	   OBDA	*
%		 *******************************/
%
%%%
%is_datatype_computable((:(_,C))) :-
%  C=..[_,Property|_],
%  is_data_property(Property).
%
%%%
%computable_obda_(_,[]) :- !.
%
%computable_obda_(M,DatatypeProperties) :-
%  % assert various can_access and access clauses
%  forall(
%    member(C,DatatypeProperties),
%    computable_obda2_(M,C)
%  ),
%  % queries can be answered in case of property is a variable
%  assert_scoped(M,
%        can_access(holds(_,P,_)),
%        var(P)),
%  % register obda client
%  obda_add(M).
%
%computable_obda2_(Module,ComputableTerm) :-
%  ComputableTerm=..[Predicate,Property],
%  % validate
%  callable(Predicate),
%  atom(Property),
%  %
%  forall(
%    % FIXME: we assume here that property hierarchy never changes.
%    %         better would be to re-initialize in case the hierarchy changes.
%    transitive(subproperty_of(Property,X)),
%    ( % assert *can_answer* clause
%      assert_can_access_(Module,X),
%      % assert *infer* clause
%      assert_access_(Module,X,Property,Predicate)
%    )
%  ).
%
%%%
%assert_can_access_(M,P) :-
%  Predicate=(:(M,can_access(P))),
%  ( clause(Predicate,true) ;
%    assertz(Predicate)
%  ).
%
%%%
%assert_access_(M,P_parent,_P_specific,Predicate) :-
%  Head=access(S,P_parent,O,QScope,FScope),
%  Body=..[Predicate,S,O,QScope->FScope],
%  assert_scoped(M,Head,Body).

