:- module(computable,
    [ computables(t),
      op(1150, fx, computables)
    ]).
/** <module> Implementation of computable properties.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/OWL'),
    [ is_object_property/1,
      is_data_property/1
    ]).
:- use_module(library('db/obda'),
    [ obda_add/1
    ]).
:- use_module(library('reasoning/pool'),
    [ reasoning_module/1
    ]).
:- use_module('./terms/is_a.pl',
    [ subproperty_of/2
    ]).

%% computables(+Computables) is det
%
%
%
computables(Computables) :-
  % peak module M
  Computables=(,(:(M,_),_)),
  computable_list_(Computables,List),
  %
  findall(OC, (
    member(OC,List),
    is_object_computable(OC)
  ), ObjectComputables),
  computable_reasoner_(M,ObjectComputables),
  %
  findall(DC, (
    member(DC,List),
    is_datatype_computable(DC)
  ), DatatypeComputables),
  computable_obda_(M,DatatypeComputables).

%%
computable_list_(Computables,[C|Xs]) :-
  Computables=','(C,Cs),
  computable_list_(Cs,Xs).

computable_list_(C,[C]).

%%
assert_scoped(M,Head,Body) :-
  assertz((:-((:(M,Head)),Body))).

		 /*******************************
		 *	   RELATIONS	*
		 *******************************/

%%
is_object_computable(C) :-
  C=..[_,Property|_],
  is_object_property(Property).

%%
computable_reasoner_(_,[]) :- !.

computable_reasoner_(M,ObjectComputables) :-
  % queries can be answered in case of property is a variable
  assert_scoped(M,
        can_answer(holds(_,P,_)),
        var(P)),
  % assert various can_answer and infer clauses
  forall(
    member(C,ObjectComputables),
    computable_reasoner2_(M,C)
  ),
  % create one more *infer* clause that calls *infer2*
  % only if property is grounded (i.e. only yield specific
  % properties in case property is a var)
  assert_scoped(M,
        infer(holds(S,P0,O),Fact,Scope),
        ( \+ var(P0),
          infer2(holds(S,P0,O),Fact,Scope)
        )),
  % register reasoner
  reasoning_module(M).

computable_reasoner2_(Module,ComputableTerm) :-
  ComputableTerm=..[Predicate,Property|Configuration],
  % validate
  callable(Predicate),
  atom(Property),
  %
  forall(
    subproperty_of(Property,SupProperty),
    ( % assert *can_answer* clause
      assert_can_answer_(Module,SupProperty),
      % assert *infer* clause
      assert_infer_(Module,SupProperty,Property,Predicate)
    )
  ).

%%
assert_can_answer_(M,P) :-
  Predicate=(:(M,can_answer(holds(_,P,_)))),
  ( clause(Predicate,true) ;
    assertz(Predicate)
  ).

%%
assert_infer_(M,P_sup,P_specific,Predicate) :-
  ( P_sup=P_specific -> X=infer ; X=infer2 ),
  Head=..[X,holds(S,P_sup,O),holds(S,P_specific,O),Scope],
  Body=..[Predicate,S,O,Scope],
  assert_scoped(M,Head,Body).

		 /*******************************
		 *	   OBDA	*
		 *******************************/

%%
is_datatype_computable(C) :-
  C=..[_,Property|_],
  is_data_property(Property).

%%
computable_obda_([]) :- !.

computable_obda_(M,DatatypeProperties) :-
  % queries can be answered in case of property is a variable
  assert_scoped(M,
        can_access(holds(_,P,_)),
        var(P)),
  % assert various can_access and access clauses
  forall(
    member(C,ObjectComputables),
    computable_obda2_(M,C)
  ),
  % register obda client
  obda_add(M).

computable_obda2_(Module,ComputableTerm) :-
  ComputableTerm=..[Predicate,Property|Configuration],
  % validate
  callable(Predicate),
  atom(Property),
  %
  forall(
    subproperty_of(Property,X),
    ( % assert *can_answer* clause
      assert_can_access_(Module,SubProperty),
      % assert *infer* clause
      assert_access_(Module,SubProperty,Property,Predicate)
    )
  ).

%%
assert_can_access_(P) :-
  Predicate=(:(M,can_access(P))),
  ( clause(Predicate,true) ;
    assertz(Predicate)
  ).

%%
assert_access_(M,P_parent,_P_specific,Predicate) :-
  Head=access(S,P_parent,O,QScope,FScope),
  Body=..[Predicate,S,O,QScope->FScope],
  assert_scoped(M,Head,Body).

