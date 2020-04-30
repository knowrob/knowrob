:- module(plowl_class,
    [ owl_subclass_of(r,t),
      owl_property_range(r,r,t),
      owl_property_cardinality(r,r,t,?,?)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('model/OWL'),
    [ owl_description/2,
      is_class/1,
      is_object_property/1,
      is_functional_property/1,
      has_equivalent_class/2,
      has_inverse_property/2
    ]).
:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2,
      subproperty_of/2
    ]).

		 /*******************************
		 *	    SUBCLASS-OF     		*
		 *******************************/

%%
%
%
owl_subclass_of(Sub,Sup) :-
  var(Sup),!,
  owl_subclass_of1(Sub,Sup0),
  once(( Sup0=class(Sup) ; Sup=Sup0 )).

owl_subclass_of(Sub,Sup) :-
  ground(Sup),!,
  owl_description(Sup,Descr),
  owl_subclass_of1(Sub,Descr).

owl_subclass_of1(Sub,only(P,Range)) :-
  owl_subclass_of_only(Sub,P,Range).

owl_subclass_of1(Sub,some(P,Range)) :-
  subclass_of(Sub,min(P,Range,Min)),
  Min>0.

owl_subclass_of1(Sub,min(P,Range,Min)) :-
  owl_property_cardinality(Sub,P,Range,Min,_),
  Min>0.

owl_subclass_of1(Sub,max(P,Range,Max)) :-
  owl_property_cardinality(Sub,P,Range,_,Max),
  Max \= inf.

owl_subclass_of1(Sub,exactly(P,Range,Card)) :-
  owl_property_cardinality(Sub,P,Range,Card,Card).

owl_subclass_of1(Sub,class(Sup)) :-
  % subclass-of constraints of equivalent classes
  ( has_equivalent_class(Sub,Sub_EQ) ; Sub_EQ=Sub ),
  ( has_equivalent_class(Sup,Sup_EQ) ; Sup_EQ=Sup ),
  once(( Sub_EQ\=Sub ; Sup_EQ\=Sup )),
  % FIXME: need to avoid cycles here!
  subclass_of(Sub_EQ,Sup_EQ).

%%
owl_subclass_of_only(Sub,P,Range) :-
  var(P),!,
  is_object_property(P),
  owl_subclass_of_only(Sub,P,Range).

owl_subclass_of_only(Sub,P,Range) :-
  ground(Range),!,
  forall(
    owl_property_range(Sub,P,R0),
    subclass_of(R0,Range)).

owl_subclass_of_only(Sub,P,Range) :-
  % var(Range),
  findall(R0,
    owl_property_range(Sub,P,R0),
    Ranges),
  % find most specific shared super classes
  common_supclass_(Ranges,Range).

%%
common_supclass_([X0|Xs],Sup) :-
  % find shared super classes
  findall(Y,
    ( subclass_of(X0,Y),
      forall(member(Xn,Xs), subclass_of(Xn,Y)) ),
    Sups),
  % yield most specific shared super class
  % FIXME: this could create problems for equivalentClass axioms,
  %           as there you have A subclassOf B AND B subclassOf A.
  %           then this would not yield anything.
  member(Sup,Sups),
  forall(
    ( member(X,Sups), Sup \= X ),
    ( \+ subclass_of(X,Sup) )
  ),
  % there is only one
  !.

		 /*******************************
		 *	    PROPERTY RANGE     		*
		 *******************************/

%% owl_property_range(?Cls,?P,?Range,+Scope) is nondet.
%
% Find the range of a property for instances of some class,
% meaning that any value must be an instance of that range.
%
owl_property_range(Cls,P,Range) :-
  var(Cls),!,
  is_class(Cls),
  owl_property_range(Cls,P,Range).

owl_property_range(Cls,P,Range) :-
  var(P),!,
  is_object_property(P),
  owl_property_range(Cls,P,Range).

owl_property_range(Cls,P,Range) :-
  owl_description(Cls,Descr),
  % get list of inferred ranges.
  % the meaning is that the range is the intersection 
  % of these classes.
  setof(R,
    owl_property_range1(Cls,P,R),
    Ranges),
  member(Range,Ranges),
  % However, the list may contain redundant entries.
  % Only yield the most specific ones.
  forall(
    ( member(X, Types), Range \= X ),
    ( \+ subclass_of(X,Range) )
  ).

%%
% TODO: rather use subclass_of (not owl_property_range)
%           to do recursion here to exploit caching?
%           (also for cardinality case below)
%
owl_property_range1(_,P,Range) :-
  has_property_range(P,Range).

owl_property_range1(only(P,Range),P,Range).

owl_property_range1(some(P0,Cls),P1,Range) :-
  owl_property_range_some(P0,Cls,P1,Range).

owl_property_range1(min(P0,Cls,Min),P1,Range) :-
  Min>0, owl_property_range_some(P0,Cls,P1,Range).

owl_property_range1(exactly(P0,Cls,Count),P1,Range) :-
  Count>0, owl_property_range_some(P0,Cls,P1,Range).

owl_property_range1(union_of(Set),P,Range) :-
  % find range constraints from members of Set
  findall(R,
    ( member(Cls,Set),
      owl_property_range(Cls,P,R) ),
    Ranges),
  % find most specific shared super class
  common_supclass_(Ranges,Range).

owl_property_range1(intersection_of(Set),P,Range) :-
  % find range constraints from members of Set
  member(Cls,Set),
  owl_property_range(Cls,P,Range).

owl_property_range1(class(Cls),P,Range) :-
  % get range constraints from super-classes of Cls
  subclass_of(Cls,Sup,Scope), Sup\=Cls,
  owl_property_range(Sup,P,Range).

%%
owl_property_range_some(P0,Cls,P1,Range) :-
  % check if restricted class has range restriction for inverse property `P0`,
  % and check if the inferred class description has a range restriction
  % for `P1`.
  has_inverse_property(P0, Pi),
  owl_property_range(Cls,      Pi,Pi_range),
  owl_property_range(Pi_range, P1,   Range).

		 /*******************************
		 *	    PROPERTY CARDINALITY    *
		 *******************************/

%% owl_property_cardinality(?Cls,?P,?Range,?Min,?Max,+Scope) is nondet.
%
% Find minimum and maximum cardinality of typed
% property values for instances of some class.
%
owl_property_cardinality(Cls,P,Range,Min,Max) :-
  var(P),!,
  is_object_property(P),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality(Cls,P,Range,Min,Max) :-
  var(Cls),!,
  has_property_domain(P,Cls),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality(Cls,P,Range,Min,Max) :-
  var(Range),!,
  owl_property_range(Cls,P,Range),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality(Cls,P,Range,Min,Max) :-
  owl_description(Cls,Descr),
  findall((Min0,Max0),
    owl_property_cardinality1(Descr,P,Range,Min0,Max0),
    Cards),
  % get the most constrained min/max values
  cardinality_all_of(Cards,[(Min,Max)]).

%%
owl_property_cardinality1(_Cls,P,_Range,0,1) :-
  is_functional_property(P).

owl_property_cardinality1(some(P0,Cls),P,Range,1,inf) :-
  subproperty_of(P0,P),
  subclass_of(Cls,Range).

owl_property_cardinality1(min(P0,Cls,Min),P,Range,Min,inf) :-
  subproperty_of(P0,P),
  subclass_of(Cls,Range).

owl_property_cardinality1(max(P0,Cls,Max),P,Range,0,Max) :-
  subproperty_of(P0,P),
  subclass_of(Cls,Range).

owl_property_cardinality1(exactly(P0,Cls,Count),P,Range,Count,Count) :-
  subproperty_of(P0,P),
  subclass_of(Cls,Range).

owl_property_cardinality1(union_of(Set),P,Range,Min,Max) :-
  % find cardinality constraints from members of Set
  findall((Min0,Max0),
    ( member(Cls,Set),
      owl_property_cardinality(Cls,P,R,Min0,Max0) ),
    Cards),
  % get the least constrained min/max values
  cardinality_one_of(Cards,[(Min,Max)]).

owl_property_cardinality1(intersection_of(Set),P,Range,Min,Max) :-
  % yield each cardinality constraint from member of the intersection
  member(Cls,Set),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality1(class(Cls),P,Range,Min,Max) :-
  % get cardinaity constraints from super-classes of Cls
  % FIXME: inf loop for equivalentClass
  subclass_of(Cls,Sup), Sup\=Cls,
  owl_description(Sup,Descr),
  owl_property_cardinality1(Descr,P,Range,Min,Max).

%%
cardinality_all_of([], []).
cardinality_all_of([(Min1, Max1),(Min2, Max2)|T], Set) :- !,
  cardinality_max(Min1, Min2, Min),
  cardinality_min(Max1, Max2, Max),
  cardinality_all_of([(Min, Max)|T], Set).
cardinality_all_of([H|T0], [H|T]) :- cardinality_all_of(T0, T).

%%
cardinality_one_of([], []).
cardinality_one_of([(Min1, Max1),(Min2, Max2)|T], Set) :- !,
  cardinality_min(Min1, Min2, Min),
  cardinality_max(Max1, Max2, Max),
  cardinality_one_of([(Min, Max)|T], Set).
cardinality_one_of([H|T0], [H|T]) :- cardinality_one_of(T0, T).

%%
cardinality_min(inf, X, X) :- !.
cardinality_min(X, inf, X) :- !.
cardinality_min(X1, X2, X) :- X is min(X1, X2).

%%
cardinality_max(inf, X, inf) :- !.
cardinality_max(X, inf, inf) :- !.
cardinality_max(X1, X2, X) :- X is max(X1, X2).
