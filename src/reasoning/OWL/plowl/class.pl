:- module(plowl_class,
    [ owl_subclass_of(r,t),
      owl_property_range(r,r,t),
      owl_property_cardinality(r,r,t,?,?)
    ]).
/** <module> Reasoning about OWL classes.

@author Daniel Beßler
*/

:- use_module(library('model/OWL'),
    [ has_description/2,
      is_class/1,
      is_object_property/1,
      is_functional_property/1,
      has_equivalent_class/2,
      has_inverse_property/2
    ]).
:- use_module(library('model/RDFS'),
    [ has_domain/2,
      has_range/2
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
%owl_subclass_of(Sub,Sup) :-
  %% TODO: this clause seems displaced, but is not covered otherwise.
  %ground(Sub),
  %has_description(Sub,intersection_of(List)),!,
  %once((
    %member(Sub0,List),
    %(Sub0=Sup ; subclass_of(Sub0,Sup))
  %)).

owl_subclass_of(Sub,Sup) :-
  var(Sup),!,
  owl_subclass_of1(Sub,class(Sup)).

owl_subclass_of(Sub,Sup) :-
  has_description(Sup,Descr),
  owl_subclass_of1(Sub,Descr).

owl_subclass_of1(Sub,only(P,Range)) :-
  owl_subclass_of_only(Sub,P,Range).

owl_subclass_of1(Sub,some(P,Range)) :-
  % TODO: rather only generate min-subclasses?
  % once((ground(P);ground(Range))),
  subclass_of(Sub,min(P,Min,Range)), Min>0.

owl_subclass_of1(Sub,min(P,Min,Range)) :-
  ( var(Min) ->
    ( owl_property_cardinality(Sub,P,Range,Min,_) );
    ( owl_property_cardinality(Sub,P,Range,Min,_), Min>0 )
  ).

owl_subclass_of1(Sub,max(P,Max,Range)) :-
  ( var(Max) ->
    ( owl_property_cardinality(Sub,P,Range,_,Max) );
    ( owl_property_cardinality(Sub,P,Range,_,Max), Max \= inf )
  ).

owl_subclass_of1(Sub,exactly(P,Card,Range)) :-
  owl_property_cardinality(Sub,P,Range,Card,Card).
  
owl_subclass_of1(Sub,Sup) :-
  owl_subclass_of_equivalent(Sub,Sup).

%%
owl_subclass_of_equivalent(Sub,Descr) :-
  ground(Sub),!,
  has_equivalent_class(Sub,EQ),
  has_description(EQ,Descr).

owl_subclass_of_equivalent(Sub,class(EQ)) :-
  ground(EQ),!,
  has_equivalent_class(EQ,EQ0),
  has_description(EQ0,class(Sub)).

owl_subclass_of_equivalent(Sub,Descr) :-
  is_class(Sub),
  owl_subclass_of_equivalent(Sub,Descr).

%%
owl_subclass_of_only(Sub,P,Range) :-
  var(P),!, % FIXME: revise range
  is_object_property(P),
  owl_subclass_of_only(Sub,P,Range).

owl_subclass_of_only(Sub,P,Range) :-
  ground(Range),!, % FIXME: revise range
  forall(
    owl_property_range(Sub,P,R0),
    subclass_of(R0,Range)).

owl_subclass_of_only(Sub,P,Range) :-
  % var(Range), % FIXME: revise range
  findall(R0,
    owl_property_range(Sub,P,R0),
    Ranges),
  % find most specific shared super classes
  common_supclass_(Ranges,Range).

%%
common_supclass_([X0|Xs],Sup) :-
  % find shared super classes
  findall(Y,
    ( transitive(subclass_of_eq(X0,Y)),
      forall(member(Xn,Xs), subclass_of_eq(Xn,Y)) ),
    Sups),
  % yield most specific shared super class
  member(Sup,Sups),
  forall(
    ( member(X,Sups) ),
    ( \+ subclass_of(X,Sup) )
  ),
  % there is only one (not true if equvalent classes are included)
  !.

		 /*******************************
		 *	    PROPERTY RANGE     		*
		 *******************************/

%% owl_property_range(?Cls,?P,?Range) is nondet.
%
% Find the range of a property for instances of some class,
% meaning that any value must be an instance of that range.
%
owl_property_range(Cls,P,Range) :-
  var(Cls),!,
  is_class(Cls), % TODO: do not iterate over all
  owl_property_range(Cls,P,Range).

owl_property_range(Cls,P,Range) :-
  var(P),!,
  is_object_property(P), % TODO: do not iterate over all
  owl_property_range(Cls,P,Range).

owl_property_range(Cls,P,Range) :-
  has_description(Cls,Descr),
  % get list of inferred ranges.
  % the meaning is that the range is the intersection 
  % of these classes.
  setof(R,
    owl_property_range1(Descr,P,R),
    Ranges),
  member(Range,Ranges),
  % However, the list may contain redundant entries.
  % Only yield the most specific ones.
  forall(
    ( member(X, Ranges), Range \= X ),
    ( \+ subclass_of(X,Range) )
  ).

%%
% TODO: rather use subclass_of (not owl_property_range)
%           to do recursion here to exploit caching?
%           (also for cardinality case below)
%
owl_property_range1(_,P,Range) :-
  has_range(P,Range).

owl_property_range1(only(P,Range),P,Range).

owl_property_range1(some(P0,Cls),P1,Range) :-
  owl_property_range_some(P0,Cls,P1,Range).

owl_property_range1(min(P0,Min,Cls),P1,Range) :-
  Min>0, owl_property_range_some(P0,Cls,P1,Range).

owl_property_range1(exactly(P0,Count,Cls),P1,Range) :-
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
  subclass_of(Cls,Sup),
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

%% owl_property_cardinality(?Cls,?P,?Range,?Min,?Max) is nondet.
%
% Find minimum and maximum cardinality of typed
% property values for instances of some class.
%
owl_property_cardinality(Cls,P,Range,Min,Max) :-
  var(Cls),!,
  % try to at least limit Cls to sub-classes of
  % the domain of P (if P given)
  ( ground(P) ->
    ( has_domain(P,Domain), subclass_of_eq(Cls,Domain) );
    ( is_class(Cls) )
  ),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality(Cls,P,Range,Min,Max) :-
  has_description(Cls,Descr),
  ( owl_property_cardinality0(Descr,P,Range,Cards)
  -> true             % some constraint was found
  ;  Cards=[(0,inf)]  % no constraints
  ),
  % TODO: handle functional properties here
  %( is_functional_property(P) -> Cards0=[(0,1)|Cards] ; Cards0=Cards ),
  Cards0=Cards,
  % get the most constrained min/max values
  cardinality_all_of(Cards0,[(Min,Max)]).

%%
owl_property_cardinality0(Descr,P,Range,Cards) :-
  var(P),var(Range),!,
  findall([P0,R0,(Min0,Max0)],
    owl_property_cardinality1(Descr,P0,R0,Min0,Max0,[]),
    CardsPR),
  cardinality_bind_pr_(CardsPR,P,Range,Cards).

owl_property_cardinality0(Descr,P,Range,Cards) :-
  var(P),!,
  findall([P0,(Min0,Max0)],
    owl_property_cardinality1(Descr,P0,Range,Min0,Max0,[]),
    CardsP),
  cardinality_bind_p_(CardsP,P,Cards).

owl_property_cardinality0(Descr,P,Range,Cards) :-
  var(Range),!,
  findall([R0,(Min0,Max0)],
    owl_property_cardinality1(Descr,P,R0,Min0,Max0,[]),
    CardsR),
  cardinality_bind_r_(CardsR,Range,Cards).

owl_property_cardinality0(Descr,P,Range,Cards) :-
  findall((Min0,Max0),
    owl_property_cardinality1(Descr,P,Range,Min0,Max0,[]),
    Cards).

%%
owl_property_cardinality1(some(P0,Range0),P,Range,1,inf,_) :-
  ( var(P)     -> P=P0         ; subproperty_of(P0,P) ),
  ( var(Range) -> Range=Range0 ; subclass_of(Range0,Range) ).

owl_property_cardinality1(min(P0,Min,Range0),P,Range,Min,inf,_) :-
  ( var(P)     -> P=P0         ; subproperty_of(P0,P) ),
  ( var(Range) -> Range=Range0 ; subclass_of(Range0,Range) ).

owl_property_cardinality1(max(P0,Max,Range0),P,Range,0,Max,_) :-
  % NOTE: max cardinality is inherited from super-classes/properties
  %         to their children while min cardinality is inherited the other way.
  ( var(P)     -> P=P0         ; subproperty_of(P,P0) ),
  ( var(Range) -> Range=Range0 ; subclass_of(Range,Range0) ).

owl_property_cardinality1(exactly(P0,Count,Range0),P,Range,Count,Count,_) :-
  ( var(P)     -> P=P0         ; subproperty_of(P0,P) ),
  ( var(Range) -> Range=Range0 ; subclass_of(Range0,Range) ).

owl_property_cardinality1(union_of(Set),P,Range,Min,Max,_) :-
  % find cardinality constraints from members of Set
  cardinality_of_union_(Set,P,Range,Cards),
  % get the least constrained min/max values
  cardinality_one_of(Cards,[(Min,Max)]).

owl_property_cardinality1(intersection_of(Set),P,Range,Min,Max,_) :-
  % yield each cardinality constraint from member of the intersection
  member(Cls,Set),
  owl_property_cardinality(Cls,P,Range,Min,Max).

owl_property_cardinality1(class(Cls),P,Range,Min,Max,Visited) :-
  % get cardinaity constraints from super-classes of Cls
  subclass_of(Cls,Sup),
  \+ memberchk(Sup,Visited),
  has_description(Sup,Descr),
  owl_property_cardinality1(Descr,P,Range,Min,Max,[Cls|Visited]).

%%
cardinality_of_union_(Union,P,Range,Cards) :-
  var(P),var(Range),!,
  findall([P0,R0,(Min0,Max0)],
    ( member(Cls,Union),
      owl_property_cardinality(Cls,P0,R0,Min0,Max0) ),
    CardsPR),
  cardinality_bind_pr_(CardsPR,P,Range,Cards).

cardinality_of_union_(Union,P,Range,Cards) :-
  var(P),!,
  findall([P0,(Min0,Max0)],
    ( member(Cls,Union),
      owl_property_cardinality(Cls,P0,Range,Min0,Max0) ),
    CardsP),
  cardinality_bind_p_(CardsP,P,Cards).

cardinality_of_union_(Union,P,Range,Cards) :-
  var(Range),!,
  findall([R0,(Min0,Max0)],
    ( member(Cls,Union),
      owl_property_cardinality(Cls,P,R0,Min0,Max0) ),
    CardsR),
  cardinality_bind_r_(CardsR,Range,Cards).

cardinality_of_union_(Union,P,Range,Cards) :-
  findall((Min0,Max0),
    ( member(Cls,Union),
      owl_property_cardinality(Cls,P,Range,Min0,Max0) ),
    Cards).

%%
cardinality_bind_pr_(CardsPR,P,R,Cards) :-
  % first bind property
  setof(P1, member([P1,_,_],CardsPR), Properties),
  member(P,Properties),
  % second bind range
  setof(R1, member([P,R1,_],CardsPR), Ranges),
  member(R,Ranges),
  % finally get cardinality tuples
  findall((Min1,Max1), (
    member([P2,R2,(Min0,Max0)],CardsPR),
    ( (subproperty_of(P2,P),subclass_of(R2,R)) -> Min1=Min0 ; Min1=0 ),
    ( (subproperty_of(P,P2),subclass_of(R,R2)) -> Max1=Max0 ; Max1=inf )
  ), Cards).

cardinality_bind_r_(CardsR,R,Cards) :-
  % first bind range
  setof(R1, member([R1,_],CardsR), Ranges),
  member(R,Ranges),
  % finally get cardinality tuples
  findall((Min1,Max1), (
    member([R2,(Min0,Max0)],CardsR),
    ( subclass_of(R2,R) -> Min1=Min0 ; Min1=0 ),
    ( subclass_of(R,R2) -> Max1=Max0 ; Max1=inf )
  ), Cards).

cardinality_bind_p_(CardsP,P,Cards) :-
  % first bind property
  setof(P1, member([P1,_],CardsP), Properties),
  member(P,Properties),
  % finally get cardinality tuples
  findall((Min1,Max1), (
    member([P2,(Min0,Max0)],CardsP),
    ( subproperty_of(P2,P) -> Min1=Min0 ; Min1=0 ),
    ( subproperty_of(P,P2) -> Max1=Max0 ; Max1=inf )
  ), Cards).

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
cardinality_max(inf, _, inf) :- !.
cardinality_max(_, inf, inf) :- !.
cardinality_max(X1, X2, X) :- X is max(X1, X2).
