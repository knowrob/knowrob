:- module(plowl_individual,
    [ owl_individual_of(r,t,t),
      owl_satisfied_by(t,r,t)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('db/tripledb'),
    [ tripledb_type_of/3
    ]).
:- use_module(library('model/OWL'),
    [ is_class/1,
      has_property_range/2,
      has_property_domain/2,
      owl_description/2
    ]).
:- use_module(library('lang/terms/holds'),
    [ holds/4
    ]).
:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2,
      instance_of/3
    ]).
:- use_module('./property.pl',
    [ owl_cardinality/5
    ]).

% TODO SCOPING: need to propagate scope manually,
%        or call ask(...) to handle scope propagation in there.

%% owl_individual_of(?Subject,?Class,+Scope) is nondet.
%
%
owl_individual_of(Subject,Class,Scope) :-
  % Subject is an individual of Class if some super-class
  % of Class is satisfied by Subject
  owl_satisfied_by(Class,Subject,Scope).

owl_individual_of(Subject,Class,Scope) :-
  % Subject is an individual of Class if Class is
  % the range/domain of some Property of Subject
  owl_individual_of_range(Subject,Class,Scope);
  owl_individual_of_domain(Subject,Class,Scope).

%%
owl_individual_of_range(Subject,Class,Scope) :-
  var(Subject),!,
  has_property_range(P,Class),
  holds(_,P,Subject,Scope).

owl_individual_of_range(Subject,Class,Scope) :-
  holds(_,P,Subject,Scope),
  has_property_range(P,Class).

%%
owl_individual_of_domain(Subject,Class,Scope) :-
  var(Subject),!,
  has_property_domain(P,Class),
  holds(Subject,P,_,Scope).

owl_individual_of_domain(Subject,Class,Scope) :-
  holds(Subject,P,_,Scope),
  has_property_domain(P,Class).

%% owl_satisfied_by(?Class,?Subject,+Scope) is nondet.
%
%
owl_satisfied_by(Class,Subject,Scope) :-
  var(Class),!,
  is_class(Class),
  owl_satisfied_by(Class,Subject,Scope).

owl_satisfied_by(Class,Subject,Scope) :-
  owl_description(Class,Descr),
  ( ground(Subject) ->
    ( owl_satisfied_by1(Descr,Subject,Scope),! );
    ( owl_satisfied_by1(Descr,Subject,Scope) )
  ).

%%
owl_satisfied_by1(class(Class),Subject,Scope) :-
  tripledb_type_of(Subject,Class,Scope).

%%
owl_satisfied_by1(class(Class),Subject,Scope) :-
  subclass_of(Class,Sup), Sup\=Class,
  owl_description(Sup,Descr),
  owl_satisfied_by1(Descr,Subject,Scope).

%owl_satisfied_by1(complement_of(Cls),Subject,Scope) :-
  %\+ owl_individual_of(Subject,Cls,Scope).

owl_satisfied_by1(intersection_of(Set),Subject,Scope) :-
  forall(
    member(X,Set),
    owl_individual_of(Subject,X,Scope)).

owl_satisfied_by1(union_of(Set),Subject,Scope) :-
  member(X,Set),
  owl_individual_of(Subject,X,Scope).

owl_satisfied_by1(only(P,Cls),Subject,Scope) :-
  forall(
    holds(Subject,P,O,Scope),
    instance_of(O,Cls,Scope)).

owl_satisfied_by1(some(P,Cls),Subject,Scope) :-
  holds(Subject,P,O,Scope),
  instance_of(O,Cls,Scope).

owl_satisfied_by1(value(P,O),Subject,Scope) :-
  holds(Subject,P,O,Scope).

owl_satisfied_by1(min(P,Cls,Min),Subject,Scope) :-
  owl_cardinality(Subject,P,Cls,Card,Scope),
  Card>=Min.

owl_satisfied_by1(max(P,Cls,Max),Subject,Scope) :-
  owl_cardinality(Subject,P,Cls,Card,Scope),
  Card=<Max.

owl_satisfied_by1(exactly(P,Cls,Count),Subject,Scope) :-
  owl_cardinality(Subject,P,Cls,Count,Scope).
