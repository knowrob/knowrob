:- module(plowl_individual,
    [ owl_individual_of(r,t,t),
      owl_satisfied_by(t,r,t)
    ]).
/** <module> Reasoning about OWL individuals.

@author Daniel Beßler
*/

:- use_module(library('model/RDFS'),
    [ has_range/2,
      has_domain/2
    ]).
:- use_module(library('model/OWL'),
    [ is_class/1,
      has_description/2
    ]).
:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2
    ]).
:- use_module('./property.pl',
    [ owl_cardinality/5
    ]).

%% owl_individual_of(?Subject,?Class,+Scope) is nondet.
%
% TODO: subject is instance_of not(A) if it is subclass of
%          a class disjoint to A? That seems to be not covered here.
%
owl_individual_of(Subject,Class,QScope->FScope) :-
  var(Class),!,
  ask(has_type(Subject,Type),QScope->FScope0),
  subclass_of(Class,Type),
  owl_individual_of(Subject,Class,QScope->FScope1),
  scope_intersect(FScope0,FScope1,FScope).

owl_individual_of(Subject,Class,Scope) :-
  % Subject is an individual of Class if some super-class
  % of Class is satisfied by Subject
  ground(Class),
  \+ (ground(Subject),has_type(Subject,Class)), % FIXME scope
  owl_satisfied_by(Class,Subject,Scope).

%owl_individual_of(Subject,Class,Scope) :-
  %% Subject is an individual of Class if Class is
  %% the range/domain of some Property of Subject
  %( owl_individual_of_range(Subject,Class,Scope);
    %owl_individual_of_domain(Subject,Class,Scope) ).

%%
%owl_individual_of_range(Subject,Class,Scope) :-
  %var(Subject),!,
  %has_range(P,Class),
  %ask(triple(_,P,Subject),Scope).

%owl_individual_of_range(Subject,Class,Scope) :-
  %ground(Subject),!,
  %ask(triple(_,P,Subject),Scope),
  %has_range(P,Class).

%%
%owl_individual_of_domain(Subject,Class,Scope) :-
  %var(Subject),!,
  %has_domain(P,Class),
  %ask(triple(Subject,P,_),Scope).

%owl_individual_of_domain(Subject,Class,Scope) :-
  %ground(Subject),!,
  %ask(triple(Subject,P,_),Scope),
  %has_domain(P,Class).

%% owl_satisfied_by(?Class,?Subject,+Scope) is nondet.
%
%
owl_satisfied_by(Class,_,_) :-
  var(Class),!,
  fail.

owl_satisfied_by(Class,Subject,Scope) :-
  has_description(Class,Descr),
  ( ground(Subject) ->
    ( owl_satisfied_by1(Descr,Subject,Scope,[]),! );
    ( owl_satisfied_by1(Descr,Subject,Scope,[]) )
  ).

%%
owl_satisfied_by1(class(Class),_,_,Visited) :-
  memberchk(Class,Visited),!,
  fail.

owl_satisfied_by1(class(Class),Subject,Scope,_) :-
  ask( has_type(Subject,Class), Scope ).
  
owl_satisfied_by1(class(Class),Subject,Scope,Visited) :-
  has_equivalent_class(Class,EQ),
  has_description(EQ,EQDescr),
  owl_satisfied_by1(EQDescr,Subject,Scope,[Class|Visited]).

%owl_satisfied_by1(not(Cls),Subject,Scope) :-
  %\+ owl_individual_of(Subject,Cls,Scope).

owl_satisfied_by1(intersection_of(Set),Subject,Scope,_) :-
  findall(instance_of(Subject,X),
          member(X,Set),
          Statements),
  ask(Statements,Scope).

owl_satisfied_by1(union_of(Set),Subject,Scope,_) :-
  member(X,Set),
  ask(instance_of(Subject,X),Scope).

owl_satisfied_by1(some(P,Cls),Subject,Scope,_) :-
  ask([ holds(Subject,P,O),
        instance_of(O,Cls) ], Scope).

owl_satisfied_by1(min(P,Min,Cls),Subject,Scope,_) :-
  owl_cardinality(Subject,P,Cls,Card,Scope),
  Card>=Min.

owl_satisfied_by1(max(P,Max,Cls),Subject,Scope,_) :-
  owl_cardinality(Subject,P,Cls,Card,Scope),
  Card=<Max.

owl_satisfied_by1(exactly(P,Count,Cls),Subject,Scope,_) :-
  owl_cardinality(Subject,P,Cls,Count,Scope).

owl_satisfied_by1(only(P,R),S,QS->FS,_) :-
  ( var(S) -> S0=_ ; S0=S ),
  ( var(P) -> P0=_ ; P0=P ),
  findall([S0,P0,V0,FS0],
    ask(holds(S0,P0,V0),QS->FS0),
    Facts),
  owl_only_satisfied_by_(S,P,R,Facts,QS->FS).

%%
owl_only_satisfied_by_(S,P,R,Facts,QS->FS) :-
  var(S),!,
  % bind subject
  setof(S0, member([S0,_,_,_],Facts), Subjects),
  member(S,Subjects),
  % and filter facts
  findall([S,P0,V0,FS0],
    member([S,P0,V0,FS0],Facts),
    Facts0),
  owl_only_satisfied_by_(S,P,R,Facts0,QS->FS).

owl_only_satisfied_by_(S,P,R,Facts,QS->FS) :-
  var(P),!,
  % bind property
  setof(P0, member([_,P0,_,_],Facts), Properties),
  member(P,Properties),
  % filter for values of P
  findall([S,P1,V0,FS0],
    ( member([S,P1,V0,FS0],Facts),
      subproperty_of(P0,P) ),
    Facts0),
  owl_only_satisfied_by_(S,P,R,Facts0,QS->FS).

owl_only_satisfied_by_(_S,_P,R,Facts,QS->FS) :-
  % ground([S,P]),
  setof(V0, member([_,_,V0,_],Facts), Values),
  findall(
    instance_of(V1,R),
    member(V1,Values),
    Statements),
  ask(Statements,QS->FS0),
  % intersect all fact scopes to obtain the scope
  % of the inferred fact.
  findall(FS1_list, (
    member(V2,Values),
    findall(FS1, member([_,_,V2,FS1],Facts), FS1_list)
  ), ValueScopes),
  scope_intersect_all_([[FS0]|ValueScopes],FS).

%%
scope_intersect_all_([X],X0) :-
  member(X0,X),!.
scope_intersect_all_([X,Y|Rest],Intersection) :-
  % only pick one scope of each list.
  % each item is an alternative scope matching the query.
  member(X0,X),
  member(Y0,Y),
  scope_intersect(X0,Y0,Z),
  scope_intersect_all_([[Z]|Rest],Intersection).
