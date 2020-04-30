:- module(plowl_property,
    [ owl_subproperty_of(r,r),
      owl_has(r,r,r,t),
      owl_cardinality(r,r,r,?,t)
    ]).
/** <module> TODO ...

@author Daniel Beßler
*/

:- use_module(library('db/tripledb'),
    [ tripledb_subproperty_of/2,
      tripledb_ask/4
    ]).
:- use_module(library('model/RDFS'),
    [ is_property/2
    ]).
:- use_module(library('model/OWL'),
    [ is_individual/1,
      is_symmetric_property/1,
      is_transitive_property/1,
      has_inverse_property/2,
      has_property_chain/2,
      same_as/2
    ]).
:- use_module(library('lang/terms/is_a'),
    [ subclass_of/2,
      instance_of/3
    ]).

% TODO look into how FScope is handled

%% owl_subproperty_of(?Sub,?Super) is nondet.
%
% Infer the rdfs:subProperty relation.
%
owl_subproperty_of(Sub,Sup) :-
  ground([Sub,Sup]),!,
  has_inverse_property(Sub,Sub_inv),
  has_inverse_property(Sup,Sup_inv),
  tripledb_subproperty_of(Sub_inv,Sup_inv).

owl_subproperty_of(Sub,Sup) :-
  ground(Sub),!,
  has_inverse_property(Sub,Sub_inv),
  tripledb_subproperty_of(Sub_inv,Sup_inv),
  has_inverse_property(Sup,Sup_inv).

owl_subproperty_of(Sub,Sup) :-
  ground(Sup),!,
  has_inverse_property(Sup,Sup_inv),
  tripledb_subproperty_of(Sub_inv,Sup_inv),
  has_inverse_property(Sub,Sub_inv).

owl_subproperty_of(Sub,Sup) :-
  is_property(Sub),
  owl_subproperty_of(Sub,Sup).

%% owl_has(?S,?P,?O,+Scope) is nondet.
%
% True if this relation is specified or can be deduced using OWL
% inference rules.
%
owl_has(S,P,O,Scope) :-
  ground([S,O]),!,
  same_as(S,S1),
  same_as(O,O1),
  has_transitive_(S1, P, O1, Scope, [O1]).

owl_has(S,P,O,Scope) :-
  ground(S),!,
  same_as(S,S1),
  has_transitive_(S1, P, O1, Scope, [O1]),
  same_as(O1,O).

owl_has(S,P,O,Scope) :-
  ground(O),!,
  same_as(O,O1),
  has_transitive_(S1, P, O1, Scope, [O1]),
  same_as(S1,S).

owl_has(S,P,O,Scope) :-
  has_transitive_(S1, P, O1, Scope, [O1]),
  same_as(S1,S),
  same_as(O1,O).

%%
has_transitive_(S,P,O,Scope,_) :-
  has_symmetric_(S,P,O,Scope).

has_transitive_(S,P,O,Scope,Visited) :-
  is_transitive_property(P),
  has_symmetric_(S,P,O1,Scope),
  \+ memberchk(O1, Visited),
  has_transitive_(O1,P,O,Scope,[O1|Visited]).

%%
has_symmetric_(S,P,O,Scope) :-
  has_direct2_(S,P,O,Scope).

has_symmetric_(S,P,O,Scope) :-
  is_symmetric_property(P),
  has_direct2_(O,P,S,Scope).

has_symmetric_(S,P,O,Scope) :-
  has_inverse_property(P,Inverse),
  has_direct2_(O,Inverse,S,Scope).

%% Simplest branch: find an explicitly stored rdf triple (S, P, O)
has_direct2_(S,P,O,Scope) :-
  % TODO: do not yield this if S/P/O are the same as in the call of owl_has to avoid redundancy
  tripledb_ask(S,P,O,Scope).

%% If P is bound to an object property, see if any of its PropertyChain axioms is able to produce explicitly known triples.
%% ASSUMPTION: no circular PropertyChain axioms (example, P defined as A o B and A defined as P o B)
has_direct2_(S,P,O,Scope) :-
  has_property_chain(P,Chain),
  has_chain_(S,Chain,O,Scope).

% use has value restrictions
has_direct2_(S,P,O,Scope) :-
  instance_of(S,value(P,O),Scope).

%% 
has_chain_(S, Chain, O, Scope) :-
  ( ground(S) ->
    ( has_chain_S2O_(S, Chain, O, Scope) ) ;
    ( reverse(Chain,Reversed),
      has_chain_O2S_(O, Reversed, S, Scope) )
  ).

has_chain_S2O_(O, [], O, _).
has_chain_S2O_(S, [P|Rest], O, Scope) :-
  owl_has(S, P, Oi, Scope),
  has_chain_S2O_(Oi, Rest, O, Scope).

has_chain_O2S_(S, [], S, _).
has_chain_O2S_(O, [P|Rest], S, Scope) :-
  owl_has(Si, P, O, Scope),
  has_chain_O2S_(Si, Rest, S, Scope).

%% owl_has_cardinality(?S,?P,?Range,?Card,+Scope) is det.
%
% Card is the number of values with type Range linked
% to S via property P.
%
owl_cardinality(S,P,Range,Card,Scope) :-
  ground(S),!,
  owl_has_all(S,P,All,Scope),
  % FIXME: scoping below
  % make sure Range is grounded
  ( ground(Range) -> true ; (
    setof(R, (
       member(Val0,All),
       ask(instance_of(Val0,R),Scope)
    ), Ranges),
    member(Range,Ranges)
  )),
  % compute cardinality on Range
  findall(X, (
    member(X,All),
    ask(instance_of(X,Range),Scope)
  ),Xs),
  length(Vs, Card).

owl_cardinality(S,P,Range,Card,Scope) :-
  ground(Range),!,
  % iterate over all instances of Range
  instance_of(S,Range,Scope),
  owl_has_cardinality(S,P,Range,Card,Scope).

owl_cardinality(S,P,Range,Card,Scope) :-
  % iterate over all OWL individuals
  is_individual(S),
  owl_has_cardinality(S,P,Range,Card,Scope).

%%
% FIXME scoping is difficult here
owl_has_all(S,P,All,Scope) :-
  ground([S,P]),!,
  setof(V, ask(holds(S,P,V), Scope), All0),
  owl_has_all1(All,All0).

owl_has_all(S,P,All,Scope) :-
  ground(S),!,
  % find all property-value pairs
  findall([P0,V0],
    ask(holds(S,P0,V0),Scope), Pairs),
  setof(P1, member([P1,_],Pairs), S_Properties),
  % for each property, yield values
  member(P,S_Properties),
  setof(V, member([P,V],Pairs), All0),
  owl_has_all1(All,All0).

owl_has_all(S,P,All,Scope) :-
  ground(P),!,
  % find all subject-value pairs
  findall([S0,V0],
    ask(holds(S0,P,V0),Scope), Pairs),
  setof(S1, member([S1,_],Pairs), P_subjects),
  % for each subject, yield values
  member(S,P_subjects),
  setof(V, member([S,V],Pairs), All0),
  owl_has_all1(All,All0).

owl_has_all(S,P,All,Scope) :-
  % iterate over all OWL individuals
  is_individual(S),
  owl_has_all(S,P,All,Scope).

%%
owl_has_all1(All_req,All_inferred) :-
  var(All_req),!,
  All_req=All_inferred.

owl_has_all1(All_req,All_inferred) :-
  forall( member(X,All_req),
          member(X,All_inferred) ).
