:- module(plowl_property,
    [ owl_subproperty_of(r,r),
      owl_has(r,r,r,t),
      owl_cardinality(r,r,r,?,t)
    ]).
/** <module> Reasoning about OWL properties.

@author Daniel Beßler
*/

:- use_module(library('semweb/rdf_db'),
    [ rdf_equal/2
    ]).
:- use_module(library('model/OWL'),
    [ is_individual/1,
      is_symmetric_property/1,
      is_transitive_property/1,
      has_inverse_property/2,
      has_property_chain/2,
      same_as/2
    ]).
:- use_module(library('model/RDFS'),
    [ instance_of/2
    ]).

%% owl_subproperty_of(?Sub,?Super) is nondet.
%
% Infer the rdfs:subProperty relation.
%
owl_subproperty_of(Sub,Sup) :-
  ground([Sub,Sup]),!,
  has_inverse_property(Sub,Sub_inv),
  has_inverse_property(Sup,Sup_inv),
  ask(triple(Sub_inv,rdfs:subPropertyOf,Sup_inv)).

owl_subproperty_of(Sub,Sup) :-
  ground(Sub),!,
  has_inverse_property(Sub,Sub_inv),
  ask(triple(Sub_inv,rdfs:subPropertyOf,Sup_inv)),
  has_inverse_property(Sup,Sup_inv).

owl_subproperty_of(Sub,Sup) :-
  ground(Sup),!,
  has_inverse_property(Sup,Sup_inv),
  ask(triple(Sub_inv,rdfs:subPropertyOf,Sup_inv)),
  has_inverse_property(Sub,Sub_inv).

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
%
% TODO: handle var(P) case in a better way.
%         currently ground(P) is used in some clauses to avoid
%         iterating over all properties.
%
has_transitive_(S,P,O,Scope,_) :-
  has_symmetric_(S,P,O,Scope).

has_transitive_(S,P,O,QScope->FScope,Visited) :-
  ground(P),
  is_transitive_property(P),
  has_symmetric_(S,P,O1,QScope->FScope0),
  \+ memberchk(O1, Visited),
  has_transitive_(O1,P,O,QScope->FScope1,[O1|Visited]),
  scope_intersect(FScope0,FScope1,FScope).

%%
has_symmetric_(S,P,O,Scope) :-
  has_direct2_(S,P,O,Scope).

has_symmetric_(S,P,O,Scope) :-
  ground(P),
  is_symmetric_property(P),
  has_direct2_(O,P,S,Scope).

has_symmetric_(S,P,O,Scope) :-
  ground(P),
  has_inverse_property(P,Inverse),
  has_direct2_(O,Inverse,S,Scope).

%% Simplest branch: find an explicitly stored rdf triple (S, P, O)
has_direct2_(S,P,O,[Options,QScope]->FScope) :-
  % TODO: do not yield this if S/P/O are the same as in the call of owl_has to avoid redundancy
  ask(triple(S,P,O),QScope,FScope,Options),
  \+ rdf_equal(P,rdf:type).

%% If P is bound to an object property, see if any of its PropertyChain axioms is able to produce explicitly known triples.
%% ASSUMPTION: no circular PropertyChain axioms (example, P defined as A o B and A defined as P o B)
has_direct2_(S,P,O,Scope) :-
  ground(P),
  has_property_chain(P,Chain),
  has_chain_(S,Chain,O,Scope).

% use has value restrictions
has_direct2_(S,P,O,Scope) :-
  ask(instance_of(S,value(P,O)),Scope).

%% 
has_chain_(S, Chain, O, Scope) :-
  ( ground(S) ->
    ( has_chain_S2O_(S, Chain, O, Scope) ) ;
    ( reverse(Chain,Reversed),
      has_chain_O2S_(O, Reversed, S, Scope) )
  ).

has_chain_S2O_(O, [], O, _).
has_chain_S2O_(S, [P|Rest], O, QScope->FScope) :-
  owl_has(S, P, Oi, QScope->FScope0),
  has_chain_S2O_(Oi, Rest, O, QScope->FScope1),
  scope_intersect(FScope0,FScope1,FScope).

has_chain_O2S_(S, [], S, _).
has_chain_O2S_(O, [P|Rest], S, QScope->FScope) :-
  owl_has(Si, P, O, QScope->FScope0),
  has_chain_O2S_(Si, Rest, S, QScope->FScope1),
  scope_intersect(FScope0,FScope1,FScope).

%% owl_has_cardinality(?S,?P,?R,?Card,+Scope) is det.
%
% Card is the number of values with type R linked
% to S via property P.
%
owl_cardinality(S,P,R,Card,QScope->FScope) :-
  ( var(S) -> S0=_ ; S0=S ),
  ( var(P) -> P0=_ ; P0=P ),
  ( var(R) -> R0=_ ; R0=R ),
  findall([S0,P0,R0,V0,FS0],
    ask([ triple(S0,P0,V0), 
          instance_of(V0,class(R0))
        ], QScope->FS0),
    Facts),
  Facts \= [],
  owl_cardinality1_(S,P,R,Facts,Card,FScope).

%%
owl_cardinality1_(S,P,R,Facts,Card,FScope) :-
  var(S),!,
  % bind subject
  setof(S0, member([S0,_,_,_,_],Facts), Subjects),
  member(S,Subjects),
  % and filter facts
  findall([S,P0,R0,V0,FS0],
    member([S,P0,R0,V0,FS0],Facts),
    Facts0),
  owl_cardinality1_(S,P,R,Facts0,Card,FScope).

owl_cardinality1_(S,P,R,Facts,Card,FScope) :-
  var(P),!,
  % bind property
  setof(P0, member([_,P0,_,_,_],Facts), Properties),
  member(P,Properties),
  % filter for values of P
  findall([S,P1,R0,V0,FS0],
    ( member([S,P1,R0,V0,FS0],Facts),
      subproperty_of(P1,P) ),
    Facts0),
  owl_cardinality1_(S,P,R,Facts0,Card,FScope).

owl_cardinality1_(S,_P,R,Facts,Card,FScope) :-
  var(R),!,
  % bind range
  setof(R0, member([_,_,R0,_,_],Facts), Ranges),
  member(R,Ranges),
  % filter for values that are instance of R
  findall([S,P0,R,V0,FS0],
    member([S,P0,R,V0,FS0],Facts),
    Facts0),
  owl_cardinality2_(Facts0,Card,FScope).

owl_cardinality1_(_S,_P,_R,Facts,Card,FScope) :-
  % ground([S,P,R]),
  owl_cardinality2_(Facts,Card,FScope).

%%
owl_cardinality2_(Facts,Card0,FScope) :-
  setof(V0, member([_,_,_,V0,_],Facts), Values),
  % collect different scopes of the same value
  findall(FS0_list, (
    member(V1,Values),
    findall(X, member([_,_,_,V1,X],Facts), FS0_list)
  ), NestedScopes),
  owl_cardinality3_(NestedScopes,Card1,FScope),
  % unify cardinality value
  ( var(Card0) -> Card1>0 ; true ),
  Card0=Card1.
  
%%
owl_cardinality3_([], 0, FS->FS) :- !.
owl_cardinality3_([FirstList|Rest], Card, FS_in->FS_out) :-
  % try to intersect with FS_in and continue
  ( scopes_intersect_(FirstList,FS_in,FS0) *->
    ( Card0=1 );
    ( Card0=0, FS0=FS_in )
  ),
  % rescursion
  owl_cardinality3_(Rest,Card1,FS0->FS_out),
  Card is Card0 + Card1.

%%
scopes_intersect_(Scopes,In,Out) :-
  member(S,Scopes),
  scope_intersect(In,S,Out).
