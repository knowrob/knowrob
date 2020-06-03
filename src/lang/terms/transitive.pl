:- module(lang_transitive,
    [ transitive(t),      % +Query
      include_parents(t)  % +Query
    ]).
/** <module> Treat a query term as transitive relation.

@author Daniel Beßler
@license BSD
*/
    
:- use_module(library('db/scope'),
    [ scope_intersect/3 ]).

%%
:- rdf_meta(transitive_db_(t)).
transitive_db_(subclass_of(_,_)).
transitive_db_(subproperty_of(_,_)).
transitive_db_(triple(_,rdfs:subClassOf,_)).
transitive_db_(triple(_,rdfs:subPropertyOf,_)).

%% include_parents(+Query) is nondet.
%
% Calls a query but not only includes direct (specific) results
% but also more general ones in case of a taxonomical property 
% (type, subClassOf, subPropertyOf) is used in the query.
%
% @param A query term.
%
include_parents(Query) ?>
  call(Query, [options([include_parents(true)])]).

%% transitive(+Query) is nondet.
%
% Assumes transitivity of a 2-ary or 3-ary query term,
% meaning that if query holds for (A,B) and (B,C), then
% it also holds for (A,C).
% In the case of a 2-ary query term, the two arguments
% are assumed to be the subject and the object of the relation.
% In case of 3-ary queries, the object is the last argument,
% and the middle argument is a property.
%
% Note that transitivity of subClassOf and subPropertyOf is
% already handled on the DB level such that no sequential
% querying is needed for these cases.
%
% @param Query A query term.
%
transitive(Query) ?>
  { transitive_db_(Query),! },
  include_parents(Query).

transitive(Query) ?>
  options(Options),
  query_scope(QScope),
  fact_scope(FScope),
  { transitive_(Query,[Options,QScope]->FScope) }.

%%
transitive_(Query,Scope) :-
  Query=..[Functor|Args],
  ( Args=[S,O] -> P=_ ; Args=[S,P,O] ),
  ( ground(S) ->
    transitive_SO_(Functor,S,P,O,Scope);
    transitive_OS_(Functor,S,P,O,Scope) ).

%%
transitive_SO_(Functor,S,P,O,QScope->FScope) :-
  call_(Functor,S,P,X,QScope->FScope0),
  transitive_SO_1_(Functor,X,P,O,QScope,FScope0->FScope,[S]).

transitive_SO_1_(_,X,_,_,_,_,Visited) :-
  memberchk(X,Visited),!, fail.

transitive_SO_1_(_,O,_,O,_,FScope->FScope,_).

transitive_SO_1_(Functor,X0,P,O,QScope,FScope0->FScope,Visited) :-
  call_(Functor,X0,P,X1,QScope->FScope1),
  scope_intersect(FScope0,FScope1,FScope2),
  transitive_SO_1_(Functor,X1,P,O,QScope,FScope2->FScope,[X0|Visited]).

%%
transitive_OS_(Functor,S,P,O,QScope->FScope) :-
  call_(Functor,X,P,O,QScope->FScope0),
  transitive_OS_1_(Functor,S,P,X,QScope,FScope0->FScope,[O]).

transitive_OS_1_(_,_,_,X,_,_,Visited) :-
  memberchk(X,Visited),!, fail.

transitive_OS_1_(_,O,_,O,_,FScope->FScope,_).

transitive_OS_1_(Functor,S,P,X0,QScope,FScope0->FScope,Visited) :-
  call_(Functor,X1,P,X0,QScope->FScope1),
  scope_intersect(FScope0,FScope1,FScope2),
  transitive_OS_1_(Functor,S,P,X1,QScope,FScope2->FScope,[X0|Visited]).

%%
call_(Functor,S,P,O,Scope) :-
  ( var(P) ->
    Goal=..[Functor,S,O];
    Goal=..[Functor,S,P,O]
  ),
  ask(Goal,Scope).
