:- module(subgraph,
    [ tripledb_add_subgraph/2,
      tripledb_get_supgraphs/2,
      tripledb_get_subgraphs/2
    ]).

:- dynamic subgraph_of_/2.

%%
%
tripledb_add_subgraph(Sub,Sup) :-
  tripledb_get_supgraphs(Sup,SupGraphs),
  tripledb_get_subgraphs(Sub,SubGraphs),
  forall(
    ( member(string(Sup_x),SupGraphs),
      member(string(Sub_x),SubGraphs) ),
    ( assert_subgraph_of_(Sub_x,Sup_x) )
  ).

%%
assert_subgraph_of_(Sub,Sup) :- subgraph_of_(Sub,Sup),!.
assert_subgraph_of_(Sub,Sup) :- assertz(subgraph_of_(Sub,Sup)).

%%
tripledb_get_supgraphs(G,_) :-
  var(G),!.
tripledb_get_supgraphs(G,Graphs) :-
  findall(string(X), (
    X=G; subgraph_of_(G,X)
  ),Graphs).

%%
tripledb_get_subgraphs(G,Graphs) :-
  findall(string(X), (
    X=G; subgraph_of_(X,G)
  ),Graphs).
