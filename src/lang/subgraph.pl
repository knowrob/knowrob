:- module(subgraph,
	[ add_subgraph/2,
	  get_supgraphs/2,
	  get_subgraphs/2,
	  load_graph_structure/0
	]).
/** <module> subgraph-of relationship between RDF graphs.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('db/mongo/client'),
	[ mng_get_db/3,
	  mng_distinct_values/4
	]).

:- dynamic subgraph_of/2.

%%
% This is used to avoid that there are any orphan graphs in case
% of there are some ontologies loaded already into the triple DB
% when KnowRob is started.
%
load_graph_structure :-
	mng_get_db(DB, Coll, 'triples'),
	mng_distinct_values(DB, Coll, 'graph', Names),
	forall(
		member(NameString,Names),
		load_graph_structure1(NameString)
	).

load_graph_structure1(NameString) :-
	string_to_atom(NameString,Name),
	(	Name=user -> true ;
	(	add_subgraph(Name,common),
		add_subgraph(user,Name)
	)).

%% add_subgraph(+Sub,+Sup) is det.
%
% Adds the subgraph-of relation between two named graphs.
%
% @param Sub Name of the subgraph.
% @param Sup Name of the super-graph.
%
add_subgraph(Sub,Sup) :-
	get_supgraphs(Sup,SupGraphs),
	get_subgraphs(Sub,SubGraphs),
	forall(
		(	member(string(Sup_x),SupGraphs),
			member(string(Sub_x),SubGraphs)
		),
		assert_subgraph_of(Sub_x,Sup_x)
	).

%%
assert_subgraph_of(Sub,Sup) :- subgraph_of(Sub,Sup),!.
assert_subgraph_of(Sub,Sup) :- assertz(subgraph_of(Sub,Sup)).

%% get_supgraphs(+GraphName,-SupGraphs) is det.
%
% Get all super-graphs of a named graph. This minds transitivity of the relation.
%
% @param GraphName Name of a graph.
% @param SupGraphs List of super-graphs.
%
get_supgraphs(G,_) :-
	var(G),
	!.

get_supgraphs(G,Graphs) :-
	findall(string(X),
		(	X=G
		;	subgraph_of(G,X)
		),
		Graphs
	).

%% get_subgraphs(+GraphName,-SubGraphs) is det.
%
% Get all subgraphs of a named graph. This minds transitivity of the relation.
%
% @param GraphName Name of a graph.
% @param SubGraphs List of subgraphs.
%
get_subgraphs(G,Graphs) :-
	findall(string(X),
		(	X=G
		;	subgraph_of(X,G)
		),
		Graphs
	).
