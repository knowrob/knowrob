:- module(subgraph,
	[ load_graph_structure/0
	]).
/** <module> subgraph-of relationship between RDF graphs.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('mongodb/client'),
	[ mng_get_db/3,
	  mng_distinct_values/4
	]).

%% load_graph_structure is det.
%
% Avoid that there are any orphan graphs.
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
