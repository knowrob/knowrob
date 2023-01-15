
:- use_module(library('semweb/rdf_db'), [ rdf_meta/1 ]).

:- use_module(mongolog).
:- use_module(arithmetic).
:- use_module(atoms).
:- use_module(comparison).
:- use_module(context).
:- use_module(control).
:- use_module(database).
:- use_module(findall).
:- use_module(fluents).
:- use_module(lists).
:- use_module(meta).
:- use_module(projection).
:- use_module(sgml).
:- use_module(terms).
:- use_module(typecheck).
:- use_module(unification).
:- use_module(rules).

:- use_module(annotation).
:- use_module(triple).
:- use_module(semweb).
:- use_module(holds).
:- use_module(temporal).

:- rdf_meta(triple(t,t,t)).
% initialize hierachical organization of triple graphs
:- load_graph_structure.
% load common ontology
:- load_owl('http://www.ease-crc.org/ont/SOMA.owl', [ namespace(soma) ]).
