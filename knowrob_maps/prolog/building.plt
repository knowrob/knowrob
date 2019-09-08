
:- begin_tests(building).

:- use_module(library('semweb/owl_parser')).
:- use_module(library(building)).

:- owl_parser:owl_parse('package://knowrob_maps/owl/ccrl2_semantic_map.owl').

:- rdf_db:rdf_register_ns(ias_map, 'http://knowrob.org/kb/ias_semantic_map.owl#', [keep(true)]).

test(building) :-
  fail.

:- end_tests(building).

