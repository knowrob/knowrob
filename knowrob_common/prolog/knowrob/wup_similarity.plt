
:- begin_tests('knowrob/wup_similarity').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/wup_similarity')).

test('rdf_wup_similarity1') :-
  rdf_wup_similarity(dul:'Object', dul:'PhysicalObject', Sim),
  Sim > 0.

test('rdf_wup_similarity2') :-
  rdf_wup_similarity(dul:'PhysicalObject', dul:'Object', Sim),
  rdf_wup_similarity(dul:'Object', dul:'PhysicalObject', Sim).

:- end_tests('knowrob/wup_similarity').
