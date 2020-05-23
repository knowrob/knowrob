
:- begin_tests('metrics_WuPalmer').

:- use_module('./WuPalmer.pl').

test('rdf_wup_similarity1') :-
  rdf_wup_similarity(dul:'Object', dul:'PhysicalObject', Sim),
  Sim > 0.

test('rdf_wup_similarity2') :-
  rdf_wup_similarity(dul:'PhysicalObject', dul:'Object', Sim),
  rdf_wup_similarity(dul:'Object', dul:'PhysicalObject', Sim).

:- end_tests('metrics_WuPalmer').
