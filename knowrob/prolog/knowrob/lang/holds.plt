
:- begin_tests('knowrob/lang/holds').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/lang/holds')).

:- owl_parser:owl_parse('package://knowrob/owl/test_comp_temporal.owl').

:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

test('holds(object_var)', [nondet]) :-
  holds(dul:hasConstituent(test_comp_temporal:'Long', X)),
  rdf_equal(X, test_comp_temporal:'Short2').

test('holds(data_var)', [nondet]) :-
  holds(knowrob:startTime(test_comp_temporal:'Short1', Stamp)),
  number(Stamp).
  
test('holds(property_var)', [nondet]) :-
  holds(test_comp_temporal:'Short1',P,_),
  rdf_equal(P, knowrob:startTime).

:- end_tests('knowrob/lang/holds').
