
:- begin_tests('knowrob/comp/rdf_data').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/comp/rdf_data')).

:- owl_parser:owl_parse('package://knowrob/owl/test_comp_temporal.owl').

:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

test('kb_rdf_pl(individual)') :-
  kb_rdf_pl(_,test_comp_temporal:'Short1',test_comp_temporal:'Short1').

test('kb_rdf_pl(individual,ungrounded)') :-
  kb_rdf_pl(_,test_comp_temporal:'Short1',X),
  rdf_equal(X,test_comp_temporal:'Short1').

test('kb_rdf_pl(float)') :-
  kb_rdf_pl(_,literal(type(xsd:float,'6')),6).

test('kb_rdf_pl(long1)') :-
  kb_rdf_pl(dul:hasRegionDataValue,literal(type(xsd:long,'4')),4).

test('kb_rdf_pl(long2)') :-
  kb_rdf_pl(_,literal(type(xsd:long,'4')),4).

test('kb_rdf_pl(long3)') :-
  kb_rdf_pl(_,literal(type(xsd:long,'4')),X),
  X is 4.

test('kb_rdf_pl(float,pl ungrounded)') :-
  kb_rdf_pl(_,literal(type(xsd:float,'6')),X),
  X is 6.

test('kb_rdf_pl(float,rdf ungrounded1)') :-
  kb_rdf_pl(_,literal(type(xsd:float,X)),6),
  X = '6'.

:- end_tests('knowrob/comp/rdf_data').
