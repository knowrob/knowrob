
:- begin_tests('knowrob/model/Resource').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/model/Resource')).

:- owl_parser:owl_parse('package://knowrob/owl/test_comp_temporal.owl').
:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

%%%%%%%%%%%%%%%%%
%% test kb_resource
%%%%%%%%%%%%%%%%%

test('kb_resource(rdf:type)') :-
  kb_resource(rdf:type).

test('kb_resource(class)') :-
  kb_resource(owl:'Class').

test('kb_resource(instance)') :-
  kb_resource(test_comp_temporal:'Short1').

test('kb_resource(not_a_resource)', [fail]) :-
  kb_resource(xyz:'NotExistingClass').

:- end_tests('knowrob/model/Resource').
