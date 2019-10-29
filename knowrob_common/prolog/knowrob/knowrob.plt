
:- begin_tests('knowrob/knowrob').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/knowrob')).

:- owl_parser:owl_parse('package://knowrob_common/owl/test_comp_temporal.owl').

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

%%%%%%%%%%%%%%%%%
%% test kb_triple
%%%%%%%%%%%%%%%%%

test('kb_triple(object_var)') :-
  kb_triple(test_comp_temporal:'Long', dul:hasConstituent, test_comp_temporal:'Short2').

test('kb_triple(object_var,ungrounded)', [nondet]) :-
  kb_triple(test_comp_temporal:'Long', dul:hasConstituent, X),
  rdf_equal(X, test_comp_temporal:'Short2').

test('kb_triple(data_var)') :-
  kb_triple(test_comp_temporal:'Short1', knowrob:startTime, 1377777000).

test('kb_triple(data_var,ungrounded)', [nondet]) :-
  kb_triple(test_comp_temporal:'Short1', knowrob:startTime, Stamp),
  number(Stamp).
  
test('kb_triple(property_var)', [nondet]) :-
  kb_triple(test_comp_temporal:'Short1', P, _),
  rdf_equal(P, knowrob:startTime).

%%%%%%%%%%%%%%%%%
%% test holds
%%%%%%%%%%%%%%%%%

test('holds(object_var)', [nondet]) :-
  holds(dul:hasConstituent(test_comp_temporal:'Long', X)),
  rdf_equal(X, test_comp_temporal:'Short2').

test('holds(data_var)', [nondet]) :-
  holds(knowrob:startTime(test_comp_temporal:'Short1', Stamp)),
  number(Stamp).
  
test('holds(property_var)', [nondet]) :-
  holds(test_comp_temporal:'Short1',P,_),
  rdf_equal(P, knowrob:startTime).

%%%%%%%%%%%%%%%%%
%% test kb_classify
%%%%%%%%%%%%%%%%%

test('kb_classify(type)') :-
  kb_classify(test_comp_temporal:'Short1', dul:'Event').

test('kb_classify(all_values_from,grounded)') :-
  kb_classify(test_comp_temporal:'Long',
      restriction(dul:hasConstituent, all_values_from(dul:'Event'))).

test('kb_classify(some_values_from)') :-
  kb_classify(test_comp_temporal:'Long',
      restriction(dul:hasConstituent, some_values_from(dul:'Event'))).

%%%%%%%%%%%%%%%%%
%% test kb_type_of
%%%%%%%%%%%%%%%%%

test('kb_type_of(grounded)') :-
  kb_type_of(test_comp_temporal:'Short1', dul:'Event').

test('kb_type_of(ungrounded)', [nondet]) :-
  kb_type_of(test_comp_temporal:'Short1', X),
  rdf_equal(X, dul:'Event').

test('kb_type_of(not_an_instance_of1)', [fail]) :-
  kb_type_of(test_comp_temporal:'Short1', dul:'Region').

test('kb_type_of(not_an_instance_of2)', [fail]) :-
  kb_type_of(test_comp_temporal:'Short1', dul:'Xfgerz4d').
  
%%%%%%%%%%%%%%%%%
%% test kb_rdf_pl
%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%
%% TODO test property_range
%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%
%% TODO test property_cardinality
%%%%%%%%%%%%%%%%%

:- end_tests('knowrob/knowrob').
