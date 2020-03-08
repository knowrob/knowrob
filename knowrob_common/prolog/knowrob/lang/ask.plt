
:- begin_tests('knowrob/lang/ask').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/lang/ask')).

:- owl_parser:owl_parse('package://knowrob_common/owl/test_comp_temporal.owl').

:- rdf_db:rdf_register_ns(test_comp_temporal, 'http://knowrob.org/kb/test_comp_temporal.owl#', [keep(true)]).

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

:- end_tests('knowrob/lang/ask').
