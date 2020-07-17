:- use_module(library('db/tripledb_tests')).

:- use_module(library('lang/query'),
    [ synchronize/1 ]).

:- begin_tripledb_tests(
		'model_EASE_OBJ',
		'package://knowrob/owl/test/test_knowrob_objects.owl',
		[ namespace('http://knowrob.org/kb/test_knowrob_objects.owl#')
		]).

:- use_module('./OBJ.pl').

test(has_disposition_type, [nondet,fixme('hasDisposition is not inferred correctly')]) :-
	synchronize(subject(test:'Dishwasher1')),
	has_disposition_type(test:'Dishwasher1', _, ease_obj:'Insertion').

%test(disposition_trigger_type) :-
  %object_disposition(test_obj:'Dishwasher1', D, ease_obj:'Insertion'),!,
  %disposition_trigger_type(D, ease_obj:'Tableware').

%test(storage_place_for1) :-
  %storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup1').

%test(storage_place_for2) :-
  %storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup2').

%test(storage_place_for3) :-
  %storage_place_for(test_obj:'Dishwasher1', ease_obj:'Tableware').

%test(storage_place_for_because1) :-
  %storage_place_for_because(test_obj:'Dishwasher1',test_obj:'Cup1',X),
  %rdf_equal(X, ease_obj:'Tableware').

%test(object_dimensions) :-
  %object_dimensions(test_obj:'Handle1', 0.015, 0.015, 0.015).

%test(object_assert_color) :-
  %object_assert_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]),
  %object_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]).

%test(object_assert_dimensions) :-
  %object_assert_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12),
  %object_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12).

:- end_tripledb_tests('model_EASE_OBJ').
