:- use_module(library('db/tripledb_tests')).

:- use_module(library('lang/query'),
    [ synchronize/1, tell/1 ]).

:- begin_tripledb_tests(
		'model_SOMA_OBJ',
		'package://knowrob/owl/test/test_knowrob_objects.owl',
		[ namespace('http://knowrob.org/kb/test_knowrob_objects.owl#')
		]).

:- use_module('./OBJ.pl').

test(has_disposition_type, [nondet,fixme('hasDisposition is not inferred correctly')]) :-
	synchronize(subject(test:'Dishwasher1')),
	has_disposition_type(test:'Dishwasher1', _, soma:'Insertion').



%test(disposition_trigger_type) :-
  %object_disposition(test_obj:'Dishwasher1', D, soma:'Insertion'),!,
  %disposition_trigger_type(D, soma:'Tableware').

%test(storage_place_for1) :-
  %storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup1').

%test(storage_place_for2) :-
  %storage_place_for(test_obj:'Dishwasher1', test_obj:'Cup2').

%test(storage_place_for3) :-
  %storage_place_for(test_obj:'Dishwasher1', soma:'Tableware').

%test(storage_place_for_because1) :-
  %storage_place_for_because(test_obj:'Dishwasher1',test_obj:'Cup1',X),
  %rdf_equal(X, soma:'Tableware').

%test(object_dimensions) :-
  %object_dimensions(test_obj:'Handle1', 0.015, 0.015, 0.015).

%test(object_assert_color) :-
  %object_assert_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]),
  %object_color(test_obj:'Cup1', [0.3, 0.5, 0.6, 1]).

%test(object_assert_dimensions) :-
  %object_assert_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12),
  %object_dimensions(test_obj:'Cup2', 0.032, 0.032, 0.12).

test(assert_an_object_dimension) :-
  tell(is_physical_object(Object)),
  tell(object_dimensions(Object, 0.5, 0.8, 0.14)).

test('assert object color') :-
  tell(is_physical_object(Object)),
  tell(object_color_rgb(Object, [0.2,0.2,0.2,1.0])).

:- end_tripledb_tests('model_SOMA_OBJ').