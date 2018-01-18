
:- begin_tests(knowrob_beliefstate).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- use_module(library('lists')).
:- use_module(library('random')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).

:- dynamic test_object/1.

test(belief_new_object) :-
  belief_new_object(knowrob:'Cup', Cup),
  rdfs_individual_of(Cup, knowrob:'Cup'),
  rdfs_individual_of(Cup, owl:'NamedIndividual'),
  rdf_has(Cup, knowrob:frameName, _),
  assertz(test_object(Cup)).

test(belief_at_update) :-
  test_object(Cup),
  belief_at_update(Cup, ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]]),
  rdf_has(Cup, knowrob:'pose', _), !.

test(belief_at_location_equal) :-
  belief_existing_object_at(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  test_object(Cup), !.

test(belief_at_location_close1, [fail]) :-
  belief_existing_object_at(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  test_object(Cup), !.

test(belief_at_location_close2) :-
  belief_existing_object_at(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], 0.5, Cup),
  test_object(Cup), !.

test(belief_at_update_class) :-
  test_object(Cup),
  belief_perceived_at(knowrob:'Milk', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  \+ rdfs_individual_of(Cup, knowrob:'Cup'),
  rdfs_individual_of(Cup, knowrob:'Milk'), !.

test(belief_at_update_class2) :-
  test_object(Cup),
  belief_perceived_at(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  rdfs_individual_of(Cup, knowrob:'Cup'),
  \+ rdfs_individual_of(Cup, knowrob:'Milk'), !.

test(belief_temporal_part) :-
  test_object(Cup),
  holds( rdf:type(Cup,knowrob:'Cup'), [0.0,T2]),
  holds( rdf:type(Cup,knowrob:'Milk'), [T2,T3]),
  holds( rdf:type(Cup,knowrob:'Cup'), [T3]), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests(knowrob_beliefstate).
