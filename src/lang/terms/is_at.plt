
:- begin_tests('lang/terms/is_at').

test(lang_is_at) :-
  fail.

%:- use_module(library('lists')).
%:- use_module(library('random')).
%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/owl')).

%:- use_module(library('knowrob/lang/ask')).
%:- use_module(library('knowrob/lang/tell')).
%:- use_module(library('knowrob/comp/object_pose')).
%:- use_module(library('knowrob/model/Object')).

%:- use_module(library('knowrob/memory')).
%:- use_module(library('knowrob/beliefstate')).

%:- owl_parser:owl_parse('package://knowrob_household/owl/kitchen.owl').
%:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).

%:- dynamic test_object/1.

%:- mem_init, mem_drop.

%test(belief_new_object) :-
  %belief_new_object(knowrob:'Cup', Cup),
  %rdfs_individual_of(Cup, knowrob:'Cup'),
  %rdfs_individual_of(Cup, owl:'NamedIndividual'),
  %assertz(test_object(Cup)).

%test(belief_at_update) :-
  %test_object(Cup),
  %belief_at_update(Cup, ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]]),
  %current_object_pose(Cup, _), !.
  
%test(belief_at_update2) :-
  %test_object(Cup),
  %object_frame_name(Cup, F1),
  %belief_perceived_at(knowrob:'Cup', [F1,_,[1.0,1.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup2),
  %current_object_pose(Cup2,[F1,_,_,_]), !.

%test('belief_at_location(equal)') :-
  %belief_existing_object_at(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  %test_object(Cup), !.

%test('belief_at_location(fails)', [fail]) :-
  %belief_existing_object_at(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  %test_object(Cup), !.

%test('belief_at_location(nearby)') :-
  %belief_existing_object_at(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], 0.5, Cup),
  %test_object(Cup), !.

%test(belief_at_update_class) :-
  %test_object(Cup),
  %belief_perceived_at(knowrob:'Milk', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  %\+ rdfs_individual_of(Cup, knowrob:'Cup'),
  %kb_type_of(Cup, knowrob:'Milk'), !.

%test(belief_at_update_class2) :-
  %test_object(Cup),
  %belief_perceived_at(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], 0.0, Cup),
  %kb_type_of(Cup, knowrob:'Cup'),
  %\+ kb_type_of(Cup, knowrob:'Milk'), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests('lang/terms/is_at').
