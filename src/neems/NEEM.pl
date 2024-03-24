:- module(mongolog_neem,
    [ knowrob_load_neem/1
    ]).


%% knowrob_load_neem(+NEEM_id) is det.
%
% Configure KnowRob to use the DB associated to some NEEM,
% and initialize position data etc.
%
knowrob_load_neem(NEEM_id) :-
	% assign DB collection prefix
	set_setting(mng_client:collection_prefix, NEEM_id),
	% re-initialize the triple DB
	% this is important e.g. to establish triple graph hierarchy.
	% else we may get orphaned graphs.
	load_graph_structure,
	% load URDF files referred to in triple store
	urdf_init,
	% initialize position of each frame for tf publishing
	tf:tf_republish_clear,
	tf_mongo:tf_mng_lookup_all(InitialTransforms),
	forall(
	    (   member([Ref,Frame,Pos,Rot],InitialTransforms),
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf:tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	),
	% publish object marker messages
	marker:republish.

:- use_module(library('mongolog/mongolog_test')).
:- begin_mongolog_tests(mongolog_neem, 'owl/test/memory.owl').

:- use_module(library('semweb/rdf_db'), [ rdf_equal/2 ]).
:- use_module(library('semweb'), [ sw_register_prefix/2 ]).
:- use_module(library('mongolog/mongolog'),
        [ mongolog_call/1,
          mongolog_project/1 ]).
:- use_module('occurs').
:- use_module('DUL').
:- use_module('SOMA').

:- dynamic test_episode/1,
           test_action/1.

:- sw_register_prefix(test, 'http://knowrob.org/kb/mem-test.owl#').

test('is_episode') :-
	mongolog_call(new_iri(Episode,dul:'Situation')),
	mongolog_project(is_episode(Episode)),
	assert_true(ground(Episode)),
	assert_true(mongolog_call(is_episode(Episode))),
	assertz(test_episode(Episode)).

test('is_setting_for') :-
	test_episode(Episode),
	%% create an action
	mongolog_call(new_iri(Action,dul:'Action')),
	mongolog_project(is_action(Action)),
	assert_true(ground(Action)),
	assert_true(is_action(Action)),
	%% assert is_setting_for
	assert_false(is_setting_for(Episode,Action)),
	assert_true(mongolog_project(is_setting_for(Episode,Action))),
	assert_true(is_setting_for(Episode,Action)),
	%%
	assertz(test_action(Action)).

test('occurs') :-
	test_action(Action),
	assert_false(occurs(Action) during [24,464]),
	assert_true(mongolog_call(project(occurs(Action) during [24,464]))),
	assert_true(occurs(Action) during [24,464]).

test('executes_task') :-
	test_action(Action),
	%% state what task the action executes
	mongolog_call(new_iri(Task,test:'TestTask')),
	mongolog_project(has_type(Task,test:'TestTask')),
	assert_true(ground(Task)),
	assert_true(is_task(Task)),
	%%
	assert_false(executes_task(Action,Task)),
	assert_true(mongolog_project(executes_task(Action,Task))),
	assert_true(executes_task(Action,Task)).

test('has_participant') :-
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	assert_false(has_participant(Action,Obj)),
	assert_true(mongolog_project(has_participant(Action,Obj))),
	assert_true(has_participant(Action,Obj)).

test('has_role') :-
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	mongolog_call(new_iri(Role,test:'ARole')),
	mongolog_project(has_type(Role,test:'ARole')),
	assert_false(has_role(Obj,Role) during Action),
	assert_true(mongolog_call(project(has_role(Obj,Role) during Action))),
	assert_true(has_role(Obj,Role) during [24,464]),
	assert_true(has_role(Obj,Role) during Action).

test('has_transition') :-
	rdf_equal(test:'TestColor_0',Q),
	assert_true(mongolog_call(project(has_region(Q,test:'TEST_GREEN') until 300))),
	assert_true(mongolog_call(project(has_region(Q,test:'TEST_RED') since 300))),
	assert_true(has_region(Q,test:'TEST_RED')),
	assert_false(has_region(Q,test:'TEST_GREEN')),
	assert_true(has_region(Q,test:'TEST_GREEN') during [200,250]).

test('action_succeeded') :-
	test_action(Action),
	assert_false(action_succeeded(Action)),
	assert_true(mongolog_project(action_succeeded(Action))),
	assert_true(action_succeeded(Action)).

test('is_masterful') :-
	test_episode(Episode),
	%%
	mongolog_call(new_iri(Masterful)),
	mongolog_project(has_type(Masterful,test:'Masterful')),
	assert_true(ground(Masterful)),
	assert_true(is_diagnosis(Masterful)),
	%%
	assert_false(satisfies(Episode,Masterful)),
	assert_true(mongolog_project(satisfies(Episode,Masterful))),
	assert_true(satisfies(Episode,Masterful)).

:- end_mongolog_tests(mongolog_neem).
