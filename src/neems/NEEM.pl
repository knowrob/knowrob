:- module(mongolog_neems,
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
	        % FIXME avoid this elsewhere
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf:tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	),
	% publish object marker messages
	marker:republish.

:- use_module(library('mongolog/mongolog_test')).
:- begin_mongolog_tests(mongolog_neems,
		'owl/test/memory.owl',
		[ namespace('http://knowrob.org/kb/mem-test.owl#') ]).

:- use_module(library('semweb/rdf_db'), [ rdf_equal/2 ]).
:- use_module(library('mongolog/mongolog'), [ mongolog_call/1 ]).
:- use_module(library('scope')).
:- use_module('terms').
:- use_module('DUL').
:- use_module('SOMA').

:- dynamic test_episode/1,
           test_action/1.

test('is_episode') :-
	universal_scope(Scope),
	mongolog_call(new_iri(Episode,dul:'Situation')),
	mongolog_call(project(is_episode(Episode)), [scope(Scope)]),
	assert_true(ground(Episode)),
	assert_true(mongolog_call(is_episode(Episode))),
	assertz(test_episode(Episode)).

test('is_setting_for') :-
	universal_scope(Scope),
	test_episode(Episode),
	%% create an action
	mongolog_call(new_iri(Action,dul:'Action')),
	mongolog_call(project(is_action(Action)), [scope(Scope)]),
	assert_true(ground(Action)),
	assert_true(is_action(Action)),
	%% assert is_setting_for
	assert_false(is_setting_for(Episode,Action)),
	assert_true(mongolog_call(project(is_setting_for(Episode,Action)), [scope(Scope)])),
	assert_true(is_setting_for(Episode,Action)),
	%%
	assertz(test_action(Action)).

test('occurs') :-
	test_action(Action),
	assert_false(occurs(Action) during [24,464]),
	assert_true(mongolog_call(project(occurs(Action) during [24,464]))),
	assert_true(occurs(Action) during [24,464]).

test('executes_task') :-
	universal_scope(Scope),
	test_action(Action),
	%% state what task the action executes
	mongolog_call(new_iri(Task,test:'TestTask')),
	mongolog_call(project(has_type(Task,test:'TestTask')), [scope(Scope)]),
	assert_true(ground(Task)),

	semweb:sw_get_subgraphs(test,Subs),
	writeln(executes_task_SUBS_XXXXXXXX(Subs)),

	assert_true(is_task(Task)),
	%%
	assert_false(executes_task(Action,Task)),
	assert_true(mongolog_call(project(executes_task(Action,Task)), [scope(Scope)])),
	assert_true(executes_task(Action,Task)).

test('has_participant') :-
	universal_scope(Scope),
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	assert_false(has_participant(Action,Obj)),
	assert_true(mongolog_call(project(has_participant(Action,Obj)), [scope(Scope)])),
	assert_true(has_participant(Action,Obj)).

test('has_role') :-
	universal_scope(Scope),
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	mongolog_call(new_iri(Role,test:'ARole')),
	mongolog_call(project(has_type(Role,test:'ARole')), [scope(Scope)]),
	assert_false(has_role(Obj,Role) during Action),
	assert_true(mongolog_call(project(has_role(Obj,Role) during Action))),
	assert_true(has_role(Obj,Role) during Action).

test('has_transition') :-
	rdf_equal(test:'TestColor_0',Q),
	% TODO: assert a description of the transistion, then state that
	%          the episode satisfies this description.
	assert_true(mongolog_call(project(has_region(Q,test:'TEST_GREEN') until 300))),
	assert_true(mongolog_call(project(has_region(Q,test:'TEST_RED') since 300))),
	assert_true(has_region(Q,test:'TEST_RED')),
	assert_false(has_region(Q,test:'TEST_GREEN')),
	assert_true(has_region(Q,test:'TEST_GREEN') during [200,250]).

test('action_succeeded') :-
	universal_scope(Scope),
	test_action(Action),
	assert_false(action_succeeded(Action)),
	assert_true(mongolog_call(project(action_succeeded(Action)), [scope(Scope)])),
	assert_true(action_succeeded(Action)).

test('is_masterful') :-
	universal_scope(Scope),
	test_episode(Episode),
	%%
	mongolog_call(new_iri(Masterful)),
	mongolog_call(project(has_type(Masterful,test:'Masterful')), [scope(Scope)]),
	assert_true(ground(Masterful)),
	assert_true(is_diagnosis(Masterful)),
	%%
	assert_false(satisfies(Episode,Masterful)),
	assert_true(mongolog_call(project(satisfies(Episode,Masterful)), [scope(Scope)])),
	assert_true(satisfies(Episode,Masterful)).

:- end_mongolog_tests(mongolog_neems).
