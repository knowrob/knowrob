:- module(model_NEEM,
    [
    ]).

:- use_module(library('semweb/rdf_db'), [ rdf_equal/2 ]).
:- use_module('terms').
 
:- begin_rdf_tests(model_NEEM,
		'package://knowrob/owl/test/memory.owl',
		[ namespace('http://knowrob.org/kb/mem-test.owl#')
		]).

:- dynamic test_episode/1,
           test_action/1.

test('is_episode') :-
	kb_call(new_iri(Episode,dul:'Situation')),
	kb_project(is_episode(Episode)),
	assert_true(ground(Episode)),
	assert_true(is_episode(Episode)),
	assertz(test_episode(Episode)).

test('is_setting_for') :-
	test_episode(Episode),
	%% create an action
	kb_call(new_iri(Action,dul:'Action')),
	kb_project(is_action(Action)),
	assert_true(ground(Action)),
	assert_true(is_action(Action)),
	%% assert is_setting_for
	assert_false(is_setting_for(Episode,Action)),
	assert_true(kb_project(is_setting_for(Episode,Action))),
	assert_true(is_setting_for(Episode,Action)),
	%%
	assertz(test_action(Action)).

test('occurs') :-
	test_action(Action),
	assert_false(occurs(Action) during [24,464]),
	assert_true(kb_project(occurs(Action) during [24,464])),
	assert_true(occurs(Action) during [24,464]).

test('executes_task') :-
	test_action(Action),
	%% state what task the action executes
	kb_call(new_iri(Task,test:'TestTask')),
	kb_project(has_type(Task,test:'TestTask')),
	assert_true(ground(Task)),
	assert_true(is_task(Task)),
	%%
	assert_false(executes_task(Action,Task)),
	assert_true(kb_project(executes_task(Action,Task))),
	assert_true(executes_task(Action,Task)).

test('has_participant') :-
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	assert_false(has_participant(Action,Obj)),
	assert_true(kb_project(has_participant(Action,Obj))),
	assert_true(has_participant(Action,Obj)).

test('has_role') :-
	test_action(Action),
	rdf_equal(test:'Substance_0',Obj),
	%%
	kb_call(new_iri(Role,test:'ARole')),
	kb_project(has_type(Role,test:'ARole')),
	assert_false(has_role(Obj,Role) during Action),
	assert_true(kb_project(has_role(Obj,Role) during Action)),
	assert_true(has_role(Obj,Role) during Action).

test('has_transition') :-
	rdf_equal(test:'TestColor_0',Q),
	% TODO: assert a description of the transistion, then state that
	%          the episode satisfies this description.
	assert_true(kb_project(has_region(Q,test:'TEST_GREEN') until 300)),
	assert_true(kb_project(has_region(Q,test:'TEST_RED') since 300)),
	assert_true(has_region(Q,test:'TEST_RED')),
	assert_false(has_region(Q,test:'TEST_GREEN')),
	assert_true(has_region(Q,test:'TEST_GREEN') during [200,250]).

test('action_succeeded') :-
	test_action(Action),
	assert_false(action_succeeded(Action)),
	assert_true(kb_project(action_succeeded(Action))),
	assert_true(action_succeeded(Action)).

test('is_masterful') :-
	test_episode(Episode),
	%%
	kb_call(new_iri(Masterful)),
	kb_project(has_type(Masterful,test:'Masterful')),
	assert_true(ground(Masterful)),
	assert_true(is_diagnosis(Masterful)),
	%%
	assert_false(satisfies(Episode,Masterful)),
	assert_true(kb_project(satisfies(Episode,Masterful))),
	assert_true(satisfies(Episode,Masterful)).

%test(mem_predicate_add_role) :-
  %test_action(TskNode,_,_),
  %mem_event_add_role(TskNode, knowrob:'Reasoner'),
  %mem_event_add_classification(TskNode,
      %test:'predicate1', knowrob:'Reasoner').

%test(mem_predicate_assignment) :-
  %test_action(TskNode,_,_),
  %mem_event_add_assignment(TskNode,
      %test:'predicate1_Argument1',test:'Bowl_0'),
  %mem_event_add_assignment(TskNode,
      %test:'predicate1_Argument2',test:'Artifact_0'),
  %%%
  %mem_event_assignment(TskNode,
      %test:'predicate1_Argument1',test:'Bowl_0'),
  %mem_event_assignment(TskNode,
      %test:'predicate1_Argument2',test:'Artifact_0').

:- end_rdf_tests(model_NEEM).
