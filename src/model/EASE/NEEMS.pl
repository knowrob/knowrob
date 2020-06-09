:- module(model_EASE_NEEMS,
    [
    ]).
 
:- begin_tests(model_EASE_NEEMS                                                   ).

test(model_EASE_NEEM) :-
  fail.

%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/owl')).
%:- use_module(library('semweb/owl_parser')).

%:- use_module(library('knowrob/lang/ask')).
%:- use_module(library('knowrob/lang/tell')).
%:- use_module(library('knowrob/model/Action')).
%:- use_module(library('knowrob/model/Constraint')).
%:- use_module(library('knowrob/model/Object')).
%:- use_module(library('knowrob/model/TimeInterval')).

%:- use_module(library('knowrob/memory')).
%:- use_module(library('knowrob/event_memory')).
%:- use_module(library('knowrob/mongo')).

%:- owl_parse('package://knowrob_memory/owl/test.owl').

%:- rdf_db:rdf_register_ns(mem_test,  'http://knowrob.org/kb/mem-test.owl#', [keep(true)]).

%:- dynamic test_episode/1.

%:- mem_init, mem_drop.

%test_action(TskNode,Action,Task) :-
  %test_episode(Episode),
  %rdf_has(Episode,ease:includesSituation,TskNode),
  %mem_event(TskNode,Action,Task).


%test(mem_episode_create) :-
  %mem_episode_create(Episode),
  %rdfs_individual_of(Episode,ease:'Episode'),
  %rdf_has(Episode,dul:includesTime,_),
  %assertz(test_episode(Episode)).

%test(mem_event_interval0) :-
  %test_episode(Episode),
  %\+ interval(Episode,_),
  %mem_event_begin(Episode,10),
  %interval(Episode,[10]).

%test(mem_event_interval1) :-
  %test_episode(Episode),
  %mem_event_begin(Episode,4),
  %interval(Episode,[4]).

%test(mem_event_interval2) :-
  %test_episode(Episode),
  %mem_event_begin(Episode,8),
  %interval(Episode,[4]).

%test(mem_event_interval3) :-
  %test_episode(Episode),
  %mem_event_end(Episode,8),
  %interval(Episode,[4,8]).

%test(mem_event_interval4) :-
  %test_episode(Episode),
  %mem_event_interval(Episode,0,10),
  %interval(Episode,[0,10]).

%test(mem_event_create) :-
  %test_episode(Episode),
  %mem_event_create(Episode,mem_test:'TestTask',TskNode),
  %% TskNode is a Situation
  %rdfs_individual_of(TskNode,dul:'Situation'),
  %% The episode includes TskNode
  %rdf_has(Episode,ease:includesSituation,TskNode),
  %% TskNode includes an action
  %rdf_has(TskNode,dul:includesEvent,Act),
  %rdfs_individual_of(Act,dul:'Action').

%test(mem_event_executed_in) :-
  %test_action(_,Act,Tsk),
  %rdf_has(Tsk,dul:isExecutedIn,Act),
  %rdfs_individual_of(Tsk,mem_test:'TestTask').

%test(mem_event_created_roles) :-
  %test_action(TskNode,_,_),
  %% Tsk has some individual roles
  %once((
    %mem_event_role(TskNode,_,mem_test:'ARole'),
    %mem_event_role(TskNode,_,mem_test:'BRole')
  %)).

%test(mem_event_set_active) :-
  %test_action(TskNode,Action,_),
  %mem_event_set_active(TskNode),
  %action_status(Action,ease_act:'ExecutionState_Active').

%test(mem_event_set_diagnosis) :-
  %test_action(TskNode,_,_),
  %mem_event_add_diagnosis(TskNode,ease:'Clumsiness'),
  %% TskNode satisfies a description of Clumsiness
  %rdf_has(TskNode,dul:satisfies,Clumsiness),
  %rdfs_individual_of(Clumsiness,ease:'Clumsiness').

%test(mem_subevent_interval) :-
  %test_episode(Episode),
  %test_action(TskNode,_,_),
  %mem_event_interval(TskNode,5,25),
  %interval(Episode,[0,25]).

%test(mem_add_classification) :-
  %test_action(TskNode,_,_),
  %mem_event_includes(TskNode,mem_test:'Substance_0',mem_test:'ARole').

%test(mem_classified_participant) :-
  %test_action(_,Act,_),
  %rdf_has(Act,dul:hasParticipant,mem_test:'Substance_0').

%test(mem_classification_concept, [nondet]) :-
  %test_action(TskNode,_,_),
  %mem_event_classification(TskNode,mem_test:'Substance_0',ARole),
  %rdfs_individual_of(ARole,mem_test:'ARole').

%test(mem_add_constraint) :-
  %test_action(TskNode,_,_),
  %% test adding a constraint to the task associated to a task node
  %mem_event_add_constraint(TskNode,
      %knowrob:'KeepCloseTo',
      %mem_test:'ARole',
      %mem_test:'BRole',
      %_).

%test(mem_constraint_parameter, [nondet]) :-
  %test_action(TskNode,_,_),
  %mem_event_role(TskNode,ARole,mem_test:'ARole'),
  %mem_event_role(TskNode,BRole,mem_test:'BRole'),
  %%% the constraint has been added as parameter of the event
  %mem_event_parameter(TskNode,KeepCloseTo,knowrob:'KeepCloseTo'),
  %has_constrained_concept(KeepCloseTo,ARole,mem_test:'ARole'),
  %has_dependent_concept(KeepCloseTo,BRole,mem_test:'BRole').

%test(mem_predicate_add_role) :-
  %test_action(TskNode,_,_),
  %mem_event_add_role(TskNode, knowrob:'Reasoner'),
  %mem_event_add_classification(TskNode,
      %mem_test:'predicate1', knowrob:'Reasoner').

%test(mem_predicate_assignment) :-
  %test_action(TskNode,_,_),
  %mem_event_add_assignment(TskNode,
      %mem_test:'predicate1_Argument1',mem_test:'Bowl_0'),
  %mem_event_add_assignment(TskNode,
      %mem_test:'predicate1_Argument2',mem_test:'Artifact_0'),
  %%%
  %mem_event_assignment(TskNode,
      %mem_test:'predicate1_Argument1',mem_test:'Bowl_0'),
  %mem_event_assignment(TskNode,
      %mem_test:'predicate1_Argument2',mem_test:'Artifact_0').

%test(mem_event_causes_transition) :-
  %test_episode(Episode),
  %rdf_has(Episode,ease:includesSituation,TskNode),
  %mem_event_causes_transition(TskNode,
      %mem_test:'Substance_0',
      %mem_test:'TestColor',
      %mem_test:'TEST_GREEN',
      %mem_test:'TEST_RED'),
  %% the episode includes the transition
  %rdf_has(Episode,ease:includesSituation,Transition),
  %rdfs_individual_of(Transition,dul:'Transition'),
  %% transition has initial state
  %rdf_has(Transition,ease:hasInitialState,T0),
  %rdf_has(T0,dul:satisfies,QR0),
  %rdf_has(QR0,dul:hasQuality,Q),
  %kb_triple(QR0,dul:hasRegion,mem_test:'TEST_GREEN'),
  %% transition has terminal state
  %rdf_has(Transition,ease:hasTerminalState,T1),
  %rdf_has(T1,dul:satisfies,QR1),
  %rdf_has(QR1,dul:hasQuality,Q),
  %kb_triple(QR1,dul:hasRegion,mem_test:'TEST_RED').
  
%test(mem_assert_dimensions) :-
  %object_assert_dimensions(mem_test:'Substance_0', 0.032, 0.032, 0.12),
  %object_dimensions(mem_test:'Substance_0', 0.032, 0.032, 0.12).

:- end_tests(model_EASE_NEEM).
