
:- begin_tests('knowrob/memory').

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/memory')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_model'), [action_status/2]).
:- use_module(library('knowrob/event_memory')).
:- use_module(library('knowrob/temporal')).

:- owl_parse('package://knowrob_memory/owl/test.owl').

:- rdf_db:rdf_register_ns(mem_test,  'http://knowrob.org/kb/mem-test.owl#', [keep(true)]).

:- dynamic test_episode/1.

:- mem_init, mem_drop.

test(mem_episode_create) :-
  mem_episode_create(Episode),
  rdfs_individual_of(Episode,ease:'Episode'),
  rdf_has(Episode,dul:includesTime,_),
  assertz(test_episode(Episode)).

test(mem_event_interval0) :-
  test_episode(Episode),
  \+ interval(Episode,_),
  mem_event_begin(Episode,10),
  interval(Episode,[10]).

test(mem_event_interval1) :-
  test_episode(Episode),
  mem_event_begin(Episode,4),
  interval(Episode,[4]).

test(mem_event_interval2) :-
  test_episode(Episode),
  mem_event_begin(Episode,8),
  interval(Episode,[4]).

test(mem_event_interval3) :-
  test_episode(Episode),
  mem_event_end(Episode,8),
  interval(Episode,[4,8]).

test(mem_event_interval4) :-
  test_episode(Episode),
  mem_event_interval(Episode,0,10),
  interval(Episode,[0,10]).

test(mem_event_create0) :-
  test_episode(Episode),
  mem_event_create(Episode,mem_test:'TestTask',TskNode),
  % TskNode is a Situation
  rdfs_individual_of(TskNode,dul:'Situation'),
  % The episode includes TskNode
  rdf_has(Episode,ease:includesSituation,TskNode),
  % TskNode includes an action that executes TestTask
  rdf_has(TskNode,dul:includesEvent,Act),
  rdfs_individual_of(Act,dul:'Action'),
  rdf_has(Tsk,dul:isExecutedIn,Act),
  rdfs_individual_of(Tsk,mem_test:'TestTask').

test(mem_event_set_active) :-
  test_episode(Episode),
  rdf_has(Episode,ease:includesSituation,TskNode),
  rdf_has(TskNode,dul:includesEvent,Act),
  mem_event_set_active(TskNode),
  action_status(Act,ease_act:'ExecutionState_Active').

test(mem_event_set_diagnosis) :-
  test_episode(Episode),
  rdf_has(Episode,ease:includesSituation,TskNode),
  mem_event_add_diagnosis(TskNode,ease:'Clumsiness'),
  % TskNode satisfies a description of Clumsiness
  rdf_has(TskNode,dul:satisfies,Clumsiness),
  rdfs_individual_of(Clumsiness,ease:'Clumsiness').

test(mem_subevent_interval) :-
  test_episode(Episode),
  rdf_has(Episode,ease:includesSituation,TskNode),
  mem_event_interval(TskNode,5,25),
  interval(Episode,[0,25]).

test(mem_event_includes) :-
  test_episode(Episode),
  rdf_has(Episode,ease:includesSituation,TskNode),
  rdf_has(TskNode,dul:includesEvent,Act),
  mem_event_includes(TskNode,mem_test:'Substance_0',mem_test:'ARole'),
  % Substance_0 participates in the action
  rdf_has(Act,dul:hasParticipant,mem_test:'Substance_0'),
  % and it is classified by mem_test:'ARole'
  rdf_has(TskNode,ease:includesSituation,Classification),
  rdfs_individual_of(Classification,dul:'Classification'),
  rdf_has(Classification,dul:includesObject,mem_test:'Substance_0'),
  rdf_has(Classification,ease:includesConcept,ARole0),
  rdfs_individual_of(ARole0,mem_test:'ARole'),
  % ARole0 is associated to the task executed in Act
  rdf_has(Tsk,dul:isTaskOf,ARole0),
  rdf_has(Tsk,dul:isExecutedIn,Act).

test(mem_event_causes_transition) :-
  test_episode(Episode),
  rdf_has(Episode,ease:includesSituation,TskNode),
  mem_event_causes_transition(TskNode,
      mem_test:'Substance_0',
      mem_test:'TestColor',
      mem_test:'TEST_GREEN',
      mem_test:'TEST_RED'),
  % the episode includes the transition
  rdf_has(Episode,ease:includesSituation,Transition),
  rdfs_individual_of(Transition,dul:'Transition'),
  % transition has initial state
  rdf_has(Transition,ease:hasInitialState,T0),
  rdf_has(T0,dul:satisfies,QR0),
  rdf_has(QR0,dul:hasQuality,Q),
  kb_triple(QR0,dul:hasRegion,mem_test:'TEST_GREEN'),
  % transition has terminal state
  rdf_has(Transition,ease:hasTerminalState,T1),
  rdf_has(T1,dul:satisfies,QR1),
  rdf_has(QR1,dul:hasQuality,Q),
  kb_triple(QR1,dul:hasRegion,mem_test:'TEST_RED').

:- end_tests('knowrob/memory').
