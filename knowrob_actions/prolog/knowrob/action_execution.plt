
:- begin_tests(action_execution).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/action_execution')).

:- owl_parse('package://knowrob_actions/owl/test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(act_exec_test,
   'http://knowrob.org/kb/action-test.owl#', [keep(true)]).

% TODO: maybe this needs to go into a package
test_task_reset :-
  forall(
    rdf_has(act_exec_test:'rdf_has1_Task',dul:isTaskOf,Role),
    rdf_retractall(_,dul:isClassifiedBy,Role)),
  forall(
    rdf_has(act_exec_test:'rdf_has1_Task',dul:hasParameter,Parameter),
    rdf_retractall(_,dul:isClassifiedBy,Parameter)).

test(rdf_has_isExecutionPossible) :-
  task_isExecutionPossible(act_exec_test:'rdf_has1_Task').

test(rdf_has_isExecutedIn) :-
  test_task_reset,
  task_isExecutedIn(act_exec_test:'rdf_has1_Task',
       knowrob:'KBQuerying',
       act_exec_test:'rdf_has1_Execution'), !.

test('rdf_has(ACTION_INPUT_MISSING)') :-
  test_task_reset,
  execute_task(act_exec_test:'rdf_has1_Task',Action),
  action_status(Action,knowrob:'ACTION_INPUT_MISSING').
  
%% all arguments unbound
test('rdf_has(?,?,?)') :-
  %%% rdf_has(S,P,O)
  test_task_reset,
  % add missing input
  rdf_instance_from_class(dul:'Role',Role),
  rdf_assert(Role, dul:classifies, act_exec_test:'rdf_has1'),
  rdf_assert(act_exec_test:'rdf_has1_Task',dul:isTaskOf,Role),
  % execute again
  execute_task(act_exec_test:'rdf_has1_Task',Action),
  action_status(Action,knowrob:'ACTION_OK').

%% ObjectProperty, third argument unbound
test('rdf_has(obj1,hasConstituent,?)') :-
  test_task_reset,
  rdf_assert(act_exec_test:'Object1',
             dul:isClassifiedBy,
             act_exec_test:'rdf_has1_Execution_S'),
  rdf_assert(dul:'hasConstituent', % FIXME formal properties cannot be classified by roles, use ReifiedRelation
             dul:isClassifiedBy,
             act_exec_test:'rdf_has1_Execution_P'),
  % execute again
  execute_task(act_exec_test:'rdf_has1_Task',Action),
  rdf_has(act_exec_test:'Object2',
          dul:isClassifiedBy,
          act_exec_test:'rdf_has1_Execution_O'),
  action_status(Action,knowrob:'ACTION_OK').

%% DataProperty, third argument unbound
test('rdf_has(obj1,hasNameString,?)') :-
  test_task_reset,
  rdf_assert(act_exec_test:'Object1',
             dul:isClassifiedBy,
             act_exec_test:'rdf_has1_Execution_S'),
  rdf_assert(ease:'hasNameString', % FIXME use ReifiedRelation
             dul:isClassifiedBy,
             act_exec_test:'rdf_has1_Execution_P'),
  % execute again
  execute_task(act_exec_test:'rdf_has1_Task',Action),
  % FIXME: it should actually not work to classify Region by rdf_has1_Execution_O,
  %           because it is a role! however, types of predicate arguments are not fixed!
  rdf_has(Region, dul:isClassifiedBy,
          act_exec_test:'rdf_has1_Execution_O'),
  rdf_has_prolog(Region, dul:hasRegionDataValue, 'obj1'),
  action_status(Action,knowrob:'ACTION_OK').

%test(execute_task_CHOICEPOINTS) :-
  %test_task_reset,
  %fail.

:- end_tests(action_execution).
