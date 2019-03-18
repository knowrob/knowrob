
:- begin_tests(action_execution).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/action_execution')).

:- owl_parse('package://knowrob_actions/owl/test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(act_exec_test,
   'http://knowrob.org/kb/action-test.owl#', [keep(true)]).

test_task_reset :-
  forall(
    rdf_has(act_exec_test:'rdf_has1_Task',dul:isTaskOf,Role),
    rdf_retractall(_,dul:isClassifiedBy,Role)),
  forall(
    rdf_has(act_exec_test:'rdf_has1_Task',dul:hasParameter,Parameter),
    rdf_retractall(_,dul:isClassifiedBy,Parameter)),
  forall(
    rdf_has(act_exec_test:'add_two_ints_Task',dul:hasParameter,Parameter),
    rdf_retractall(_,dul:isClassifiedBy,Parameter)).

		 /*******************************
		 *	KB QUERYING		*
		 *******************************/

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

		 /*******************************
		 *	ROS QUERYING		*
		 *******************************/
  
test('add_two_ints(POSSIBLE)') :-
  task_isExecutedIn(act_exec_test:'add_two_ints_Task',ros:'ServiceQuerying',_),!.

test('add_two_ints(CREATE)') :-
  owl_create_atomic_region(xsd:long, 2, Region_a),
  owl_create_atomic_region(xsd:long, 4, Region_b),
  rdf_has_prolog(Region_a, dul:hasRegionDataValue, 2),
  rdf_has_prolog(Region_b, dul:hasRegionDataValue, 4),
  rdf_assert(Region_a, dul:isClassifiedBy, act_exec_test:'add_two_ints_Execution_a'),
  rdf_assert(Region_b, dul:isClassifiedBy, act_exec_test:'add_two_ints_Execution_b'),
  %%
  create_action_symbol(ros:'ServiceQuerying',act_exec_test:'add_two_ints_Task',Action),
  rdf_has(Action, dul:executesTask, act_exec_test:'add_two_ints_Task'),
  rdf_has(Action, dul:hasRegion, Region_a),
  rdf_has(Action, dul:hasRegion, Region_b).

test('add_two_ints(ENCODE)') :-
  rdf_has(Action, dul:executesTask, act_exec_test:'add_two_ints_Task'),
  %%%%
  create_ros_request(Action, act_exec_test:'add_two_ints_RequestType', Request),
  rdf_has(Action, dul:hasParticipant, Request),
  %%%
  once((
    rdf_has(Request, dul:hasPart, Slot_a),
    rdf_has(Slot_a, dul:realizes, Slot_a_type),
    rdf_has_prolog(Slot_a_type, ros:hasSlotName, a),
    rdf_has(Slot_a, dul:hasRegion, Region_a),
    rdf_has_prolog(Region_a, dul:hasRegionDataValue, 2)
  )),
  %%%%
  ros_request_encode(Request,Request_json),
  Request_json='{"a": ["int64", 2 ], "b": ["int64", 4 ]}'.
  
test('add_two_ints(DECODE)') :-
  Response_json='{"sum": 5}',
  %%%%
  rdf_has(Action, dul:executesTask, act_exec_test:'add_two_ints_Task'),
  %%%%
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Response,dul:realizes,act_exec_test:'add_two_ints_ResponseType'),
  rdf_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    rdf_has(Response, dul:hasPart, Slot_sum),
    rdf_has(Slot_sum, dul:realizes, Slot_sum_type),
    rdf_has_prolog(Slot_sum_type, ros:hasSlotName, sum),
    rdf_has(Slot_sum, dul:hasRegion, Region_sum),
    rdf_has_prolog(Region_sum, dul:hasRegionDataValue, 5)
  )).

% TODO: test arrays
% TODO: test message fields
% TODO: test status field

:- end_tests(action_execution).
