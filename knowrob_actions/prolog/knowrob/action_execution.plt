
:- begin_tests(action_execution).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/action_execution')).
:- use_module(library('http/json')).

:- owl_parse('package://knowrob_actions/owl/test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(acext,
   'http://knowrob.org/kb/action-test.owl#', [keep(true)]).

:- rdf_meta get_dict(r,+,r),
            create_input_dict(t,t).

create_input_dict(Dict,List) :-
  findall(X-Y, member([X,Y],List),Pairs),
  dict_pairs(Dict,_,Pairs).

		 /*******************************
		 *	KB QUERYING		*
		 *******************************/

test(rdf_has_isExecutionPossible) :-
  task_isExecutionPossible(acext:'rdf_has1_Task').

test(rdf_has_isExecutedIn) :-
  task_isExecutedIn(acext:'rdf_has1_Task',
       knowrob:'KBQuerying',
       acext:'rdf_has1_Execution'), !.

test('rdf_has(ACTION_INPUT_MISSING)') :-
  execute_task(acext:'rdf_has1_Task',_{},Action,_),
  action_status(Action,knowrob:'ACTION_INPUT_MISSING').
  
%% all arguments unbound
test('rdf_has(?,?,?)') :-
  create_input_dict(InputDict,[
      [acext:'rdf_has1_Execution_Predicate', acext:'rdf_has1']
  ]),
  % execute again
  execute_task(acext:'rdf_has1_Task',InputDict,Action,_),
  action_status(Action,knowrob:'ACTION_OK').

%% ObjectProperty, third argument unbound
test('rdf_has(obj1,hasConstituent,?)') :-
  owl_reified_relation(dul:'hasConstituent', HasConstituent),
  create_input_dict(InputDict, [
      [acext:'rdf_has1_Execution_S', acext:'Object1'],
      [acext:'rdf_has1_Execution_P', HasConstituent],
      [acext:'rdf_has1_Execution_Predicate', acext:'rdf_has1']
  ]),
  % execute again
  execute_task(acext:'rdf_has1_Task',InputDict,Action,OutputDict),
  get_dict(acext:'rdf_has1_Execution_O', OutputDict, acext:'Object2'),
  action_status(Action,knowrob:'ACTION_OK').

%% DataProperty, third argument unbound
test('rdf_has(obj1,hasNameString,?)') :-
  owl_reified_relation(ease:'hasNameString', HasNameString),
  create_input_dict(InputDict, [
      [acext:'rdf_has1_Execution_S',acext:'Object1'],
      [acext:'rdf_has1_Execution_P',HasNameString],
      [acext:'rdf_has1_Execution_Predicate',acext:'rdf_has1']
  ]),
  % execute again
  execute_task(acext:'rdf_has1_Task',InputDict,Action,OutputDict),
  % FIXME: it should actually not work to classify Region by rdf_has1_Execution_O,
  %           because it is a role! however, types of predicate arguments are not fixed!
  get_dict(acext:'rdf_has1_Execution_O', OutputDict, Region),
  rdf_has_prolog(Region, dul:hasRegionDataValue, 'obj1'),
  action_status(Action,knowrob:'ACTION_OK').

test('current_object_pose(obj1,?)') :-
  create_input_dict(InputDict, [
      [acext:'current_object_pose_Execution_O',acext:'Object1'],
      [acext:'current_object_pose_Execution_Predicate',acext:'current_object_pose']
  ]),
  % execute
  execute_task(acext:'current_object_pose_Task',InputDict,Action,OutputDict),
  get_dict(acext:'current_object_pose_Execution_P', OutputDict, Transform),
  transform_data(Transform, ([0.004,0.003,0.085], [0.0,0.0,0.0,1.0])),
  action_status(Action,knowrob:'ACTION_OK').

		 /*******************************
		 *	ROS QUERYING		*
		 *******************************/
  
test('add_two_ints(POSSIBLE)') :-
  task_isExecutedIn(acext:'add_two_ints_Task',ros:'ServiceQuerying',_),!.

test('add_two_ints(CREATE)') :-
  owl_create_atomic_region(xsd:long, 2, Region_a),
  owl_create_atomic_region(xsd:long, 4, Region_b),
  create_input_dict(InputDict, [
      [acext:'add_two_ints_Execution_a',Region_a],
      [acext:'add_two_ints_Execution_b',Region_b],
      [acext:'add_two_ints_Execution_iface',acext:'add_two_ints_Interface']
  ]),
  %%
  create_action_symbol(ros:'ServiceQuerying',acext:'add_two_ints_Task',InputDict,Action),
  rdf_has(Action, dul:executesTask, acext:'add_two_ints_Task'),
  rdf_has(Action, dul:hasRegion, Region_a),
  rdf_has(Action, dul:hasRegion, Region_b).

test('add_two_ints(ENCODE)') :-
  rdf_has(Action, dul:executesTask, acext:'add_two_ints_Task'),
  owl_create_atomic_region(xsd:long, 2, Region_a),
  owl_create_atomic_region(xsd:long, 4, Region_b),
  create_input_dict(InputDict, [
      [acext:'add_two_ints_Execution_a',Region_a],
      [acext:'add_two_ints_Execution_b',Region_b],
      [acext:'add_two_ints_Execution_iface',acext:'add_two_ints_Interface']
  ]),
  %%%%
  create_ros_request(Action, InputDict, acext:'add_two_ints_RequestType', Request),
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
  rdf_has(Action, dul:executesTask, acext:'add_two_ints_Task'),
  %%%%
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Response,dul:realizes,acext:'add_two_ints_ResponseType'),
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

test('sum_array(CREATE)') :-
  owl_create_atomic_region(knowrob:array_double, [4.0, 5.0, 2.0], Region_a),
  create_input_dict(InputDict, [
      [acext:'sum_array_Execution_a',Region_a]
  ]),
  %%
  create_action_symbol(ros:'ServiceQuerying',acext:'sum_array_Task',InputDict,Action),
  rdf_has(Action, dul:executesTask, acext:'sum_array_Task'),
  rdf_has(Action, dul:hasRegion, Region_a).

test('sum_array(ENCODE)') :-
  rdf_has(Action, dul:executesTask, acext:'sum_array_Task'),
  owl_create_atomic_region(knowrob:array_double, [4.0, 5.0, 2.0], Region_a),
  create_input_dict(InputDict, [
      [acext:'sum_array_Execution_a',Region_a],
      [acext:'sum_array_Execution_iface',acext:'sum_array_Interface']
  ]),
  %%%%
  create_ros_request(Action, InputDict, acext:'sum_array_RequestType', Request),
  rdf_has(Action, dul:hasParticipant, Request),
  %%%
  once((
    rdf_has(Request, dul:hasPart, Slot_a),
    rdf_has(Slot_a, dul:realizes, Slot_a_type),
    rdf_has_prolog(Slot_a_type, ros:hasSlotName, a),
    rdf_has(Slot_a, dul:hasRegion, Region_a),
    rdf_has_prolog(Region_a, dul:hasRegionDataValue, [4.0, 5.0, 2.0])
  )),
  %%%%
  ros_request_encode(Request,Request_json),
  Request_json='{"a": ["array(float64)",  [4.0, 5.0, 2.0 ] ]}'.
  
test('sum_array(DECODE)') :-
  Response_json='{"b": [9.0, 7.0 ]}',
  %%%%
  rdf_has(Action, dul:executesTask, acext:'sum_array_Task'),
  %%%%
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Response,dul:realizes,acext:'sum_array_ResponseType'),
  rdf_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    rdf_has(Response, dul:hasPart, Slot_sum),
    rdf_has(Slot_sum, dul:realizes, Slot_sum_type),
    rdf_has_prolog(Slot_sum_type, ros:hasSlotName, b),
    rdf_has(Slot_sum, dul:hasRegion, Region_sum),
    rdf_has_prolog(Region_sum, dul:hasRegionDataValue, [9.0, 7.0])
  )).

test_pose_in([
    'geometry_msgs/Transform',
    _{
      rotation: [
	'geometry_msgs/Quaternion',
	_{
	  w: [float64, 0.4 ],
	  x: [float64, 0.8 ],
	  y: [float64, 0.1 ],
	  z: [float64, 0.0 ]
	}
      ],
      translation: [
	'geometry_msgs/Vector3',
	_{
	  x: [float64, 3.2 ],
	  y: [float64, 0.1 ],
	  z: [float64, 0.4 ]
	}
      ]
    }
]).
test_pose_out(_{
  rotation:    _{w: 0.4, x: 0.8, y: 0.1, z: 0.0 },
  translation: _{x: 3.2, y: 0.1, z: 0.4 }
}).

pose_test_input(Dict) :-
  rdf_instance_from_class(dul:'Region',Region_a),
  rdf_assert(Region_a,knowrob:translation,
        literal(type(knowrob:array_double,'3.2 0.1 0.4'))),
  rdf_assert(Region_a,knowrob:quaternion,
        literal(type(knowrob:array_double,'0.8 0.1 0.0 0.4'))),
  create_input_dict(Dict, [
      [acext:'pose_test_Execution_a',Region_a]
  ]).

test('pose_test(CREATE)') :-
  pose_test_input(InputDict),
  %%
  create_action_symbol(ros:'ServiceQuerying',acext:'pose_test_Task',InputDict,Action),
  rdf_has(Action, dul:executesTask, acext:'pose_test_Task'),
  rdf_has(Action, dul:hasRegion, _).

test('pose_test(ENCODE)') :-
  rdf_has(Action, dul:executesTask, acext:'pose_test_Task'),
  pose_test_input(InputDict),
  %%%%
  create_ros_request(Action, InputDict, acext:'pose_test_RequestType', Request),
  rdf_has(Action, dul:hasParticipant, Request),
  %%%
  once((
    rdf_has(Request, dul:hasPart, Slot_a),
    rdf_has(Slot_a, dul:realizes, Slot_a_type),
    rdf_has_prolog(Slot_a_type, ros:hasSlotName, a),
    rdf_has(Slot_a, dul:hasRegion, Region_a),
    rdf_has_prolog(Region_a, knowrob:translation, [3.2, 0.1, 0.4]),
    rdf_has_prolog(Region_a, knowrob:quaternion, [0.8, 0.1, 0.0, 0.4])
  )),
  %%%%
  ros_request_encode(Request,Request_json),
  test_pose_in(TestPose),
  with_output_to(atom(Request_json), 
    json_write_dict(current_output, _{a:TestPose})
  ).

test('pose_test(DECODE)') :-
  test_pose_out(TestPose),
  with_output_to(atom(Response_json), 
    json_write_dict(current_output, _{b:TestPose})
  ),
  %%%%
  rdf_has(Action, dul:executesTask, acext:'pose_test_Task'),
  %%%%
  rdf_instance_from_class(ros:'Message',Response),
  rdf_assert(Response,dul:realizes,acext:'pose_test_ResponseType'),
  rdf_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    rdf_has(Response, dul:hasPart, Slot_pose),
    rdf_has(Slot_pose, dul:realizes, Slot_pose_type),
    rdf_has_prolog(Slot_pose_type, ros:hasSlotName, b),
    rdf_has(Slot_pose, dul:hasRegion, Region_pose),
    rdf_has_prolog(Region_pose, knowrob:translation, [3.2, 0.1, 0.4]),
    rdf_has_prolog(Region_pose, knowrob:quaternion, [0.8, 0.1, 0.0, 0.4])
  )).

% TODO: test message fields
% TODO: test status field

:- end_tests(action_execution).
