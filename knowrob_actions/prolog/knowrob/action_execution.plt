
:- begin_tests(action_execution).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/action_model')).
:- use_module(library('knowrob/action_execution')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/rosowl')).
:- use_module(library('http/json')).

:- use_module(library('knowrob/action_execution_pl')).
:- use_module(library('knowrob/action_execution_ros')).

:- owl_parse('package://knowrob_actions/owl/test.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(acext,
   'http://knowrob.org/kb/action-test.owl#', [keep(true)]).

:- rdf_meta get_dict(r,+,r),
            create_input_dict(t,t),
            create_region_(t,r,r).

create_region_(Data_pl,Data_type,Region) :-
  kb_create(dul:'Region',Region),
  kb_rdf_data(Data_atom,Data_type,Data_pl),
  rdf_assert(Region,dul:hasRegionDataValue,
             literal(type(Data_type,Data_atom))).

create_input_dict(Dict,List) :-
  findall(X-Y, member([X,Y],List),Pairs),
  dict_pairs(Dict,_,Pairs).

		 /*******************************
		 *	KB QUERYING		*
		 *******************************/

test(rdf_has_execution_goal) :-
  plan_execution_goal(
    acext:'rdf_has1_Execution',
    _,
    knowrob:'KBQuerying'
    ,_), !.
  
%% all arguments unbound
test('rdf_has(?,?,?)', [nondet]) :-
  create_input_dict(InputDict,[]),
  % execute again
  execute_plan(acext:'rdf_has1_Execution',InputDict,_,Situation),
  kb_triple(Situation,dul:includesAction,Action),
  action_status(Action,ease_act:'ExecutionState_Succeeded').

%% ObjectProperty, third argument unbound
test('rdf_has(obj1,hasConstituent,?)', [nondet]) :-
  kb_reification(dul:'hasConstituent', HasConstituent),
  create_input_dict(InputDict, [
      [acext:'rdf_has1_Task_S', acext:'Object1'],
      [acext:'rdf_has1_Task_P', HasConstituent]
  ]),
  % execute again
  execute_plan(acext:'rdf_has1_Execution',InputDict,OutputDicts,Situation),
  kb_triple(Situation,dul:includesAction,Action),
  member(OD2,OutputDicts),
  get_dict(acext:'rdf_has1_Task_O', OD2, acext:'Object2'),
  member(OD3,OutputDicts), get_dict(acext:'rdf_has1_Task_O', OD3, acext:'Object3'),
  action_status(Action,ease_act:'ExecutionState_Succeeded').

%% DataProperty, third argument unbound
test('rdf_has(obj1,hasNameString,?)', [nondet]) :-
  kb_reification(ease:'hasNameString', HasNameString),
  create_input_dict(InputDict, [
      [acext:'rdf_has1_Task_S',acext:'Object1'],
      [acext:'rdf_has1_Task_P',HasNameString]
  ]),
  % execute again
  execute_plan(acext:'rdf_has1_Execution',InputDict,[OutputDict|_],Situation),
  kb_triple(Situation,dul:includesAction,Action),
  % FIXME: it should actually not work to classify Region by rdf_has1_Execution_O,
  %           because it is a role! however, types of predicate arguments are not fixed!
  get_dict(acext:'rdf_has1_Task_O', OutputDict, Region),
  kb_triple(Region, dul:hasRegionDataValue, 'obj1'),
  action_status(Action,ease_act:'ExecutionState_Succeeded').

		 /*******************************
		 *	ROS QUERYING		*
		 *******************************/
  
test(add_two_ints_execution_goal) :-
  plan_execution_goal(
    acext:'add_two_ints_Execution',
    _,
    ros:'ServiceInvokation'
    ,_), !.

test('add_two_ints(CREATE)') :-
  create_region_(2, xsd:long, Region_a),
  create_region_(4, xsd:long, Region_b),
  create_input_dict(InputDict, [
      [acext:'add_two_ints_Task_a',Region_a],
      [acext:'add_two_ints_Task_b',Region_b]
  ]),
  %%
  knowrob_action_execution:plan_execution_create(
      ros:'ServiceInvokation',
      acext:'add_two_ints_Task',
      acext:'add_two_ints_Execution',
      InputDict,Action,_),
  kb_triple(Action, dul:executesTask, acext:'add_two_ints_Task'),
  kb_triple(Action, dul:hasRegion, Region_a),
  kb_triple(Action, dul:hasRegion, Region_b).

test('add_two_ints(ENCODE)') :-
  once(kb_triple(Action, dul:executesTask, acext:'add_two_ints_Task')),
  create_region_(2, xsd:long, Region_a),
  create_region_(4, xsd:long, Region_b),
  create_input_dict(InputDict, [
      [acext:'add_two_ints_Task_a',Region_a],
      [acext:'add_two_ints_Task_b',Region_b]
  ]),
  knowrob_action_execution:action_bindings(acext:'add_two_ints_Execution',ActionDict),
  %%%%
  create_ros_request(Action, InputDict, ActionDict, acext:'add_two_ints_RequestType', Request),
  kb_triple(Action, dul:hasParticipant, Request),
  %%%
  once((
    kb_triple(Request, dul:hasPart, Slot_a),
    kb_triple(Slot_a, dul:realizes, Slot_a_type),
    kb_triple(Slot_a_type, ros:hasSlotName, a),
    kb_triple(Slot_a, dul:hasRegion, Region_a),
    % FIXME: warning here. Might be due to owl_same_as stripping the type!
    kb_triple(Region_a, dul:hasRegionDataValue, 2)
  )),
  %%%%
  ros_request_encode(Request,Request_json),
  Request_json='{"a": ["int64", 2 ], "b": ["int64", 4 ]}'.
  
test('add_two_ints(DECODE)') :-
  Response_json='{"sum": 5}',
  %%%%
  once(kb_triple(Action, dul:executesTask, acext:'add_two_ints_Task')),
  %%%%
  kb_create(ros:'Message',Response),
  kb_assert(Response,dul:realizes,acext:'add_two_ints_ResponseType'),
  kb_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    kb_triple(Response, dul:hasPart, Slot_sum),
    kb_triple(Slot_sum, dul:realizes, Slot_sum_type),
    kb_triple(Slot_sum_type, ros:hasSlotName, sum),
    kb_triple(Slot_sum, dul:hasRegion, Region_sum),
    % FIXME: warning here. Might be due to owl_same_as stripping the type!
    kb_triple(Region_sum, dul:hasRegionDataValue, 5)
  )).

test('sum_array(CREATE)') :-
  create_region_([4.0, 5.0, 2.0], ease:array_double, Region_a),
  create_input_dict(InputDict, [
      [acext:'sum_array_Task_a',Region_a]
  ]),
  %%
  knowrob_action_execution:plan_execution_create(
      ros:'ServiceInvokation',
      acext:'sum_array_Task',
      acext:'sum_array_Execution',
      InputDict,Action,_),
  kb_triple(Action, dul:executesTask, acext:'sum_array_Task'),
  kb_triple(Action, dul:hasRegion, Region_a).

test('sum_array(ENCODE)') :-
  once(kb_triple(Action, dul:executesTask, acext:'sum_array_Task')),
  create_region_([4.0, 5.0, 2.0], ease:array_double, Region_a),
  create_input_dict(InputDict, [
      [acext:'sum_array_Task_a',Region_a]
  ]),
  knowrob_action_execution:action_bindings(acext:'sum_array_Execution',ActionDict),
  %%%%
  create_ros_request(Action, InputDict, ActionDict, acext:'sum_array_RequestType', Request),
  kb_triple(Action, dul:hasParticipant, Request),
  %%%
  once((
    kb_triple(Request, dul:hasPart, Slot_a),
    kb_triple(Slot_a, dul:realizes, Slot_a_type),
    kb_triple(Slot_a_type, ros:hasSlotName, a),
    kb_triple(Slot_a, dul:hasRegion, Region_a),
    kb_triple(Region_a, dul:hasRegionDataValue, [4.0, 5.0, 2.0])
  )),
  %%%%
  ros_request_encode(Request,Request_json),
  Request_json='{"a": ["array(float64)",  [4.0, 5.0, 2.0 ] ]}'.
  
test('sum_array(DECODE)') :-
  Response_json='{"b": [9.0, 7.0 ]}',
  %%%%
  once(kb_triple(Action, dul:executesTask, acext:'sum_array_Task')),
  %%%%
  kb_create(ros:'Message',Response),
  kb_assert(Response,dul:realizes,acext:'sum_array_ResponseType'),
  kb_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    kb_triple(Response, dul:hasPart, Slot_sum),
    kb_triple(Slot_sum, dul:realizes, Slot_sum_type),
    kb_triple(Slot_sum_type, ros:hasSlotName, b),
    kb_triple(Slot_sum, dul:hasRegion, Region_sum),
    kb_triple(Region_sum, dul:hasRegionDataValue, [9.0, 7.0])
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
  kb_create(dul:'Region',Region_a),
  kb_assert(Region_a,knowrob:translation,
        literal(type(ease:array_double,'3.2 0.1 0.4'))),
  kb_assert(Region_a,knowrob:quaternion,
        literal(type(ease:array_double,'0.8 0.1 0.0 0.4'))),
  create_input_dict(Dict, [
      [acext:'pose_test_Task_a',Region_a]
  ]).

test('pose_test(CREATE)') :-
  pose_test_input(InputDict),
  %%
  knowrob_action_execution:plan_execution_create(
      ros:'ServiceInvokation',
      acext:'pose_test_Task',
      acext:'pose_test_Execution',
      InputDict,Action,_),
  once( kb_triple(Action, dul:executesTask, acext:'pose_test_Task') ),
  once( kb_triple(Action, dul:hasRegion, _) ).

test('pose_test(ENCODE)') :-
  once(kb_triple(Action, dul:executesTask, acext:'pose_test_Task')),
  pose_test_input(InputDict),
  knowrob_action_execution:action_bindings(acext:'pose_test_Execution',ActionDict),
  %%%%
  create_ros_request(Action, InputDict, ActionDict, acext:'pose_test_RequestType', Request),
  kb_triple(Action, dul:hasParticipant, Request),
  %%%
  once((
    kb_triple(Request, dul:hasPart, Slot_a),
    kb_triple(Slot_a, dul:realizes, Slot_a_type),
    kb_triple(Slot_a_type, ros:hasSlotName, a),
    kb_triple(Slot_a, dul:hasRegion, Region_a),
    kb_triple(Region_a, knowrob:translation, [3.2, 0.1, 0.4]),
    kb_triple(Region_a, knowrob:quaternion, [0.8, 0.1, 0.0, 0.4])
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
  once(kb_triple(Action, dul:executesTask, acext:'pose_test_Task')),
  %%%%
  kb_create(ros:'Message',Response),
  kb_assert(Response,dul:realizes,acext:'pose_test_ResponseType'),
  kb_assert(Action,ros:hasResponse,Response),
  %%%%
  ros_response_decode(Response_json, Response),
  %%%
  once((
    kb_triple(Response, dul:hasPart, Slot_pose),
    kb_triple(Slot_pose, dul:realizes, Slot_pose_type),
    kb_triple(Slot_pose_type, ros:hasSlotName, b),
    kb_triple(Slot_pose, dul:hasRegion, [map,_,
        [3.2, 0.1, 0.4],
        [0.8, 0.1, 0.0, 0.4]
    ])
  )).

% TODO: test message fields
% TODO: test status field

:- end_tests(action_execution).
