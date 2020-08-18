:- begin_tripledb_tests(
    'srdl',
    'package://knowrob/owl/robots/PR2.owl',
    [ namespace('http://knowrob.org/kb/PR2.owl#')
    ]).

:- use_module('./srdl.pl').

test(robot_set_urdf) :-
	ros_package_path('knowrob', X),
	atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
	object_set_urdf(
		test:'PR2_0',
		FileURL,
		[ prefix('/test/'),
		  load_rdf(true)
		]).
	

%:- dynamic testbot/1.

%is_composition(Comp) :-
  %rdf_has(Comp,srdlcomp:hasBaseLink,_),
  %rdf_has(Comp,srdlcomp:hasEndLink,_),!.

%test(robot_create) :-
%	srdl_create(knowrob:'PR2',PR2),
%	asserta(testbot(PR2)),
%	%% test created component symbols
%	once((
%		srdl_component(PR2,_,test:'PR2Base'),
%		srdl_component(PR2,_,test:'PR2ArmL'),
%		srdl_component(PR2,_,test:'PR2ArmR'),
%		srdl_component(PR2,_,test:'PR2GripperL'),
%		srdl_component(PR2,_,test:'PR2GripperR')
%	)).

%test(robot_set_urdf) :-
  %ros_package_path('urdfprolog', X),
  %atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
  %%% load the URDF
  %testbot(PR2),
  %robot_set_urdf(PR2,FileURL),
  %%% test link-component association
  %forall(
    %has_direct_component(PR2,BodyPart,_),
    %( is_composition(BodyPart) ; component_type(BodyPart,_) )
  %).

%%test(comp_unknown_action) :-
  %%\+ action_feasible_on_robot(knowrob:'leainhfoiezr8hfddef8ez3radsfij',  pr2:'PR2Robot1'),!.

%%test(cap_available_on_robot) :-
  %%cap_available_on_robot(srdl2cap:'GraspingCapability', pr2:'PR2Robot1'),!.


%%test(required_comp_for_action) :-
  %%required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmComponent'),
  %%required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmMotionController'),!.


%%test(missing_comp_for_action) :-
  %%\+ missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  %%% FIXME(daniel) below seems semantically wrong because the arm is _not_ a missing component.
  %%missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmComponent'),
  %%missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmMotionController'),!.


%%test(required_cap_for_action) :-
  %%required_cap_for_action(act:'MakingPancakes', srdl2cap:'PickingUpAnObjectCapability'),
  %%required_cap_for_action(act:'MakingPancakes', srdl2cap:'ArmMotionCapability'),!.


%%test(missing_cap_for_action) :-
  %%\+ missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  %%missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability'),
  %%missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'ArmMotionCapability'),!.


%%test(missing_for_action) :-
  %%\+ missing_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _, _),
  %%missing_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability', srdl2comp:'ArmMotionController'),!.

%%test(action_feasible_on_robot) :-
  %%\+ action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robt1'),
  %%action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robot1'),!.

:- end_tests('srdl').
