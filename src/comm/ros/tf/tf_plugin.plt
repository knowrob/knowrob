:- use_module('tf_plugin.pl').
:- use_module(library('rostest')).
:- use_module(library('lang/query')).
:- use_module(library('lang/scopes/temporal')).
:- use_module(library('db/mongo/client')).

:- use_module(library('lang/terms/is_at'), [ is_at/2 ]).
:- use_module(library('semweb/rdf_db')).

:- begin_tripledb_tests(
		'tf_plugin',
		'package://knowrob/owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#'),
		  setup(tf_mng_whipe),
		  cleanup(tf_mng_whipe)
		]).

:- rdf_meta(test_set_pose(r,+,+)).
:- rdf_meta(test_get_pose(r,+,+)).
:- rdf_meta(test_trajectory(r,+,+,+)).
:- rdf_meta(test_transform_pose(r,+,+)).
:- rdf_meta(test_is_unlocalized(r,+)).

% some test facts
test_pose_fred0([world,[1.0,0.4,2.32],[0.0,0.0,0.0,1.0]], 1593178679.123).
test_pose_fred1([world,[2.0,0.4,2.32],[0.0,0.0,0.0,1.0]], 1593178680.123).
test_pose_fred2([world,[3.0,0.4,2.32],[0.0,0.0,0.0,1.0]], 1593178681.123).
test_pose_alex1(['Fred',[0.0,1.0,0.0],[0.0,0.0,0.0,1.0]], 1593178680.123).

test_set_pose(Object,Pose,Stamp) :-
	time_scope(=(Stamp), =<('Infinity'), FScope),
	assert_true(tf_set_pose(Object,Pose,FScope)).

test_get_pose(Object,Stamp,Expected) :-
	time_scope(=<(Stamp), >=(Stamp), QScope),
	assert_true(tf_get_pose(Object,_,QScope,_)),
	( tf_get_pose(Object,Actual,QScope,_)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_trajectory(Obj,Begin,End,Expected) :-
	assert_true(tf_get_trajectory(Obj,Begin,End,_)),
	( tf_get_trajectory(Obj,Begin,End,Actual)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_transform_pose(Object,Stamp,Query) :-
	time_scope(=<(Stamp), >=(Stamp), QScope),
	assert_true(tf_get_pose(Object,Query,QScope,_)).

test_lookup(Frame,Stamp,Expected) :-
	assert_true(tf_mng_lookup(Frame,Stamp,Stamp,_,_,_)),
	( tf_mng_lookup(Frame,Stamp,Stamp,Actual,_,_)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_lookup_fails(Frame,Stamp) :-
	assert_false(tf_mng_lookup(Frame,Stamp,Stamp,_,_,_)).

test_is_unlocalized(Object,Stamp) :-
	time_scope(=<(Stamp), >=(Stamp), QScope),
	assert_false(tf_get_pose(Object,_,QScope,_)).

test('tf_pose') :-
	test_pose_fred0(Pose0,Stamp0),
	Past   is Stamp0 - 1.0,
	Future is Stamp0 + 1.0,
	%%
	test_is_unlocalized(test:'Fred',Stamp0),
	test_set_pose(test:'Fred',Pose0,Stamp0),
	test_get_pose(test:'Fred',Stamp0,Pose0),
	test_get_pose(test:'Fred',Future,Pose0),
	test_is_unlocalized(test:'Fred',Past).

test('tf_mongo_lookup') :-
	test_pose_fred0(Pose0,Stamp0),
	Past is Stamp0 - 1.0,
	test_lookup('Fred',Stamp0,Pose0),
	test_lookup_fails('Fred',Past).

test('tf_pose_memory') :-
	test_pose_fred0(Pose0,Stamp0),
	test_pose_fred1(Pose1,Stamp1),
	test_pose_fred2(Pose2,Stamp2),
	Stamp01 is 0.5*(Stamp0 + Stamp1),
	Future is Stamp2 + 1.0,
	%%
	test_set_pose(test:'Fred',Pose1,Stamp1),
	test_set_pose(test:'Fred',Pose2,Stamp2),
	%%
	test_get_pose(test:'Fred',Stamp0,Pose0),
	test_get_pose(test:'Fred',Stamp01,Pose0),
	test_get_pose(test:'Fred',Stamp1,Pose1),
	test_get_pose(test:'Fred',Stamp2,Pose2),
	test_get_pose(test:'Fred',Future,Pose2).

test('tf_trajectory') :-
	test_pose_fred0(Pose0,Stamp0),
	test_pose_fred1(Pose1,Stamp1),
	test_pose_fred2(Pose2,Stamp2),
	test_trajectory(test:'Fred',Stamp0,Stamp2,
		[Stamp0-Pose0, Stamp1-Pose1, Stamp2-Pose2]),
	test_trajectory(test:'Fred',Stamp1,Stamp2,
		[Stamp1-Pose1, Stamp2-Pose2]).

test('tf_transform_pose') :-
	test_pose_alex1(Pose1,Stamp1),
	test_set_pose(test:'Alex',Pose1,Stamp1),
	test_get_pose(test:'Alex',Stamp1,Pose1),
	test_transform_pose(test:'Alex',Stamp1,[world,[2.0,1.4,2.32],_]),
	test_transform_pose(test:'Fred',Stamp1,['Alex',_,_]).

test('tf_is_at') :-
	test_pose_fred0(Pose0,Stamp0),
	assert_true(ask(
		during(is_at(test:'Fred',Pose0), [Stamp0,Stamp0])
	)).

:- end_tests('tf_plugin').
