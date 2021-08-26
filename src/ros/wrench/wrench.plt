:- use_module(library('rostest')).
:- use_module(library('lang/query')).
:- use_module(library('lang/scope')).
:- use_module(library('db/mongo/client')).
:- use_module(library('semweb/rdf_db')).

:- use_module('wrench').
:- use_module('wrench_mongo').

:- begin_rdf_tests(
		'wrench',
		'package://knowrob/owl/test/swrl.owl',
		[ namespace('http://knowrob.org/kb/swrl_test#'),
		  setup(wrench_setup),
		  cleanup(wrench_cleanup)
		]).

:- rdf_meta(test_set_wrench(r,+,+)).
:- rdf_meta(test_get_wrench(r,+,+)).
:- rdf_meta(test_trajectory(r,+,+,+)).

wrench_setup :-
    wrench_mng_drop,
	wrench_logger_enable.

wrench_cleanup :-
	wrench_logger_disable,
	wrench_mng_drop.

% some test facts
test_wrench_fred0([[1.0,0.4,2.32],[0.0,0.0,0.0]], 1593178679.123).
test_wrench_fred1([[2.0,0.4,2.32],[0.0,0.0,0.0]], 1593178680.123).
test_wrench_fred2([[3.0,0.4,2.32],[0.0,0.0,0.0]], 1593178681.123).
test_wrench_alex1([[0.0,1.0,0.0],[0.0,0.0,0.0]], 1593178680.123).
test_wrench_alex2([[0.0,0.0,1.0],[0.0,0.0,0.0]], 1593178681.123).
test_wrench_alex3([[0.0,1.0,0.0],[0.0,0.0,0.0]], 1593178682.123).

test_set_wrench(Object,Wrench,Stamp) :-
	time_scope(=(Stamp), =<('Infinity'), FScope),
	assert_true(wrench:wrench_set(Object,Wrench,FScope)).

test_get_wrench(Object,Stamp,Expected) :-
	time_scope(=<(Stamp), >=(Stamp), QScope),
	assert_true(wrench:wrench_get(Object,_,QScope,_)),
	( wrench:wrench_get(Object,Actual,QScope,_)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_trajectory(Obj,Begin,End,Expected) :-
	assert_true(wrench_mongo:wrench_mng_trajectory(Obj,Begin,End,_)),
	( wrench_mongo:wrench_mng_trajectory(Obj,Begin,End,Actual)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_lookup(Frame,Stamp,Expected) :-
	assert_true(wrench:wrench_mng_lookup(Frame,Stamp,Stamp,_,_,_)),
	( wrench:wrench_mng_lookup(Frame,Stamp,Stamp,Actual,_,_)
	-> assert_unifies(Actual,Expected)
	;  true
	).

test_lookup_fails(Frame,Stamp) :-
	assert_false(wrench:wrench_mng_lookup(Frame,Stamp,Stamp,_,_,_)).

test('wrench_set_get') :-
	test_wrench_fred0(Wrench0,Stamp0),
	Past   is Stamp0 - 1.0,
	Future is Stamp0 + 1.0,
	%%
	test_set_wrench(test:'Fred',Wrench0,Stamp0),
	test_get_wrench(test:'Fred',Stamp0,Wrench0),
	test_get_wrench(test:'Fred',Future,Wrench0).

test('wrench_mongo_lookup') :-
	test_wrench_fred0(Wrench0,Stamp0),
	Past is Stamp0 - 1.0,
	test_lookup('Fred',Stamp0,Wrench0),
	test_lookup_fails('Fred',Past).

test('wrench_wrench_memory') :-
	test_wrench_fred0(Wrench0,Stamp0),
	test_wrench_fred1(Wrench1,Stamp1),
	test_wrench_fred2(Wrench2,Stamp2),
	Stamp01 is 0.5*(Stamp0 + Stamp1),
	Future is Stamp2 + 1.0,
	%%
	test_set_wrench(test:'Fred',Wrench1,Stamp1),
	test_set_wrench(test:'Fred',Wrench2,Stamp2),
	%%
	test_get_wrench(test:'Fred',Stamp0,Wrench0),
	test_get_wrench(test:'Fred',Stamp01,Wrench0),
	test_get_wrench(test:'Fred',Stamp1,Wrench1),
	test_get_wrench(test:'Fred',Stamp2,Wrench2),
	test_get_wrench(test:'Fred',Future,Wrench2).

test('wrench_trajectory') :-
	test_wrench_fred0(Wrench0,Stamp0),
	test_wrench_fred1(Wrench1,Stamp1),
	test_wrench_fred2(Wrench2,Stamp2),
	test_trajectory(test:'Fred',Stamp0,Stamp2,
		[Stamp0-Wrench0, Stamp1-Wrench1, Stamp2-Wrench2]),
	test_trajectory(test:'Fred',Stamp1,Stamp2,
		[Stamp1-Wrench1, Stamp2-Wrench2]).

:- end_tests('wrench').
