
% dependencies
:- register_ros_package(knowrob).
:- register_ros_package(knowrob_mongo).

:- use_module(library('knowrob/event_memory')).
:- use_module(library('knowrob/triple_memory')).
:- use_module(library('knowrob/ros_memory')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/memory')).

:- mem_init.
