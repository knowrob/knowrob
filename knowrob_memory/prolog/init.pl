
% dependencies
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_mongo).
:- register_ros_package(knowrob_objects).

:- use_module(library('knowrob/event_memory')).
:- use_module(library('knowrob/triple_memory')).
:- use_module(library('knowrob/ros_memory')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/memory')).

:- owl_parser:owl_parse('package://knowrob_memory/owl/knowrob_memory.owl').

:- mem_init.
