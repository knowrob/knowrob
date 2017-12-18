
:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_actions).

:- use_module(library('household')).

:- owl_parser:owl_parse('package://knowrob_household/owl/household.owl').
