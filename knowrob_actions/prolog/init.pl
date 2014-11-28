%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies


:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).

:- use_module(library('knowrob_actions')).

% :- owl_parser:owl_parse('/work/tenorth/work/roboearth/ros_roboearth/re_ontology/owl/serve_drink_new.owl', false, false, true).


:- rdf_db:rdf_register_ns(serve_drink,   'http://www.roboearth.org/kb/serve_drink.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(make_pancakes, 'http://knowrob.org/kb/pancake-making.owl#',    [keep(true)]).

