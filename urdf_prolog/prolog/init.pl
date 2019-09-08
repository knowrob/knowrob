:- register_ros_package(knowrob_common).
:- register_ros_package(rosprolog).
:- register_ros_package(urdf_prolog).

:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).
:- owl_parser:owl_parse('package://urdf_prolog/owl/urdf.owl').

:- use_module('rdf_urdf').
