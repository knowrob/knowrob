
% make sure library path is expanded
:- register_ros_package(knowrob).

% register ROS packages to resolve IRI prefixes to local paths
:- ros_package_iri(knowrob, 'http://www.ontologydesignpatterns.org/ont/dul').
:- ros_package_iri(knowrob, 'http://www.ease-crc.org/ont').
:- ros_package_iri(knowrob, 'http://knowrob.org/kb').
:- ros_package_iri(knowrob, 'http://www.w3.org/2002/07'). %/owl.rdf

% load init files in sub-directories
:- use_directory('urdf').
:- use_directory('tf').
:- use_directory('marker').
