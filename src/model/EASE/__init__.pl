
% register ROS packages to resolve IRI prefixes to local paths
:- ros_package_iri(ease_ontology, 'http://www.ease-crc.org/ont').
:- ros_package_iri(rosowl,        'http://www.ease-crc.org/ont').
% TODO: rather load bundled version here

% load modules into user
:- use_module('ACT').
:- use_module('OBJ').
:- use_module('PROC').
:- use_module('STATE').
:- use_module('WF').
:- use_module('EXT').
