
% load modules into user
:- use_module('RDFS').
:- use_module('XSD').
:- use_module('OWL').
:- use_module('QUDT').

% load init files in sub-directories
:- use_directory('DUL').
:- use_directory('EASE').
%:- use_directory('URDF').

% load knowrob.owl
:- ros_package_iri(knowrob,'http://knowrob.org/kb').
:- tripledb_load(
        'http://knowrob.org/kb/knowrob.owl',
        [ graph(static),
          namespace(knowrob)
        ]).
