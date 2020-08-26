
% load modules into user
:- use_module('./RDFS.pl').
:- use_module('./XSD.pl').
:- use_module('./OWL.pl').
:- use_module('./QUDT.pl').
:- use_module('./portray.pl').

% load init files in sub-directories
:- use_directory('DUL').
:- use_directory('EASE').
:- use_directory('URDF').

% load additional ontologies
:- tripledb_load(
        'http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl',
        [ namespace(io)
        ]).
:- tripledb_load(
        'http://knowrob.org/kb/knowrob.owl',
        [ namespace(knowrob)
        ]).

%
:- use_module('./notify.pl').
