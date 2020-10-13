
% load modules into user
:- use_module('./RDFS.pl').
:- use_module('./XSD.pl').
:- use_module('./OWL.pl').
:- use_module('./QUDT.pl').
:- use_module('./portray.pl').
:- use_module('./DUL/Object.pl').
:- use_module('./DUL/Event.pl').

% load additional ontologies
:- tripledb_load(
        'http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl',
        [ namespace(io)
        ]).
:- tripledb_load(
        'http://knowrob.org/kb/knowrob.owl',
        [ namespace(knowrob)
        ]).

% load init files in sub-directories
:- use_directory('DUL').
:- use_directory('SOMA').

%
:- use_module('./notify.pl').
