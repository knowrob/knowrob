
% load modules into user
:- use_module('./RDFS.pl').
:- use_module('./XSD.pl').
:- use_module('./OWL.pl').
:- use_module('./QUDT.pl').
:- use_module('./portray.pl').
:- use_module('./DUL/Object.pl').
:- use_module('./DUL/Event.pl').

% load additional ontologies
:- load_owl(
        'http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl',
        [ namespace(io)
        ]).
:- load_owl(
        'http://knowrob.org/kb/knowrob.owl',
        [ namespace(knowrob)
        ]).

% load init files in sub-directories
:- use_directory('DUL').
:- use_directory('SOMA').

%
:- use_module('./notify.pl').
:- use_module('./neem_validation.pl').
