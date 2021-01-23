
% load modules into user
:- use_module('RDFS').
:- use_module('XSD').
:- use_module('OWL').
:- use_module('QUDT').
:- use_module('DUL').
:- use_module('portray').

% load additional ontologies
:- load_owl('http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl',
	[ namespace(io) ]).
:- load_owl('http://knowrob.org/kb/knowrob.owl',
	[ namespace(knowrob) ]).

% load init files in sub-directories
:- use_directory('SOMA').
