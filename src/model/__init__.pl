
:- use_module('portray').

:- use_module('RDFS').
:- use_module('XSD').
:- use_module('OWL').
:- use_module('QUDT').
:- use_module('DUL').
:- use_module('SOMA').

% load additional ontologies
:- load_owl('http://knowrob.org/kb/knowrob.owl',
	[ namespace(knowrob) ]).
