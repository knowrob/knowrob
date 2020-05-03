
% configure how Prolog prints terms
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options,
        [ quoted(true),
          portray(true),
          max_depth(0),
          attributes(portray)
        ]).
:- set_prolog_flag(float_format, '%.12g').

% load some standard Prolog libraries into user
%:- use_module(library('semweb/rdf_db')).
%:- use_module(library('semweb/rdf_portray')).
%:- use_module(library('semweb/rdfs')).

% load utility modules into user
:- use_module('utility/module').
:- use_module('utility/algebra').
:- use_module('utility/atoms').
:- use_module('utility/filesystem').
:- use_module('utility/functional').

% load init files in sub-directories
:- use_directory('lang').
:- use_directory('db').
:- use_directory('model').
:- use_directory('reasoning').
:- use_directory('comm').
:- use_directory('vis').

% TODO: load additional modules
%env(PL_MODULES)
