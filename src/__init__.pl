
% configure how Prolog prints terms
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options,
        [ quoted(true),
          portray(true),
          max_depth(0),
          attributes(portray)
        ]).
:- set_prolog_flag(float_format, '%.12g').

% tell/ask queries generate discontiguous clauses.
% TODO: find a better solution then disabling this globally.
:- style_check(-discontiguous).

% load some standard Prolog libraries into user
:- use_module(library('semweb/rdf_db'), [rdf_meta/1, rdf_current_ns/2]).
%:- use_module(library('semweb/rdf_portray')).
%:- use_module(library('semweb/rdfs')).

% make sure library path is expanded
:- register_ros_package(knowrob).

% load utility modules into user
:- use_module('utility/module').
:- use_module('utility/algebra').
:- use_module('utility/atoms').
:- use_module('utility/filesystem').
:- use_module('utility/functional').
:- use_module('utility/url').

% register ROS packages to resolve IRI prefixes to local paths
:- ros_package_iri(dul,           'http://www.ontologydesignpatterns.org/ont/dul').
:- ros_package_iri(ease_ontology, 'http://www.ease-crc.org/ont').
:- ros_package_iri(rosowl,        'http://www.ease-crc.org/ont').
:- ros_package_iri(knowrob,       'http://knowrob.org/kb').
:- ros_package_iri(knowrob,       'http://www.w3.org/2002/07').

% initialize databases
:- use_directory('db').
:- tripledb_init.
:- tripledb_add_subgraph(tbox,common).
:- tripledb_add_subgraph(user,tbox).
:- tripledb_add_subgraph(test,user).

% load init files in sub-directories
:- use_directory('lang').
:- use_directory('model').
:- use_directory('reasoning').
:- use_directory('comm').
:- use_directory('vis').

% TODO: load additional modules
%env(PL_MODULES)
