%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dependencies


:- register_ros_package(knowrob_common).

:- use_module(library('jpl')).
:- jpl_set_default_jvm_opts(['-Xmx2048M']).


:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdf_edit')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_portray')).

:- use_module(library('owl')).
:- use_module(library('owl_parser')).
:- use_module(library('rdfs_computable')).


:- use_module(library('util')).
:- use_module(library('knowrob_owl')).
% :- use_module(library('knowrob_units')).
:- use_module(library('owl_export')).
:- use_module(library('knowrob_cad_parser')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://knowrob_common/owl/owl.owl').
:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob.owl').

:- rdf_db:rdf_register_ns(rdfs,    'http://www.w3.org/2000/01/rdf-schema#',     [keep(true)]).
:- rdf_db:rdf_register_ns(owl,     'http://www.w3.org/2002/07/owl#',            [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',     [keep(true)]).


% convenience: set some Prolog flags in order *not to* trim printed lists with [...]
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options, [quoted(true), portray(true), max_depth(0), attributes(portray)]).

:- set_prolog_flag(float_format, '%.12g').



% load and configure unit testing environment
:- use_module(library(plunit)).
:- set_test_options([load('always'),
                     run('make'),
                     silent(true)]).
