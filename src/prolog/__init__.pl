
% this is a global initialization for Prolog-based
% reasoning in KnowRob. Several reasoner instances
% are able to access what is imported here.

% configure how Prolog prints terms
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options,
        [ quoted(true),
          portray(true),
          max_depth(0),
          attributes(portray)
        ]).
:- set_prolog_flag(float_format, '%.12g').

% rules may generate discontiguous clauses.
%:- style_check(-discontiguous).

% add the toplevel src directory as Prolog library_directory
:- prolog_load_context(directory, PrologDir),
   atom_concat(SrcDir, '/prolog', PrologDir),
   atom_concat(KnowRobDir, '/src', SrcDir),
   atom_concat(KnowRobDir, '/tests', TestDir),
   % expand library search path, e.g. used by use_module/2 to locate Prolog source files
   asserta(user:library_directory(PrologDir)),
   asserta(user:library_directory(SrcDir)),
   % expand file search path, e.g. used by absolute_file_name/3 predicate
   assertz(file_search_path(knowrob,KnowRobDir)),
   assertz(file_search_path(test, TestDir)).

% load common Prolog libraries
:- use_module(library('semweb/rdf_db'), [rdf_meta/1, rdf_current_ns/2, rdf_register_prefix/3]).
% register some common RDF namespaces
:- rdf_register_prefix(dul,  'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_register_prefix(soma, 'http://www.ease-crc.org/ont/SOMA.owl#', [keep(true)]).

% more fancy module declarations
:- use_module(library('module')).
% message formatting and logging
:- use_module(library('messages')).
:- use_module(library('logging')).
% tooling around plunit
:- use_module(library('unittest')).

:- dynamic user:defined_reasoner_setting/4.

:- use_module(library('filesystem')).
:- use_module(library('functional')).
:- use_module(library('algebra')).
:- use_module(library('url')).
%:- use_module(library('atom')).

% extensions for semantic web
:- use_module(library('semweb')).
:- sw_add_subgraph(user,common),
   sw_add_subgraph(test,user).

% auto-load common models
:- use_module(library('xsd')).
:- use_module(library('qudt')).

% predicates that interact with the QA system in some way
:- use_module(library('blackboard')).
