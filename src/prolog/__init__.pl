
% this init file is intended to be loaded by the
% PrologReasoner as soon as it starts.

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
% TODO: find a better solution then disabling this globally.
%:- style_check(-discontiguous).

% add the toplevel src directory as Prolog library_directory
:- prolog_load_context(directory, PrologDir),
   atom_concat(SrcDir, '/prolog', PrologDir),
   asserta(user:library_directory(PrologDir)),
   asserta(user:library_directory(SrcDir)).

% load some standard Prolog libraries into user
:- use_module(library('semweb/rdf_db'), [rdf_meta/1, rdf_current_ns/2, rdf_register_prefix/3]).

% register some custom RDF namespaces
:- rdf_register_prefix(dul,  'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_register_prefix(soma, 'http://www.ease-crc.org/ont/SOMA.owl#', [keep(true)]).

% more fancy module declarations
:- use_module(library('module')).

% message formatting and logging
:- use_module(library('messages')).
:- use_module(library('logging')).

% tooling around plunit
:- use_module(library('unittest')).

% TODO Load settings from file.
%:- ( getenv('KNOWROB_SETTINGS', File)
%	-> load_settings(File,[undefined(load)])
%	;  true
%	).

:- use_module(library('filesystem')).
:- use_module(library('functional')).
:- use_module(library('algebra')).
:- use_module(library('url')).
%:- use_module(library('atom')).

% auto-loaded models
:- use_module(library('xsd')).
:- use_module(library('qudt')).
:- use_module(library('semweb_ext')).
