
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

% rules may generate discontiguous clauses.
% TODO: find a better solution then disabling this globally.
%:- style_check(-discontiguous).

% load some standard Prolog libraries into user
:- use_module(library('semweb/rdf_db'), [rdf_meta/1, rdf_current_ns/2]).
%:- use_module(library('semweb/rdf_portray')).
%:- use_module(library('semweb/rdfs')).

% more fancy module declarations
:- use_module(library('utility/module')).

% message formatting and logging
:- use_module(library('messages')).
:- use_module(library('utility/logging')).

% tooling around plunit
:- use_module(library('unittest')).

% TODO Load settings from file.
%:- ( getenv('KNOWROB_SETTINGS', File)
%	-> load_settings(File,[undefined(load)])
%	;  true
%	).

:- use_module(library('utility/functional')).
:- use_module(library('utility/filesystem')).
:- use_module(library('utility/algebra')).
%:- use_module(library('utility/atoms')).
%:- use_module(library('utility/threads')).
%:- use_module(library('utility/url')).

% auto-loaded models
:- use_module(library('xsd')).
:- use_module(library('qudt')).
:- use_module(library('ontology')).
