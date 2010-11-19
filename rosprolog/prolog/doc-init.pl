
:- style_check(-discontiguous).

:- ['~/.plrc'].

:- dynamic   user:file_search_path/2.
:- multifile user:file_search_path/2.


:- load_files([ library(pldoc),
		library(doc_http),
		library(doc_latex),
		library(http/http_error)
	      ],
	      [ silent(true),
		if(not_loaded)
	      ]).
:- debug(pldoc).

:- use_module(library(pldoc/doc_files)).

%	Using doc_server(4004, []), the server   is publically available
%	on the internet. ?-  doc_server(4004)   only  allows access from
%	localhost.

% :- doc_server(4004, [root('/work/knowrob/trunk/')]).
%:- doc_server(4004).
