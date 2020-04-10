
:- module(owl_parser,
    [
      owl_parse/1
    ]).
/** <module> Methods for recursively parsing OWL files

@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdf_db.pl'),  [ load_rdf/3 ]).
:- use_module(library('http/http_open.pl'), [ http_open/3 ]).

:- dynamic owl_file_loaded_/1.

% TODO: move out the ROS related stuff!!
:- dynamic registry/2.

%% registry(?URI,?PkgName) nondet.
%
% URI to ROS package name. Ontology files local to the package are preferred.
%

%% owl_parse(+URL)
%
% Parse an OWL file and load it into the local RDF database.
% 
% Resolves owl:imports and supports both the common URL formats
% (file paths, file:// or http://) and the package:// URLs used
% in ROS to reference files with respect to the surrounding ROS
% package.
%
% @param URL Local or global file path or URL of the forms file://, http:// or package://
% 
owl_parse(URL) :-
  owl_file_loaded_(URL),!.

owl_parse(URL) :-
  owl_parse_(URL),
  ( owl_new_import_(Import) ->
    owl_parse(Import);
    true
  ).

owl_parse_(URL) :-
  sub_string(URL,0,4,_,'http'), !,
  http_open(URL,RDF_Stream,[]),
  owl_load_(URL,RDF_Stream),
  close(RDF_Stream).

%owl_parse_(URL) :-
  %file_base_name(URL,FileName),
  %file_directory_name(URL,Prefix),
  %owl_parser:registry(Prefix,Pkg),
  %ros_package_path(Pkg,PkgPath),
  %atomic_list_concat([PkgPath,owl,FileName],'/',LocalPath),
  %exists_file(LocalPath),
  %owl_parse_(LocalPath),
  %assertz(owl_file_loaded_(URL)),
  %!.

%owl_parse_(URL) :-
  %ros_path(URL,GlobalPath), !,
  %owl_parse(GlobalPath),
  %assert(owl_file_loaded_(URL)).

owl_parse_(URL) :-
  % TODO: resolve ROS paths
  resolve_path(URL,Resolved),
  owl_load_(URL,Resolved).

%%
owl_new_import_(Import) :-
  ask( _ 'http://www.w3.org/2002/07/owl#imports' Import ),
  \+ owl_file_loaded_(Import).

%%
owl_load_(Id, URL) :-
  load_rdf(URL, Triples, [blank_nodes(noshare)]),
  tell(Triples),
  assert(owl_file_loaded_(Id)).
