
:- module(owl_parser, [
      owl_parse/1
   ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('url.pl')).
:- use_module(library('http/http_open.pl')).



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
  owl_parse_1(URL,[URL]).
  


owl_parse_1(URL,Imported) :-

  ((sub_string(URL,0,4,_,'http'), !,
    http_open(URL,RDF_Stream,[]),
    rdf_load(RDF_Stream,[blank_nodes(noshare)]),
    close(RDF_Stream)),
    assert(owl_file_loaded(URL))
    ;
   (sub_string(URL,0,7,_,'package'), !,

    % retrieve part after package://
    sub_atom(URL, 10, _, 0, Path),
    atomic_list_concat(PathList, '/', Path),

    % determine package name and resolve path
    selectchk(Pkg, PathList, LocalPath),
    rospack_package_path(Pkg, PkgPath),

    % build global path and load OWL file
    atomic_list_concat([PkgPath|LocalPath], '/',  GlobalPath),

    rdf_load(GlobalPath,[blank_nodes(noshare)]),
    assert(owl_file_loaded(URL))
    ) ; (
    rdf_load(URL,[blank_nodes(noshare)])),
    assert(owl_file_loaded(URL))
  ),
  (   rdf(_,'http://www.w3.org/2002/07/owl#imports',Import_URL),
      not( owl_file_loaded(Import_URL)),!,
      owl_parse_1(Import_URL,[Import_URL|Imported])
    ; true).

