
:- module(owl_parser, [ owl_parse/1 ]).

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('url.pl')).
:- use_module(library('http/http_open.pl')).



owl_parse(URL) :-
  owl_parse_1(URL,[URL]).
  

owl_parse_1(URL,Imported) :-

  ((sub_string(URL,0,4,_,'http'), !,
    http_open(URL,RDF_Stream,[]),
    rdf_load(RDF_Stream,[blank_nodes(noshare)]),
    close(RDF_Stream))
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

    rdf_load(GlobalPath,[blank_nodes(noshare)])
    ) ; (
    RDF_Stream = URL, % URL is a file name
    rdf_load(RDF_Stream,[blank_nodes(noshare)]))
  ),
  (   rdf(_,'http://www.w3.org/2002/07/owl#imports',Import_URL),
      not( member(Import_URL, Imported)),!,
      owl_parse_1(Import_URL,[Import_URL|Imported])
    ; true).

