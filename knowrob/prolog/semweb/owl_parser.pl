/*
  Copyright (C) 2014 Moritz Tenorth
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


:- module(owl_parser, [
      owl_parse/1,
      owl_parse/2
   ]).
/** <module> Methods for recursively parsing OWL files

@author Moritz Tenorth
@license BSD
*/

:- use_module(library('semweb/rdf_db.pl')).
:- use_module(library('semweb/rdf_edit.pl')).
:- use_module(library('semweb/rdfs.pl')).
:- use_module(library('url.pl')).
:- use_module(library('http/http_open.pl')).



%% owl_parse(+URL)
%% owl_parse(+URL,+Graph)
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
owl_parse(URL)        :- owl_parse(URL,[URL],user).
owl_parse(URL, Graph) :- owl_parse(URL,[URL],Graph).

:- dynamic owl_file_loaded/1,
           registry/2.

%% registry(?URI,?PkgName) nondet.
%
% URI to ROS package name. Ontology files local to the package are preferred.
%

owl_parse(URL,_,_) :-
  owl_file_loaded(URL), !.
owl_parse(URL,Imported,Graph) :-
  owl_parse_1(URL,Graph), !,
  (   rdf(_,'http://www.w3.org/2002/07/owl#imports',Import_URL),
      not( owl_file_loaded(Import_URL)),!,
      owl_parse(Import_URL,[Import_URL|Imported],Graph)
    ; true).

owl_parse_1(URL,Graph) :-
  file_base_name(URL,FileName),
  file_directory_name(URL,Prefix),
  owl_parser:registry(Prefix,Pkg),
  ros_package_path(Pkg,PkgPath),
  atomic_list_concat([PkgPath,owl,FileName],'/',LocalPath),
  exists_file(LocalPath),
  owl_parse_1(LocalPath,Graph),
  assertz(owl_file_loaded(URL)),
  !.

owl_parse_1(URL,Graph) :-
  sub_string(URL,0,4,_,'http'), !,
  http_open(URL,RDF_Stream,[]),
  owl_load(URL,RDF_Stream,Graph),
  close(RDF_Stream).

owl_parse_1(URL,Graph) :-
  ros_path(URL,GlobalPath), !,
  owl_parse(GlobalPath,Graph),
  assert(owl_file_loaded(URL)).

owl_parse_1(URL,Graph) :-
  owl_load(URL,URL,Graph).

owl_parse_assert(Graph, rdf(S,P,O)) :- rdf_assert(S,P,O,Graph).

owl_load(Id, URL, Graph) :-
  load_rdf(URL, Triples, [blank_nodes(noshare)]),
  maplist(owl_parse_assert(Graph), Triples),
  assert(owl_file_loaded(Id)).
