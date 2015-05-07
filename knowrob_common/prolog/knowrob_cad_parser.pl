/** <module> Predicates for parsing cad models

  Copyright (C) 2012 Stefan Profanter
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

@author Stefan Profanter
@license BSD

*/

:- module(knowrob_cad_parser,
    [
		get_model_path/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta get_model_path(r,?).

%% get_model_path(+Identifier,-Path) is det.
%
% searches for knowrob:pathToCadModel property
%
% @param Identifer  Object instance or class identifier
% @param Path       Found path
get_model_path(Identifier,Path) :-

  % Identifier is an instance, specification includes xsd:type
  (rdf_has(Identifier,knowrob:pathToCadModel,literal(type(_,Path))) ; 
  rdfs_individual_of(Identifier, Class), class_properties(Class, knowrob:pathToCadModel,literal(type(_,Path)));

  % Identifier is an instance, no xsd:type specified (included for compatibility reasons)
  rdf_has(Identifier,knowrob:pathToCadModel,literal(Path)) ; 
  rdfs_individual_of(Identifier, Class), class_properties(Class, knowrob:pathToCadModel,literal(Path))),! ;

  % Identifier is a class
  class_properties(Identifier, knowrob:pathToCadModel,literal(type(_,Path))),!.

