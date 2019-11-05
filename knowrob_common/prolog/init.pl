/*
  Copyright (C) 2009-2014 Moritz Tenorth
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

@author Moritz Tenorth
@license BSD

*/

:- use_module(library('jpl')).
:- jpl_set_default_jvm_opts(['-Xmx2048M']).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdf_edit')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_portray')).

:- rdf_db:rdf_register_ns(rdfs,      'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl,       'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul,       'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease,      'http://www.ease-crc.org/ont/EASE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_act,  'http://www.ease-crc.org/ont/EASE-ACT.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_wf,   'http://www.ease-crc.org/ont/EASE-WF.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_obj,  'http://www.ease-crc.org/ont/EASE-OBJ.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_proc, 'http://www.ease-crc.org/ont/EASE-PROC.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ease_state, 'http://www.ease-crc.org/ont/EASE-STATE.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,   'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(ros,       'http://www.ease-crc.org/ont/ROS.owl#', [keep(true)]).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
%:- use_module(library('semweb/owl_export')).

% URI to ROS package name.
owl_parser:registry('http://www.ontologydesignpatterns.org/ont/dul', dul).
owl_parser:registry('http://www.ease-crc.org/ont', ease_ontology).
owl_parser:registry('http://www.ease-crc.org/ont', rosowl).
owl_parser:registry('http://knowrob.org/kb', knowrob_common).

:- register_ros_package(knowrob_common).

% parse OWL files, register name spaces
:- owl_parser:owl_parse('package://knowrob_common/owl/owl.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/ROS.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/EASE.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/EASE-ACT.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/EASE-WF.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/EASE-IO.owl').
:- owl_parser:owl_parse('http://www.ease-crc.org/ont/EASE-OBJ.owl').
:- owl_parser:owl_parse('http://knowrob.org/kb/knowrob.owl').

:- use_module(library('knowrob/utility/atoms')).

:- use_module(library('knowrob/xsd')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/computable')).
% :- use_module(library('knowrob/units')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/rosowl')).
:- use_module(library('knowrob/wup_similarity')).

% convenience: set some Prolog flags in order *not to* trim printed lists with [...]
:- set_prolog_flag(toplevel_print_anon, false).
:- set_prolog_flag(toplevel_print_options, [quoted(true), portray(true), max_depth(0), attributes(portray)]).

:- set_prolog_flag(float_format, '%.12g').
