/*
  Copyright (C) 2010-2014 Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
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

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).

:- use_module(library('knowrob/actions')).
:- use_module(library('knowrob/action_effects')).
:- use_module(library('knowrob/action_planning')).
:- use_module(library('knowrob/object_change')).

:- owl_parser:owl_parse('package://knowrob_actions/owl/action-effects.owl').
:- owl_parser:owl_parse('package://knowrob_actions/owl/object-change.owl').

:- rdf_db:rdf_register_ns(make_pancakes, 'http://knowrob.org/kb/pancake-making.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(action_effects, 'http://knowrob.org/kb/action-effects.owl#', [keep(true)]).

%:- register_ros_package(flanagan).
%:- use_module(library('flanagan')).
%:- use_module(library('robcog-flanagan')).
%:- use_module(library('tokenizer')).
%:- use_module(library('esg')).
%:- use_module(library('parser')).

%:- owl_parser:owl_parse('package://flanagan/owl/actions.owl').

%:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

%:- rdf_db:rdf_register_ns(ease, 'http://www.ease.org/ont/ease.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(allen, 'http://www.ease.org/ont/allen.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(actions, 'http://www.ease.org/ont/actions.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(motions, 'http://www.ease.org/ont/motions.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(states, 'http://www.ease.org/ont/states.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(qualities, 'http://www.ease.org/ont/qualities.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(objects, 'http://www.ease.org/ont/objects.owl#', [keep(true)]).
%:- rdf_db:rdf_register_ns(force_dynamics, 'http://www.ease.org/ont/force-dynamics.owl#', [keep(true)]).

