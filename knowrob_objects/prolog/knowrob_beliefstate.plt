/** <module> knowrob_beliefstate

  Copyright (C) 2017 Mihai Pomarlan

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

  @author Mihai Pomarlan
  @license BSD
*/

:- begin_tests(knowrob_beliefstate).

:- use_module(library('lists')).
:- use_module(library('util')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('knowrob_owl')).
:- use_module(library('random')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- dynamic test_object/1.

test(belief_new_object) :-
  belief_new_object(knowrob:'Cup', Cup),
  rdfs_individual_of(Cup, knowrob:'Cup'),
  rdfs_individual_of(Cup, owl:'NamedIndividual'),
  rdf_has(Cup, srdl2comp:urdfName, _),
  assertz(test_object(Cup)).

test(belief_at_update) :-
  test_object(Cup),
  belief_at_update(Cup, ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]]),
  rdf_has(Cup, knowrob:'pose', _), !.

test(belief_at_location_equal) :-
  belief_class_at_location(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], [0.0,0.0], Cup),
  test_object(Cup), !.

test(belief_at_location_close1, [fail]) :-
  belief_class_at_location(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], [0.0,0.0], Cup),
  test_object(Cup), !.

test(belief_at_location_close2) :-
  belief_class_at_location(knowrob:'Cup', ['map',_,[1.001,0.001,0.0],[1.0,0.0,0.0,0.0]], [0.5,0.0], Cup),
  test_object(Cup), !.

test(belief_at_update_class) :-
  test_object(Cup),
  belief_perceived_at(knowrob:'Milk', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], [0.0,0.0], Cup),
  \+ rdfs_individual_of(Cup, knowrob:'Cup'),
  rdfs_individual_of(Cup, knowrob:'Milk'), !.

test(belief_at_update_class2) :-
  test_object(Cup),
  belief_perceived_at(knowrob:'Cup', ['map',_,[1.0,0.0,0.0],[1.0,0.0,0.0,0.0]], [0.0,0.0], Cup),
  rdfs_individual_of(Cup, knowrob:'Cup'),
  \+ rdfs_individual_of(Cup, knowrob:'Milk'), !.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- end_tests(knowrob_beliefstate).
