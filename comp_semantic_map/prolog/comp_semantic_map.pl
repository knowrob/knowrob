/** <module> comp_semantic_map

This module contains all computables that calculate relations for a semantic map.

Copyright (c) 2011, Lars Kunze <kunzel@cs.tum.edu>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Intelligent Autonomous Systems Group/
      Technische Universitaet Muenchen nor the names of its contributors 
      may be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

@author Lars Kunze
@license BSD
*/

:- module(comp_semantic_map,
    [
     
     comp_number_of_levels/2,
     comp_number_of_stories/2,
     nth_level_of_building/3
    ]).

%%  hasRooms(L|B,R), hasLevels(B,L), objLoc(O,L), objLocWrtMap(O,L,M)

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).

:- owl_parser:owl_parse('../owl/comp_semantic_map.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(comp_sem_map, 'http://ias.cs.tum.edu/kb/comp_semantic_map.owl#', [keep(true)]).



% define predicates as rdf_meta predicates
% (i.e. rdf namespaces are automatically expanded)
:-  rdf_meta
  comp_number_of_levels(r,-),
  comp_number_of_stories(r,-),
  nth_level_of_building(@,r,r).


comp_number_of_levels(B, N):-
  setof(L, (owl_has(B, rdf:type,knowrob:'Building'),
            rdf_triple(knowrob:'hasLevels',B, L)), Ls),
  length(Ls, N).

comp_number_of_stories(B, N) :-
  setof(L, (owl_has(B, rdf:type, knowrob:'Building'),
            rdf_triple(knowrob:'hasLevels',B, L),
            owl_has(L, rdf:type, knowrob:'AboveGroundLevelInAConstruction')), Ls),
  length(Ls, N).

nth_level_of_building(Number, Building, Level) :-
  owl_has(Building, knowrob:hasLevels, Level), 
  owl_has(Level, knowrob:floorNumber, literal(N)),
  atom_number(N, Number).

