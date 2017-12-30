/*
  Copyright (C) 2011 Moritz Tenorth
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

:- module(object_change,
    [
      transformed_into/2,
      transformed_into_transitive/2,
      comp_thermicallyConnectedTo/2
    ]).
/** <module> Methods for reasoning about object changes caused by actions

@author Moritz Tenorth
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/owl')).

:- owl_parse('package://knowrob_actions/owl/object-change.owl').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:-  rdf_meta
        transformed_into(r, r),
        transformed_into_transitive(r, r),
        comp_thermicallyConnectedTo(r,r).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% object transformations
%

%% transformed_into(?From:iri, ?To:iri)
%
% Compute which objects have been transformed into which other ones
% by actions or processes. This predicate operates on the object
% modification graph created by the action projection rules
%
% @param From Input of some action
% @param To   Output created by this action
%
transformed_into(From, To) :-
  ( owl_has(Event, knowrob:thingIncorporated, From);
    owl_has(Event, knowrob:objectAddedTo, From);
    owl_has(Event, knowrob:inputsDestroyed, From);
    owl_has(Event, knowrob:inputsCommitted, From);
    owl_has(Event, knowrob:transformedObject, From);
    owl_has(Event, knowrob:objectRemoved, From);
    owl_has(Event, knowrob:objectOfStateChange, From);
    owl_has(Event, knowrob:outputsRemaining, From) ),

  ( owl_has(Event, knowrob:outputsRemaining, To);
    owl_has(Event, knowrob:outputsCreated, To)).


%% transformed_into_transitive(?From:iri, ?To:iri)
%
% Transitive version of the transformed_into predicate that tracks
% in- and outputs of actions over several steps and different
% properties
%
% @param From Input of some action
% @param To   Output created by this action
%
transformed_into_transitive(From, To) :-
  transformed_into(From, To).

transformed_into_transitive(From, To) :-
  transformed_into(From, Sth),
  From\=Sth,
  transformed_into_transitive(Sth, To).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% process-relevant object relations
%

%% comp_thermicallyConnectedTo(?Obj1:iri, ?Obj2:iri)
%
% Compute if a heat path exists between two objects. This is the case if
% they are either on top of each other or if one contains the other one
%
% @param Obj1 Object instance
% @param Obj2 Object instance
%
comp_thermicallyConnectedTo(Obj1, Obj2) :- once(
  rdf_triple(knowrob:'on-Physical', Obj1, Obj2);
  rdf_triple(knowrob:'on-Physical', Obj2, Obj1)).

comp_thermicallyConnectedTo(Obj1, Obj2) :- once(
  rdf_triple(knowrob:'in-ContGeneric', Obj1, Obj2);
  rdf_triple(knowrob:'in-ContGeneric', Obj2, Obj1)).

