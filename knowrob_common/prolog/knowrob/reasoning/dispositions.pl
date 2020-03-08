/*
  Copyright (C) 2011-2014 Moritz Tenorth
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
*/

:- module('knowrob/reasoning/dispositions',
    [
      storage_place_for/2,
      storage_place_for_because/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- use_module(library('knowrob/lang/ask')).
:- use_module(library('knowrob/model/Object'),[
    object_disposition/3
]).

:-  rdf_meta
    storage_place_for(r,r),
    storage_place_for_because(r,r,r).


%% storage_place_for(St, ObjT) is nondet
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% 
storage_place_for(St, ObjT) :-
  storage_place_for_because(St, ObjT, _).

%% storage_place_for_because(St, ObjType, ObjT) is nondet
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% In addition to the storage place, this predicate further returns the superclass
% of Obj for which this information is asserted (e.g. Perishable)
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% @param ObjType  Class for which information about the storage place has been asserted
%
storage_place_for_because(Container,Object,PatientType) :-
  ground([Container,Object]) ->
  once(storage_place_for_because_(Container,Object,PatientType)) ;
  storage_place_for_because_(Container,Object,PatientType).

storage_place_for_because_(Container,Object,PatientType) :-
  atom(Object),
  rdfs_individual_of(Object,dul:'Entity'),!,
  kb_type_of(Object,ObjType),
  storage_place_for_because_(Container,ObjType,PatientType).

storage_place_for_because_(Container,ObjType,PatientType) :-
  atom(Container),
  object_disposition(Container, Disposition, ease_obj:'Insertion'),
  storage_place_for_because__(Disposition,ObjType,PatientType).

storage_place_for_because__(Disposition,ObjType,PatientType) :-
  % FIXME: bug in property_range, ease_obj:affordsTrigger range inferred as plain role,
  %        without including constraints derived from axioms of Disposition!
  property_range(Disposition,[ease_obj:affordsTrigger,dul:classifies],PatientType),
  rdfs_subclass_of(PatientType,dul:'PhysicalObject'),
  rdfs_subclass_of(ObjType,PatientType).
