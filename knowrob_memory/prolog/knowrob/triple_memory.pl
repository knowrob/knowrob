/*
  Copyright (C) 2019 Daniel Beßler
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

:- module(triple_memory,
    [
        mem_store_triple/3,
        mem_store_triple/4,
        mem_store_triple/5,
        mem_store_triple/6,
        mem_retrieve_triple/3,
        mem_retrieve_triple/4,
        mem_retrieve_triple/5,
        mem_update_triple_begin/2,
        mem_update_triple_end/2,
        mem_update_triple_value/2,
        mem_triple_stop/3,
        mem_triple_stop/4,
        mem_triple_value/3,
        mem_xsd_value/3,
        mem_triple_interval/2,
        mem_subject/1,
        mem_property/1,
        mem_triples_init/0
    ]).
/** <module> Storing and retrieving temporalized triples using mongo DB.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/rdfs')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/computable')).

:-  rdf_meta
    mem_store_triple(r,r,r),
    mem_store_triple(r,r,r,-),
    mem_store_triple(r,r,r,-,+),
    mem_store_triple(r,r,r,-,+,?),
    mem_retrieve_triple(r,r,r),
    mem_retrieve_triple(r,r,r,+),
    mem_retrieve_triple(r,r,r,+,?),
    mem_triple_stop(r,r,r),
    mem_triple_stop(r,r,r,+),
    mem_triple_value(+,r,?),
    mem_xsd_value(+,r,?),
    mem_subject(r),
    mem_property(r).

:- dynamic mem_subject/1,
           mem_property/1.

%% mem_triples_init
%
% Initialize the triple memory.
%
mem_triples_init :-
  mng_distinct_values(triples, subject, Subjects),
  mng_distinct_values(triples, property, Properties),
  forall( member(S,Subjects),   mem_add_subject(S) ),
  forall( member(P,Properties), mem_add_property(P) ).

%%
mem_add_subject(S) :-
  once( mem_subject(S)  ; asserta(mem_subject(S)) ).

%%
mem_add_property(List) :-
  is_list(List),!,
  forall(
    member(P,List),
    once( mem_property(P) ; asserta(mem_property(P)) )
  ).

mem_add_property(Sub) :-
  forall(
    rdfs_subproperty_of(Sub,P),
    once( mem_property(P) ; asserta(mem_property(P)) )
  ).

%% mem_store_triple(+Subject,+Property,+Value)
%% mem_store_triple(+Subject,+Property,+Value,-DBObject)
%% mem_store_triple(+Subject,+Property,+Value,-DBObject,+Begin)
%% mem_store_triple(+Subject,+Property,+Value,-DBObject,+Begin,+End)
%
% Stores a temporalized triple in a mongo DB
% collection named 'triples'.
% It is ensured that only one value is "active" at a time
% in case of functional properties, otherwhise "updating"
% triples in case they do not hold anymore needs to be
% taken care of externally.
%
mem_store_triple(Subject,Property,Value) :-
  mem_store_triple(Subject,Property,Value,_).

mem_store_triple(Subject,Property,Value,DBObject) :-
  current_time(Now),
  mem_store_triple(Subject,Property,Value,DBObject,Now,_).

mem_store_triple(Subject,Property,Value,DBObject,Begin) :-
  mem_store_triple(Subject,Property,Value,DBObject,Begin,_).

mem_store_triple(Subject,Property,Value,DBObject,Begin,End) :-
  %%
  ground([Subject,Property,Value,Begin]),
  %%
  ( rdfs_individual_of(Property,owl:'FunctionalProperty') ->
    mem_triple_stop(Subject,Property,_,Begin) ; true ),
  %%
  % TODO: not so nice to save the list of all super-properties in each sample!
  %          better use a separate collection?
  findall(Y,rdfs_subproperty_of(Property,Y),Ys),
  findall(X, (
    member(X, [
      'subject'-Subject,
      'property'-Property,
      'value'-Value,
      'begin'-Begin,
      'end'-End,
      'property_chain'-Ys
    ]),
    ground(X)
  ), Pairs),
  dict_pairs(Dict, _, Pairs),
  mng_store(triples, Dict, DBObject),
  %%
  mem_add_subject(Subject),
  mem_add_property(Ys).

%% mem_update_triple_begin(+DBObject,+Begin)
%
% Assign begin timestamp to existing document.
%
mem_update_triple_begin(DBObject,Begin) :-
  ground([DBObject,Begin]),
  mng_update(triples, DBObject, _{begin: Begin}).

%% mem_update_triple_end(+DBObject,+End)
%
% Assign end timestamp to existing document.
%
mem_update_triple_end(DBObject,End) :-
  ground([DBObject,End]),
  mng_update(triples, DBObject, _{end: End}).

%% mem_update_triple_value(+DBObject,+Value)
%
% Assign triple value to existing document.
%
mem_update_triple_value(DBObject,Value) :-
  ground([DBObject,Value]),
  mng_update(triples, DBObject, _{value: Value}).

%% mem_triple_stop(?Subject,?Property,?Value)
%% mem_triple_stop(?Subject,?Property,?Value,+Stamp)
%
% Assign given time stamp as end time for each matching
% triple active at that time.
%
mem_triple_stop(Subject,Property,Value) :-
  current_time(Now),
  mem_triple_stop(Subject,Property,Value,Now).

mem_triple_stop(Subject,Property,Value,Stamp) :-
  ground(Stamp),
  forall(
    % triples that are active
    mem_retrieve_triple(Subject,Property,Value,DBObject,Stamp),
    % assign new end time
    mem_update_triple_end(DBObject,Stamp)
  ).

%% mem_retrieve_triple(?Subject,?Property,?Value,-DBObject)
%% mem_retrieve_triple(?Subject,?Property,?Value,-DBObject,?Stamp)
%
% Retrieve all matching triples from mong DB that are
% active at the given time.
% In case no time is given, the current time is used --
% i.e., to ask for what the properties are that are active
% at the moment.
%
mem_retrieve_triple(Subject,Property,Value) :-
  mem_retrieve_triple(Subject,Property,Value,_).

mem_retrieve_triple(Subject,Property,Value,DBObject) :-
  current_time(Now),
  mem_retrieve_triple(Subject,Property,Value,DBObject,Now).

mem_retrieve_triple(Subject,Property,Value,DBObject,Stamp) :-
  % build query pattern
  findall(X, ((
      member(X, [
        ['subject', 'is', Subject],
        ['property_chain', 'is', Property],
        ['value', 'is', Value]
      ])
      ; X=['begin', '<=', Stamp]
      ; X=or([
            ['end', '>=', Stamp],
            ['end', 'exists', false]
          ])
      ),
      ground(X)),
      Pattern),
  % execute the query
  mng_cursor(triples, Pattern, DBCursor),
  mng_cursor_descending(DBCursor, 'begin', DBCursorDescending),
  mng_cursor_read(DBCursorDescending, DBObject),
  % bind output
  ( ground(Property) -> true ; mng_get_string(DBObject,property,Property)),
  ( ground(Value)    -> true ; mem_triple_value(DBObject,Property,Value)),
  ( ground(Stamp)    -> true ; mng_get_long(DBObject,begin,Stamp)),
  ( ground(Subject)  -> true ; mng_get_string(DBObject,subject,Subject)).

%% mem_triple_value(+DBObj,+P,?Val)
%
% Retrieves the value of the 'value' field
% of a mongo triple document.
%
mem_triple_value(DBObj,P,Val) :-
  rdfs_individual_of(P,owl:'ObjectProperty'),!,
  mng_get_string(DBObj,value,Val).

mem_triple_value(DBObj,P,Val) :-
  rdf_phas(P,rdfs:range,Range),
  mem_xsd_value(DBObj,Range,Val),!.

mem_triple_value(DBObj,_,Val) :-
  mng_get_string(DBObj,value,Val).

%% mem_xsd_value(+DBObj,?XSD_Type,?Val)
%
% Retrieves the value of the 'value' field
% of a mongo triple document.
%
mem_xsd_value(DBObj,XSD_Type,Val) :-
  rdf_equal(XSD_Type,xsd:float),!,
  mng_get_double(DBObj,value,Val).

mem_xsd_value(DBObj,XSD_Type,Val) :-
  rdf_equal(XSD_Type,xsd:double),!,
  mng_get_double(DBObj,value,Val).

mem_xsd_value(DBObj,XSD_Type,Val) :-
  rdf_equal(XSD_Type,xsd:long),!,
  mng_get_long(DBObj,value,Val).

mem_xsd_value(DBObj,_XSD_Type,Val) :-
  mng_get_string(DBObj,value,Val).

%% mem_triple_interval(+DBObj,?Interval)
%
mem_triple_interval(DBObject,Interval) :-
  mng_get_long(DBObject,begin,Begin),
  ( mng_get_long(DBObject,end,End) ->
    Interval=[Begin,End] ;
    Interval=[Begin] ).

%%
% Integration of triple memory into virtual KB.
%
rdfs_computable:rdfs_computable_triple_during(Property,Subject,Value,[Stamp]) :-
  ( ground(Subject)  -> mem_subject(Subject) ; true ),
  ( ground(Property) -> mem_property(Property) ; true ),
  mem_retrieve_triple(Subject,Property,Value,DBObject,Stamp),
  \+ mng_get_long(DBObject,end,_).

rdfs_computable:rdfs_computable_triple_during(Property,Subject,Value,[Begin,End]) :-
  ( ground(Subject)  -> mem_subject(Subject) ; true ),
  ( ground(Property) -> mem_property(Property) ; true ),
  mem_retrieve_triple(Subject,Property,Value,DBObject,Begin),
  ( mng_get_long(DBObject,end,X) -> 
  ( var(End) -> End=X ; X >= End )
  ; true ).
