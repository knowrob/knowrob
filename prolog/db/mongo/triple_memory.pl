/*
  Copyright (C) 2019-2020 Daniel Beßler
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
        mem_retrieve_triple/3,
        mem_retrieve_triple/4,
        mem_triple_stop/3,
        mem_triple_stop/4,
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

:- use_module(library('knowrob/mongo')).
:- use_module(library('knowrob/memory'), [mem_db_name/1]).

:-  rdf_meta
    mem_store_triple(r,r,r),
    mem_store_triple(r,r,r,+),
    mem_store_triple(r,r,r,+,?),
    mem_retrieve_triple(r,r,r),
    mem_retrieve_triple(r,r,r,+),
    mem_retrieve_triple(r,r,r,+,?),
    mem_triple_stop(r,r,r),
    mem_triple_stop(r,r,r,+),
    mem_subject(r),
    mem_property(r).

:- dynamic mem_subject/1,
           mem_property/1.

%% mem_triples_init
%
% Initialize the triple memory.
%
mem_triples_init :-
  mem_db_name(DB),
  mng_distinct_values(DB, triples, subject, Subjects),
  mng_distinct_values(DB, triples, property, Properties),
  forall( member(S,Subjects),   mem_add_subject(S) ),
  forall( member(P,Properties), mem_add_property(P) ).

%%
mem_add_subject(S) :-
  once( mem_subject(S)  ; asserta(mem_subject(S)) ).

%%
mem_add_property(Sub) :-
  forall(
    rdfs_subproperty_of(Sub,P),
    once( mem_property(P) ; asserta(mem_property(P)) )
  ).

%% mem_store_triple(+Subject,+Property,+Value)
%% mem_store_triple(+Subject,+Property,+Value,+Begin)
%% mem_store_triple(+Subject,+Property,+Value,+Begin,+End)
%
% Stores a temporalized triple in a mongo DB
% collection named 'triples'.
% It is ensured that only one value is "active" at a time
% in case of functional properties, otherwhise "updating"
% triples in case they do not hold anymore needs to be
% taken care of externally.
%
mem_store_triple(Subject,Property,Value) :-
  get_time(Now),
  mem_store_triple(Subject,Property,Value,Now,_).

mem_store_triple(Subject,Property,Value,Begin) :-
  mem_store_triple(Subject,Property,Value,Begin,_).

mem_store_triple(Subject,Property,Value,Begin,End) :-
  %%
  ground([Subject,Property,Value,Begin]),
  mem_triple_typed_value_(Property,Value,TypedValue),
  %%
  ( rdfs_individual_of(Property,owl:'FunctionalProperty') ->
    mem_triple_stop(Subject,Property,_,Begin) ; true ),
  %%
  % TODO: not so nice to save the list of all super-properties in each sample!
  %          better use a separate collection?
  findall(string(Y),rdfs_subproperty_of(Property,Y),Ys),
  findall(X, (
    ( X=['subject',string(Subject)];
      X=['property',string(Property)];
      X=['value',TypedValue];
      X=['begin',time(Begin)];
      X=['end',time(End)];
      X=['property_chain',array(Ys)]
    ),
    ground(X)
  ), Doc),
  mem_db_name(DB),
  mng_store(DB, triples, Doc),
  %%
  mem_add_subject(Subject),
  forall(
    member(string(P),Ys),
    once( mem_property(P) ; asserta(mem_property(P)) )
  ).

%% mem_triple_stop(?Subject,?Property,?Value)
%% mem_triple_stop(?Subject,?Property,?Value,+Stamp)
%
% Assign given time stamp as end time for each matching
% triple active at that time.
%
mem_triple_stop(Subject,Property,Value) :-
  get_time(Now),
  mem_triple_stop(Subject,Property,Value,Now).

mem_triple_stop(Subject,Property,Value,Stamp) :-
  ground(Stamp),
  mem_triple_typed_value_(Property,Value,TypedValue),
  mem_db_name(DB),
  findall(X, (
    ( X=['subject',        ['$eq',string(Subject)]];
      X=['property_chain', ['$eq',string(Property)]];
      X=['value',          ['$eq',TypedValue]];
      X=['end',            ['$exists',bool(0)]]
    ),
    ground(X)
  ), Query),
  mng_update(DB, triples, Query, ['end', time(Stamp)]).

%% mem_retrieve_triple(?Subject,?Property,?Value)
%% mem_retrieve_triple(?Subject,?Property,?Value,?Stamp)
%
% Retrieve all matching triples from mong DB that are
% active at the given time.
% In case no time is given, the current time is used --
% i.e., to ask for what the properties are that are active
% at the moment.
%
mem_retrieve_triple(Subject,Property,Value) :-
  get_time(Now),
  mem_retrieve_triple(Subject,Property,Value,Now).

mem_retrieve_triple(Subject,Property,Value,Stamp) :-
  mem_db_name(DB),
  mem_triple_typed_value_(Property,Value,TypedValue),
  % execute the query
  setup_call_cleanup(
  %% setup
  ( mng_cursor_create(DB,triples,Cursor),
    mng_cursor_descending(Cursor,'begin'),
    mng_cursor_limit(Cursor,1),
    forall(
      ( ( X=['subject',        ['$eq',string(Subject)]];
          X=['property_chain', string(Property)];
          X=['value',          ['$eq',TypedValue]];
          X=['begin',          ['$lte',time(Stamp)]];
          X=['$or', list([['end',    ['$gte',time(Stamp)]],
                          ['end',    ['$exists',bool(0)]]])]
        ),
        ground(X)),
      ( mng_cursor_filter(Cursor,X) )
    )
  ),
  %% call
  ( mng_cursor_materialize(Cursor, TripleDict),
    % bind output
    ( ground(Property) -> true ; mng_get_dict(property, TripleDict, Property)),
    ( ground(Value)    -> true ; mng_get_dict(value,    TripleDict, Value)),
    ( ground(Stamp)    -> true ; mng_get_dict(begin,    TripleDict, Stamp)),
    ( ground(Subject)  -> true ; mng_get_dict(subject,  TripleDict, Subject))
  ),
  %% cleanup
  ( mng_cursor_destroy(Cursor) )
  ).

%%
mem_triple_typed_value_(P,Value,string(Value)) :-
  atom(P),
  rdfs_individual_of(P,owl:'ObjectProperty'),!.

mem_triple_typed_value_(P,Value,TypedValue) :-
  atom(P),
  rdf_phas(P,rdfs:range,Range),
  mem_triple_typed_xsd_(Range,Value,TypedValue),!.

mem_triple_typed_value_(_P,Value,string(Value)).

%%
mem_triple_typed_xsd_(XSD_Type,Value,double(Value)) :-
  ( rdf_equal(XSD_Type,xsd:float) ;
    rdf_equal(XSD_Type,xsd:double)
  ),!.

mem_triple_typed_xsd_(XSD_Type,Value,int(Value)) :-
  ( rdf_equal(XSD_Type,xsd:long) ;
    rdf_equal(XSD_Type,xsd:int) ;
    rdf_equal(XSD_Type,xsd:short) ;
    rdf_equal(XSD_Type,xsd:byte)
  ),!.

mem_triple_typed_xsd_(_Range,Value,string(Value)).

%%
% expand knowrob:vkb_has_triple
%
% FIXME
'knowrob/triples/triple_store':vkb_has_triple(S,P,O,DBArgs) :-
  ( ground(S) -> mem_subject(S) ; true ),
  ( ground(P) -> mem_property(P) ; true ),
  ( get_dict(during,DBArgs,Stamp) ->
    mem_retrieve_triple(S,P,O,Stamp) ;
    mem_retrieve_triple(S,P,O)
  ).

%%
% FIXME
:- 'knowrob/lang/tell':set_temporalized_db(
      triple_memory:mem_triple_start_,
      triple_memory:mem_triple_stop_).

%%
mem_triple_start_(S,P,O,DBArgs) :-
  ( get_dict(during,DBArgs,Stamp) ->
    mem_store_triple(S,P,O,Stamp) ;
    mem_store_triple(S,P,O)
  ).

mem_triple_stop_(S,P,O,DBArgs) :-
  ( get_dict(during,DBArgs,Stamp) ->
    mem_triple_stop(S,P,O,Stamp) ;
    mem_triple_stop(S,P,O)
  ).
