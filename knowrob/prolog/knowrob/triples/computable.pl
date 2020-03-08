/*
  Copyright (C) 2008-10 Bernhard Kirchlechner, Moritz Tenorth
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

:- module('knowrob/triples/computable',
    [
      rdfs_compute/3,
      rdfs_compute/4,
      rdfs_computable/1
    ]).
/** <module> Reasoning using procedural attachments, called "computables"

@author Bernhard Kirchlechner
@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/
%% TODO: allow caching data values without RDF representation
%% TODO: allow caching temporalized properties
%% TODO: can tabling be used to replace the caching?

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_meta
  computable_(+,r,t),
  rdfs_compute(t,t,t),
  rdfs_compute(t,t,t,+).

:- dynamic computable_store_/3.

%% 
:- op(1150, fx, user:(rdfs_computable)).
  
%% 
rdfs_computable(Computables) :-
  Computables=','(First,Rest),!,
  computable_(First),
  rdfs_computable(Rest).

rdfs_computable(Last) :-
  computable_(Last).

%%
computable_(Computable) :-
  Computable=..[Predicate,Property|Args],
  computable_(Predicate,Property,Args).

computable_(Predicate,Property,Configuration) :-
  % get the module where the predicate is declared
  prolog_load_context(module, M),
  ScopedPredicate=M:Predicate,
  % validate
  callable(ScopedPredicate),
  rdf_global_id(Property,Property_Atom),
  atom(Property_Atom),
  term_to_atom(ScopedPredicate,PredicateAtom),
  % remember the computable
  rdf_assert('ComputableStore',Property,PredicateAtom,computables),
  assertz(computable_store_(PredicateAtom,ScopedPredicate,Configuration)).

%% rdfs_compute(?Frame, +Property, ?Value).
%% rdfs_compute(?Frame, +Property, ?Value, +Time).
%
% Unify the property triple for a computable property.
% Full caching is enabled for everything but (+,-,-)
% and temporal computable properties.
%
rdfs_compute(Frame, Property, Value) :-
  get_time(Time),
  rdfs_compute(Frame, Property, Value, Time).

rdfs_compute(_,Property,_,_) :-
  var(Property),!,
  throw(error(instantiation_error, _)).

rdfs_compute(Frame, Property, Value, Time) :-
  rdf_has('ComputableStore',Property,Cmd),
  ( is_cachable_(Cmd) ->
    cached_triple_(Property, Frame, Value, Time, Cmd) ;
    compute_triple(Cmd, Frame, Value, Time)
  ).

% both frame and value unbound -> no caching
cached_triple_(_Property, Frame, Value, Time, Cmd) :-
  var(Frame), var(Value), !,
  compute_triple(Cmd, Frame, Value, Time).

% both frame and value bound
cached_triple_(Property, Frame, Value, Time, Cmd) :-
  nonvar(Frame), nonvar(Value), !,
  ( rdf(Frame, Property, Value, cache) -> fail ; ( % cache miss
    compute_triple(Cmd, Frame, Value, Time),
    computable_cache_values(Property, Frame, Value),
    !
  )).

% frame bound, value unbound
cached_triple_(Property, Frame, Value, Time, Cmd) :-
  nonvar(Frame), var(Value), !,
  (  rdf(cachedAllValuesFor, Property, Frame, cache)
  -> fail ; (
     setof(MyValue, compute_triple(Cmd, Frame, MyValue, Time), Values),
     rdf_assert(cachedAllValuesFor, Property, Frame, cache),
     maplist(computable_cache_values(Property, Frame), Values),
     member(Value, Values)
  )).

% frame unbound, value bound
cached_triple_(Property, Frame, Value, Time, Cmd) :-
  var(Frame), nonvar(Value), !,
  (  rdf(Property, Value, cachedAllFramesFor, cache)
  -> fail ; (
     setof(MyFrame, compute_triple(Cmd, MyFrame, Value, Time), Frames),
     rdf_assert(Property, Value, cachedAllFramesFor, cache),
     maplist(computable_cache_frames(Property, Value), Frames),
     member(Frame, Frames)
  )).

%%
computable_cache_values(Property, Frame, Value) :-
  rdf_assert(Frame, Property, Value, cache).
%%
computable_cache_frames(Property, Value, Frame) :-
  rdf_assert(Frame, Property, Value, cache).

%%
is_cachable_(Cmd) :-
  \+ is_temporalized_(Cmd),
  computable_store_(Cmd,_,Args),
  member(cache=true, Args), !.

is_temporalized_(Cmd) :-
  % predicates with three arguments are temporalized,
  % they have an additional time argument.
  computable_store_(Cmd,Mod:Predicate,_),
  current_predicate(Mod:Predicate/3).

%% 
compute_triple(Cmd, Frame, Value, Time) :-
  catch(
    compute_triple_(Cmd, Frame, Value, Time),
    error(instantiation_error, _),
    fail
  ).

compute_triple_(Cmd, Frame, Value, Time) :-
  term_to_atom(Module:Pred, Cmd),
  %%
  findall(X, (
    X=Frame; X=Value; (
      is_temporalized_(Cmd),
      X=Time
    )
  ),Args),
  Goal=..[Pred|Args],
  %%
  call(Module:Goal).
