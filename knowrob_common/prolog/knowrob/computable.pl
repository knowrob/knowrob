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

:- module(rdfs_computable,
    [
      rdfs_computable_class/2,       % is the specified class computable?
      rdfs_computable_property/2,    % is the specified property computable?
      rdfs_computable_triple/3,      % calculate the value of computable properties
      rdfs_computable_triple_during/4,
      rdfs_computable_has/3,         % S, P, O maps to rdf_triple
      rdf_triple/3                   % P, S, O combine rdfs_computable_triple with rdf:rdf_has
    ]).
/** <module> Reasoning using procedural attachments, called "computables"

@author Bernhard Kirchlechner
@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/
%% FIXME: allow caching data values without RDF representation

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('lists')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).

:- rdf_register_ns(computable, 'http://knowrob.org/kb/computable.owl#').

:- rdf_meta
  rdfs_computable_class(t,t),
  rdfs_computable_property(t,t),
  rdfs_computable_triple(t,t,t),
  rdf_triple(t,t,t),
  rdfs_computable_has(r,r,r).

:- dynamic rdfs_computable_triple_during/4.
:- multifile rdfs_computable_triple_during/4.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Query predicates
%

%% rdfs_computable_has(?Frame, ?Property, ?Value) is nondet.
%
% Maps to rdf_triple(Property, Frame, Value).
%
rdfs_computable_has(Frame, Property, Value) :- rdf_triple(Property, Frame, Value).

%% rdf_triple(?Property, ?Frame, ?Value) is nondet.
%
% True for triples (Frame, Property, Value) ins the RDF store
% and which can be deduced from computable predicates.
%
rdf_triple(Property, Frame, Value) :-
  rdf_has(Frame, Property, Value).

rdf_triple(Property, Frame, Value) :-
  rdfs_subproperty_of(SubProperty, Property),
  catch(
    rdfs_computable_triple(SubProperty, Frame, Value),
    error(instantiation_error, _),
    fail).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Find computable definitions
%

%% rdfs_computable_class(+Class, -ComputableClass) is nondet.
%
% Search for computable classes with the target Class
%
rdfs_computable_class(Class, ComputableClass) :-
  rdf_has(ComputableClass, computable:'target', Class),
  once(rdfs_individual_of(ComputableClass,computable:'PrologClass')).

%% rdfs_computable_property(+Property, -CP) is nondet.
%
% Search CP with the target Property.
%
rdfs_computable_property(Property, CP) :-
  rdf_has(CP,computable:'target',Property),
  once(rdfs_individual_of(CP,computable:'PrologProperty')).
rdfs_computable_property(CP, CP) :-
  ground(CP),
  rdfs_individual_of(CP, computable:'PrologProperty'), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Perform computation (caching, call the different types of computables)
%

%% rdfs_computable_triple(+Property, ?Frame, ?Value).
%
% Unify the property triple for a computable property.
% Full caching is enabled for everything but (+,-,-)
% and temporal computable properties.
%
rdfs_computable_triple(Property, _, _) :-
  var(Property),!,
  throw(error(instantiation_error, _)).

rdfs_computable_triple(Property, Frame, Value) :-
  rdfs_computable_property(Property, CP),
  rdfs_computable_triple(CP, Property, Frame, Value).
  
rdfs_computable_triple(CP, Property, Frame, Value) :-
  \+ rdfs_computable_cachable(CP),!,
  rdfs_computable_triple_1(CP, Property, Frame, Value).

rdfs_computable_triple(CP, Property, Frame, Value) :-
  nonvar(Frame), nonvar(Value), % both frame and value bound
  ( rdf(Frame, Property, Value, cache) -> fail ; ( % cache miss
    rdfs_computable_triple_1(CP, Property, Frame, Value),
    rdfs_computable_cache_values(Property, Frame, Value)
  )).

rdfs_computable_triple(CP, Property, Frame, Value) :-
  nonvar(Frame), var(Value), % frame bound, value unbound
  (  rdf(computable:cachedAllValuesFor, Property, Frame, cache)
  -> fail ; (
     % compute values and cache them
     setof(MyValue, rdfs_computable_triple_1(CP, Property, Frame, MyValue), Values),
     rdf_assert(computable:cachedAllValuesFor, Property, Frame, cache),
     maplist(rdfs_computable_cache_values(Property, Frame), Values),
     member(Value, Values)
  )).
      
rdfs_computable_triple(CP, Property, Frame, Value) :-
  var(Frame), nonvar(Value), % frame unbound, value bound
  (  rdf(Property, Value, computable:cachedAllFramesFor, cache)
  -> fail ; (
     setof(MyFrame, rdfs_computable_triple_1(CP, Property, MyFrame, Value), Frames),
     rdf_assert(Property, Value, computable:cachedAllFramesFor, cache),
     maplist(rdfs_computable_cache_frames(Property, Value), Frames),
     member(Frame, Frames)
  )).

rdfs_computable_triple(CP, Property, Frame, Value) :-
  var(Frame), var(Value), % both frame and value unbound -> no caching
  rdfs_computable_triple_1(CP, Property, Frame, Value).

rdfs_computable_triple_1(CP, Property, Frame, Value) :-
  catch(
    rdfs_compute_triple(CP, Property, Frame, Value),
    error(instantiation_error, _),
    fail
  ).

% Helpers for caching...
rdfs_computable_cachable(CP) :-
  \+ rdfs_individual_of(CP, computable:'PrologTemporalProperty'),
  rdf_has(CP, computable:cache, literal(type(_, cache))), !.
rdfs_computable_cache_values(Property, Frame, Value) :- rdf_assert(Frame, Property, Value, cache).
rdfs_computable_cache_frames(Property, Value, Frame) :- rdf_assert(Frame, Property, Value, cache).

%%
rdfs_computable_triple_during(Property, Frame, Value, Interval) :-
  rdfs_subproperty_of(SubProperty, Property),
  rdfs_computable_property(SubProperty, CP),
  rdfs_computable_triple_during_(CP, SubProperty, Frame, Value, Interval).

rdfs_computable_triple_during_(CP, Property, Frame, Value, Interval) :-
  rdfs_individual_of(CP,computable:'PrologTemporalProperty'),!,
  rdfs_compute_triple(CP, Property, Frame, Value, Interval).

rdfs_computable_triple_during_(CP, Property, Frame, Value, _) :-
  rdfs_computable_triple(CP, Property, Frame, Value).


%% rdfs_compute_triple(CP, ?Property, ?Frame, ?Value, +Interval).
%
% Unify the property triple for a computable property.
% No caching is performed.
%
rdfs_compute_triple(CP, _Property, Frame, Value, Interval) :-
  ( rdfs_individual_of(CP,computable:'PrologTemporalProperty') -> 
    Args=[Frame, Value, Interval];
    Args=[Frame, Value] ),
  % get the Prolog predicate that is used for computation
  rdf_has(CP, computable:command, literal(type(_, Cmd))),
  % handle the case that the predicate is defined in another module
  ( term_to_atom(Module:Pred, Cmd) ->
  ( Goal=..[Pred|Args], Command=Module:Goal ) ;
  ( Command=..[Cmd|Args] )),
  % Make sure the predicate exists
  computable_predicate(Command, Predicate),
  once(( current_predicate(Predicate) ; (
         term_to_atom(Predicate, CmdAtom),
         atomic_list_concat([
            'Undefined computable ', CmdAtom, '.',
            ' Missing register_ros_package?'], ErrStr),
         ros_warn(ErrStr),
         fail
  ))),
  % execute the Prolog predicate (namespace expansion etc.)
  call(Command).

rdfs_compute_triple(CP, Property, Frame, Value) :-
  current_time(Instant),
  rdfs_compute_triple(CP, Property, Frame, Value, [Instant,Instant]).

computable_predicate(Command, Predicate) :-
  functor(Command, N, A),
  ( N=':' -> (
    Command =.. [':', Module,Command_i],
    functor(Command_i, N_i, A_i),
    Predicate=Module:N_i/A_i
  ) ; Predicate=N/A ).
