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

:- module(knowrob_computable,
    [
      rdfs_computable_property/2,
      rdfs_computable_triple/3,
      rdfs_computable_triple/4
    ]).
/** <module> Reasoning using procedural attachments, called "computables"

@author Bernhard Kirchlechner
@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/
%% TODO: allow caching data values without RDF representation
%% TODO: allow caching temporalized properties

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('lists')).
:- use_module(library('knowrob/knowrob')).

:- rdf_register_ns(computable, 'http://knowrob.org/kb/computable.owl#').

:- rdf_meta
  rdfs_computable_property(t,t),
  rdfs_computable_triple(t,t,t),
  rdfs_computable_triple(t,t,t,+).

%% rdfs_computable_property(+Property, -CP) is nondet.
%
% Search CP with the target Property.
%
rdfs_computable_property(Property, CP) :-
  ground(Property),
  rdfs_subproperty_of(SubProperty,Property),
  rdf_has(CP,computable:'target',SubProperty),
  once(rdfs_individual_of(CP,computable:'PrologProperty')).

rdfs_computable_property(CP, CP) :-
  ground(CP),
  rdfs_individual_of(CP, computable:'PrologProperty'), !.

%% rdfs_computable_triple(+Property, ?Frame, ?Value).
%
% Unify the property triple for a computable property.
% Full caching is enabled for everything but (+,-,-)
% and temporal computable properties.
%
rdfs_computable_triple(Property, Frame, Value) :-
  rdfs_computable_triple(Property, Frame, Value, _{}).

rdfs_computable_triple(Property,_,_,_) :-
  var(Property),!,
  throw(error(instantiation_error, _)).

rdfs_computable_triple(Property, Frame, Value, DBArgs) :-
  ground([Frame,Value]),!,
  rdfs_computable_property(Property, CP),
  rdfs_computable_triple(Property, Frame, Value, DBArgs, CP),
  !.

rdfs_computable_triple(Property, Frame, Value, DBArgs) :-
  rdfs_computable_property(Property, CP),
  rdfs_computable_triple(Property, Frame, Value, DBArgs, CP).
  
rdfs_computable_triple(_Property, Frame, Value, DBArgs, CP) :-
  \+ is_computable_cachable(CP),!,
  rdfs_compute_triple(CP, Frame, Value, DBArgs).

% both frame and value unbound -> no caching
rdfs_computable_triple(_Property, Frame, Value, DBArgs, CP) :-
  var(Frame), var(Value), !,
  rdfs_compute_triple(CP, Frame, Value, DBArgs).

% both frame and value bound
rdfs_computable_triple(Property, Frame, Value, DBArgs, CP) :-
  nonvar(Frame), nonvar(Value),
  ( rdf(Frame, Property, Value, cache) -> fail ; ( % cache miss
    rdfs_compute_triple(CP, Frame, Value, DBArgs),
    rdfs_computable_cache_values(Property, Frame, Value)
  )).

% frame bound, value unbound
rdfs_computable_triple(Property, Frame, Value, DBArgs, CP) :-
  nonvar(Frame), var(Value),
  (  rdf(computable:cachedAllValuesFor, Property, Frame, cache)
  -> fail ; (
     % compute values and cache them
     setof(MyValue, rdfs_compute_triple(CP, Frame, MyValue, DBArgs), Values),
     rdf_assert(computable:cachedAllValuesFor, Property, Frame, cache),
     maplist(rdfs_computable_cache_values(Property, Frame), Values),
     member(Value, Values)
  )).

% frame unbound, value bound
rdfs_computable_triple(Property, Frame, Value, DBArgs, CP) :-
  var(Frame), nonvar(Value), 
  (  rdf(Property, Value, computable:cachedAllFramesFor, cache)
  -> fail ; (
     setof(MyFrame, rdfs_compute_triple(CP, MyFrame, Value, DBArgs), Frames),
     rdf_assert(Property, Value, computable:cachedAllFramesFor, cache),
     maplist(rdfs_computable_cache_frames(Property, Value), Frames),
     member(Frame, Frames)
  )).

% Helpers for caching...
is_computable_cachable(CP) :-
  \+ rdfs_individual_of(CP, computable:'PrologTemporalProperty'),
  rdf_has(CP, computable:cache, literal(type(_, cache))), !.

rdfs_computable_cache_values(Property, Frame, Value) :-
  kb_assert(Frame, Property, Value, _{graph:cache}).

rdfs_computable_cache_frames(Property, Value, Frame) :-
  kb_assert(Frame, Property, Value, _{graph:cache}).


%% rdfs_compute_triple(CP, ?Frame, ?Value, +Interval).
%
% Unify the property triple for a computable property.
% No caching is performed.
%
rdfs_compute_triple(CP, Frame, Value) :-
  rdfs_compute_triple(CP, Frame, Value, _{}).

rdfs_compute_triple(CP, Frame, Value, DBArgs) :-
  catch(
    rdfs_compute_triple_(CP, Frame, Value, DBArgs),
    error(instantiation_error, _),
    fail
  ).

rdfs_compute_triple_(CP, Frame, Value, DBArgs) :-
  rdfs_computable_args(CP, Frame, Value, DBArgs, Args),
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

rdfs_computable_args(CP, Frame, Value, DBArgs, [Frame, Value, Stamp]) :-
  rdfs_individual_of(CP,computable:'PrologTemporalProperty'),
  get_dict(during, DBArgs, Stamp),!.

rdfs_computable_args(_CP, Frame, Value, DBArgs, [Frame, Value]) :-
  \+ get_dict(during, DBArgs, _).

%%
computable_predicate(Command, Predicate) :-
  functor(Command, N, A),
  ( N=':' -> (
    Command =.. [':', Module,Command_i],
    functor(Command_i, N_i, A_i),
    Predicate=Module:N_i/A_i
  ) ; Predicate=N/A ).

%%
knowrob:vkb_has_triple(S,P,O,Args) :-
  rdfs_computable_triple(P,S,O,Args).
