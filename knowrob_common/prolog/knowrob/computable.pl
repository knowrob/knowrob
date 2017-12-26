/** <module> Reasoning using procedural attachments, called "computables"

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

@author Bernhard Kirchlechner, Moritz Tenorth
@license BSD

*/

:- module(rdfs_computable,
    [
      rdfs_computable_class/2,       % is the specified class computable?
      rdfs_computable_instance_of/2, % query computable instances of a class
      rdfs_computable_property/2,    % is the specified property computable?
      rdfs_computable_triple/3,      % calculate the value of computable properties
      rdfs_computable_prolog_triple/3,
      rdfs_computable_prolog_triple/4,
      rdfs_computable_prolog_triple/5,
      rdfs_computable_has/3,
      rdfs_instance_of/2,            % combine rdfs_computable_instance_of (with subclass handling) and rdfs:rdfs_individual_of
      rdf_triple/3,                  % combine rdfs_computable_triple with rdf:rdf_has
      rdfs_assert_prop_conc/2,       % assert property concatenations in a row
      rdfs_assert_prop_conc/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('lists')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).

:- rdf_register_ns(computable, 'http://knowrob.org/kb/computable.owl#').
:- rdf_meta
  rdfs_computable_class(t,t),
  rdfs_computable_instance_of(t,t),
  rdfs_computable_property(t,t),
  rdfs_computable_triple(t,t,t),
  rdfs_instance_of(t,t),
  rdf_triple(t,t,t),
  rdfs_assert_prop_conc(t,-),
  rdf_class_compare(?, r, r).

:- dynamic user:rdf_triple_hook/3.
:- multifile user:rdf_triple_hook/3.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Query predicates
%

rdfs_computable_has(Frame, Property, Value) :- rdf_triple(Property, Frame, Value).

%% rdf_triple(?Property, ?Frame, ?Value) is nondet.
%
% Hook entry that computes triples:
% (rdfs_computable_compute_property_concatenation, rdf_has, rdfs_computable_triple).
%
rdf_triple(Property, Frame, Value) :-
  rdf_has(Frame, Property, Value).

rdf_triple(Property, Frame, Value) :-
  rdfs_subproperty_of(SubProperty, Property),
  ( rdfs_computable_compute_property_concatenation(SubProperty, Frame, Value)
  ; catch( rdfs_computable_triple(SubProperty, Frame, Value), error(instantiation_error, _), fail)
  ; user:rdf_triple_hook(SubProperty, Frame, Value)
  ).

:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:range),
    rdfs_instance_of(Frame, computable:'PropertyConcatenation'),
    rdf_has(Frame, computable:'rest', RestProperty),
    rdf_triple(rdfs:range, RestProperty, Value)), X),
  assert(user:(X)).
:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdfs:domain),
    rdfs_instance_of(Frame, computable:'PropertyConcatenation'),
    rdf_has(Frame, computable:'first', FirstProperty),
    rdf_triple(rdfs:domain, FirstProperty, Value)), X),
  assert(user:(X)).

:- user:expand_term((rdf_triple_hook(Property, Frame, Value):-
    rdf_equal(Property, rdf:type),
    (nonvar(Frame); nonvar(Value)),
    rdfs_instance_of(Frame, Value))
  , X), assert(user:(X)).

%% rdfs_instance_of(?Resource, ?Class) is nondet.
%
% combine rdfs_computable_instance_of (with subclass handling) and rdfs:rdfs_individual_of
%
rdfs_instance_of(Resource, Class) :-
  nonvar(Resource),
  ( nonvar(Class) -> once((
    % check if Resource belongs to Class
    rdfs_individual_of(Resource, Class) ;
    rdfs_computable_instance_of_subclass(Resource, Class))) ;
    % compute the class of the given instance
    rdfs_individual_of(Resource, MyClass) ).

rdfs_instance_of(Resource, Class) :-
  var(Resource), nonvar(Class), (
  rdfs_individual_of(Resource, Class);
  rdfs_computable_instance_of_subclass(Resource, Class)).

rdfs_instance_of(Resource, Class) :-
  rdf_equal(rdf:type, Property),
  rdfs_computable_prolog_triple(Resource, Property, Class).

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

%% rdfs_computable_property(+Property, -ComputableProperty) is nondet.
%
% Search ComputableProperty with the target Property.
%
rdfs_computable_property(Property, ComputableProperty) :-
  rdf_has(ComputableProperty,computable:'target',Property),
  once(rdfs_individual_of(ComputableProperty,computable:'PrologProperty')).
rdfs_computable_property(ComputableProperty, ComputableProperty) :-
  ground(ComputableProperty),
  rdfs_individual_of(ComputableProperty, computable:'PrologProperty'), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Perform computation (caching, call the different types of computables)
%

%% rdfs_computable_instance_of_subclass(?Instance, ?Class) is nondet.
%
% Compute computable instances of all sub-classes of Class
%
rdfs_computable_instance_of_subclass(Resource, Class) :-
  nonvar(Class), !,
  findall(SC, rdfs_subclass_of(SC, Class), SCs),
  member(SubClass, SCs),
  rdfs_computable_instance_of(Resource, SubClass).
rdfs_computable_instance_of_subclass(Resource, Class) :-
  nonvar(Resource), !,
  findall(SC, rdfs_subclass_of(SC, Class), SCs),
  member(SubClass, SCs),
  rdfs_subclass_of(SubClass, Class).
rdfs_computable_instance_of_subclass(_,_) :-  % both are Variables
  throw(error(instantiation_error, _)).


%% rdfs_computable_instance_of(?Instance, ?Class) is nondet.
%
% Compute computable instances of Class
%
% At least one of Instance and Class needs to be nonvar.
%
rdfs_computable_instance_of(Instance, Class) :-
  var(Class), var(Instance),!,
  throw(error(instantiation_error, _)).
rdfs_computable_instance_of(Instance, Class) :-
  nonvar(Class), var(Instance), !, % compute instances
  findall(I, rdfs_computable_prolog_instance_of(I, Class), Is),
  member(Instance, Is).
rdfs_computable_instance_of(Instance, Class) :-
  var(Class), nonvar(Instance), !, % determine classes
  findall(C, rdfs_computable_prolog_instance_of(Instance, C), Cs),
  member(Class, Cs).
rdfs_computable_instance_of(Instance, Class) :-
  nonvar(Class), nonvar(Instance), !, % check type
  findall(C, rdfs_computable_prolog_instance_of(Instance, C), Cs),
  member(Cls, Cs),
  rdfs_subclass_of(Cls, Class).

%% rdfs_computable_triple(+Property, ?Frame, ?Value)
%
% Unify the property triple for a computable property.
% Full caching is enabled for everything but (+,-,-).
%
rdfs_computable_triple(Property, _, _) :-
  var(Property),!,
  throw(error(instantiation_error, _)).
  
rdfs_computable_triple(Property, Frame, Value) :-
  \+ rdfs_computable_cachable(Property),!,
  rdfs_computable_triple_1(Property, Frame, Value).

rdfs_computable_triple(Property, Frame, Value) :-
  nonvar(Frame), nonvar(Value), % both frame and value bound
  ( rdf(Property, Frame, Value, cache) -> true ; ( % cache miss
    rdfs_computable_triple_1(Property, Frame, Value),
    rdfs_computable_cache_values(Property, Frame, Value)
  )).

rdfs_computable_triple(Property, Frame, Value) :-
  nonvar(Frame), var(Value), % frame bound, value unbound
  (  rdf(computable:cachedAllValuesFor, Property, Frame, cache)
  -> rdf(Property, Frame, Value, cache) ; (
     % compute values and cache them
     setof(MyValue, rdfs_computable_triple_1(Property, Frame, MyValue), Values),
     rdf_assert(computable:cachedAllValuesFor, Property, Frame, cache),
     maplist(rdfs_computable_cache_values(Property, Frame), Values),
     member(Value, Values)
  )).
      
rdfs_computable_triple(Property, Frame, Value) :-
  var(Frame), nonvar(Value), % frame unbound, value bound
  (  rdf(Property, Value, computable:cachedAllFramesFor, cache)
  -> rdf(Property, Frame, Value, cache) ; (
     setof(MyFrame, rdfs_computable_triple_1(Property, MyFrame, Value), Frames),
     rdf_assert(Property, Value, computable:cachedAllFramesFor, cache),
     maplist(rdfs_computable_cache_frames(Property, Value), Frames),
     member(Frame, Frames)
  )).

rdfs_computable_triple(Property, Frame, Value) :-
  var(Frame), var(Value), % both frame and value unbound -> no caching
  rdfs_computable_triple_1(Property, Frame, Value).

% The real work is done here
rdfs_computable_triple_1(Property, Frame, Value) :-
  nonvar(Property),
  catch(
    rdfs_computable_prolog_triple(Property, Frame, Value),
    error(instantiation_error, _),
    fail
  ).

%% rdfs_computable_prolog_triple(?Property, ?Frame, ?Value).
%
% Evaluation of RDF triples using Prolog
%
rdfs_computable_prolog_triple(Property, Frame, Value) :-
  rdfs_computable_property(Property, ComputableProperty),
  ( rdfs_individual_of(ComputableProperty,computable:'PrologTemporalProperty') -> (
    current_time(Instant),
    rdfs_computable_prolog_triple(Property, Frame, Value, [[Instant,Instant]], ComputableProperty) );
    rdfs_computable_prolog_triple(Property, Frame, Value, [], ComputableProperty)
  ).

rdfs_computable_prolog_triple(Property, Frame, Value, CmdArgs) :-
  rdfs_computable_property(Property, ComputableProperty),
  rdfs_computable_prolog_triple(Property, Frame, Value, CmdArgs, ComputableProperty).

rdfs_computable_prolog_triple(Property, Frame, Value, CmdArgs, ComputableProperty) :-
  % get the Prolog predicate that is used for evaluation:
  rdf_has(ComputableProperty, computable:command, literal(type(_, Cmd))),
  % handle the case that the predicate is defined in another module
  ((term_to_atom(Module:Pred, Cmd)) ->
    (Command=Module:Pred) ;
    (Command=user:Cmd)),
  % execute the Prolog predicate (namespace expansion etc.)
  ( nonvar(Value) ->
    rdfs_computable_prolog_call(ComputableProperty, Command, Frame, Value, CmdArgs); (
    rdfs_computable_prolog_call(ComputableProperty, Command, Frame, PrologValue, CmdArgs),
    rdfs_prolog_to_rdf(Property, PrologValue, Value))).

rdfs_computable_prolog_call(_, Command, Frame, Value, AdditionalArgs) :-
  append([Frame, Value], AdditionalArgs, Args),
  ( Command=Module:Predicate
  -> (
    Goal=..[Predicate|Args],
    call(Module:Goal)
  ) ; (
    Goal=..[Command|Args],
    call(Goal)
  )).

% Helpers for caching...
rdfs_computable_cachable(Property) :-
  rdf_has(CP, computable:target, Property),
  rdf_has(CP, computable:cache, literal(type(_, cache))).
rdfs_computable_cache_values(Property, Frame, Value) :- rdf_assert(Property, Frame, Value, cache).
rdfs_computable_cache_frames(Property, Value, Frame) :- rdf_assert(Property, Frame, Value, cache).

%% rdfs_computable_prolog_instance_of(?Instance, ?Class).
%
% Implementation of rdfs_computable_instance_of for PrologClasses.
%
rdfs_computable_prolog_instance_of(Instance, Class) :-
  % get the associated prolog computable
  rdfs_computable_prolog_class(Class, ComputableClass),
  % get the Prolog predicate that is used for evaluation:
  rdf_has(ComputableClass, computable:command, literal(type(_, Cmd))),
  % handle the case that the predicate is defined in another module
  ((term_to_atom(Module:Pred, Cmd)) ->
    (Command=Module:Pred) ;
    (Command=Cmd)),
  (
    nonvar(Instance) ->
    call(Command, Instance, Class);
  (
    call(Command, MyInstance, Class),
    % check if MyInstance is already a global RDF URI
    ((rdf_split_url('', _, MyInstance)) -> (
      rdf_split_url(Namespace, _, Class),
      rdf_split_url(Namespace, MyInstance, Instance),
      rdf_assert(Instance, rdf:type, Class)
    );(
      Instance=MyInstance,
      rdf_assert(Instance, rdf:type, Class)
    ))
     )).

% Convert between Prolog and RDF values
rdfs_prolog_to_rdf(_, [], _)       :- fail, !.
rdfs_prolog_to_rdf(P, [X_pl|_], X) :- rdfs_prolog_to_rdf(P, X_pl, X).
rdfs_prolog_to_rdf(P, [_|Xs], X)   :- rdfs_prolog_to_rdf(P, Xs, X).
rdfs_prolog_to_rdf(_, X, X)        :- atom(X), rdf_split_url(_,_,X), !.
rdfs_prolog_to_rdf(_, literal(type(T,V)),
                      literal(type(T,V))) :- !.
rdfs_prolog_to_rdf(P, PrologValue, RDFValue) :-
  rdf_phas(P, rdfs:range, Range),
  rdf_global_id(xsd:_, Range),
  term_to_atom(PrologValue, ValueAtom),
  RDFValue=literal(type(Range, ValueAtom)), !.
rdfs_prolog_to_rdf(_, PrologValue, literal(type('http://www.w3.org/2001/XMLSchema#float', ValueAtom))) :-
  number(PrologValue),
  term_to_atom(PrologValue, ValueAtom), !.
rdfs_prolog_to_rdf(_, V, V).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Property concatenations

%%
% Create a propertyConcatenation from a prolog list
%
rdfs_assert_prop_conc(List, Ressource) :-
  rdfs_assert_prop_conc(List, Ressource, user).
rdfs_assert_prop_conc([], _, _) :- !,
  throw(error(not_enough_arguments_in_proc_conc_list, _)).
rdfs_assert_prop_conc([_], _, _) :- !,
  throw(error(not_enough_arguments_in_proc_conc_list, _)).
rdfs_assert_prop_conc([A,B], List, DB) :- !,
  rdf_bnode(List),
  rdf_assert(List, computable:rest, B, DB),
  rdf_assert(List, computable:first, A, DB),
  rdf_assert(List, rdf:type, computable:'PropertyConcatenation', DB).
rdfs_assert_prop_conc([A|Rest], List, DB) :- !,
  rdfs_assert_prop_conc(Rest, Tail, DB),
  rdf_bnode(List),
  rdf_assert(List, computable:rest, Tail, DB),
  rdf_assert(List, computable:first, A, DB),
  rdf_assert(List, rdf:type, computable:'PropertyConcatenation', DB).

%%
% Compute the value of a propertyConcatenation
%
rdfs_computable_compute_property_concatenation(Parameter, Frame, ParameterValue) :-
  once(rdfs_individual_of(Parameter, computable:'PropertyConcatenation')),
  rdf_has(Parameter, computable:'first', FirstProperty),
  rdf_has(Parameter, computable:'rest', RestProperty),
  rdf_triple(FirstProperty, Frame, Value),
  rdf_triple(RestProperty, Value, ParameterValue).

% TODO: Allow am:PropertyConcatenation and computable:PropertyCombination
%
% Compute a parameter for a function. This can be any type of Parameter.
% e.g. this can be a ComputableProperty or a PropertyConcatenation.
%
% rdfs_computable_compute_parameter(Parameter, Frame, ParameterValue) :-
%  rdf_triple(Parameter, Frame, ParameterValue).
