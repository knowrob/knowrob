/** <module> Utilities for handling OWL information in KnowRob.

  Copyright (C) 2011 Moritz Tenorth, 2016 Daniel Beßler
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

@author Moritz Tenorth, Daniel Beßler
@license BSD

*/

:- module(knowrob_owl,
    [
      entity/2,
      entity_has/3,
      entity_type/2,
      entity_compute/2,
      entity_assert/2,
      class_properties/3,
      class_properties_some/3,
      class_properties_all/3,
      class_properties_value/3,
      class_properties_nosup/3,
      class_properties_transitive_nosup/3,
      create_restr/6,
      rdf_instance_from_class/2,
      rdf_instance_from_class/3,
      get_timepoint/1,
      get_timepoint/2,
      create_timepoint/2,
      create_interval/2,
      inspect/3
    ]).

:- use_module(library('crypt')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl')).

% define meta-predicates and allow the definitions
% to be in different source files
:- meta_predicate entity_type(0, ?, ?),
                  entity_compute(0, ?, ?).
:- multifile entity_type/2,
             entity_compute/2.

:- rdf_meta entity(r,?),
            entity_property(+,?,?),
            entity_type(r,?),
            entity_compute(r,?),
            entity_assert(r,?),
            entity_iri(r,r),
            class_properties(r,r,t),
            class_properties_some(r,r,t),
            class_properties_all(r,r,t),
            class_properties_value(r,r,t),
            class_properties_nosup(r,r,r),
            class_properties_transitive_nosup(r,r,r),
            class_properties_transitive_nosup_1(r,r,r),
            rdf_instance_from_class(r,r),
            rdf_instance_from_class(r,r,r),
            create_timepoint(+,r),
            get_timepoint(r),
            get_timepoint(+,r),
            create_restr(r, r, r, r, +, r),
            inspect(r,r,r).

:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs,   'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% experimental: inspect() predicate for reading information about things


%% inspect(?Thing, ?P, ?O).
%
% Read information stored for Thing. Basically a wrapper around owl_has and
% class_properties that aggregates their results. The normal use case is to
% have Thing bound and ask for all property/object pairs for this class or
% individual.
%
% @param Thing  RDF identifier, either an OWL class or an individual
% @param P      OWL property identifier 
% @param O      OWL class, individual or literal value specified as property P of Thing
% 
inspect(Thing, P, O) :-
  (rdf_has(Thing, rdf:type, owl:'namedIndividual');
   rdf_has(Thing, rdf:type, owl:'NamedIndividual')),
  owl_has(Thing, P, Olit),
  strip_literal_type(Olit, O).
  
inspect(Thing, P, O) :-
  rdf_has(Thing, rdf:type, owl:'Class'),
  class_properties(Thing, P, Olit),
  strip_literal_type(Olit, O).
  


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% rdf_instance_from_class(+Class, -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers
%
% @param Class   Class describing the type of the instance
% @param Inst    Identifier of the generated instance of Class
rdf_instance_from_class(Class, Instance) :-
    rdf_instance_from_class(Class, _, Instance).



%% rdf_instance_from_class(+Class, +SourceRef, -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers using
% the source reference SourceRef for rdf_assert
%
% @param Class     Class describing the type of the instance
% @param SourceRef Atom as source reference for rdf_assert
% @param Inst      Identifier of the generated instance of Class

rdf_instance_from_class(Class, SourceRef, Instance) :-

  % create instance from type
  ((concat_atom(List, '#', Class),length(List,Length),Length>1) -> (
    % Class is already a URI
    T=Class
  );(
    atom_concat('http://knowrob.org/kb/knowrob.owl#', Class, T)
  )),

  rdf_unique_id(Class, Instance),

  ( ( nonvar(SourceRef), rdf_assert(Instance, rdf:type, T, SourceRef),!);
    ( rdf_assert(Instance, rdf:type, T)) ).


rdf_unique_id(Class, UniqID) :-

  append("$1$", _, Seed),
  crypt(Class, Seed),
  format(atom(Hash), '~s~n', [Seed]),
  sub_atom(Hash, 3, 8, _, Sub),

  atom_concat(Class,  '_', Class2),
  atom_concat(Class2, Sub, Instance),

  % check if there is no triple with this identifier as subject or object yet
  ((rdf(Instance,_,_);rdf(_,_,Instance)) ->
    (rdf_unique_id(Class, UniqID));
    (UniqID = Instance)).



%% create_restr(+Class, +Prop, +Value, +RestrType, +SourceRef, -Restr) is det.
%
% Create a restriction for property Prop and value Value on class Class
% with the sourceRef SourceRef
%
% @param Class     Class that is to be annotated with the restriction
% @param Prop      Property to be used for the restriction
% @param Value     Value to be used for the restriction
% @param RestrType Restriction type as OWL identifier, e.g. owl:someValuesFrom or owl:hasValue
% @param SourceRef Atom as source reference for rdf_assert
%
create_restr(Class, Prop, Value, RestrType, SourceRef, Restr) :-

  \+ (class_properties(Class, Prop, Value)),

  rdf_node(Restr),
%   rdf_assert(Restr, rdf:'type', owl:'Restriction', SourceRef),

  rdf_assert(Class, rdfs:'subClassOf', Restr, SourceRef),

%   assert(blanknode(Prop, someValuesFrom(Value), used)),

  rdf_assert(Restr, owl:'onProperty', Prop, SourceRef),
  rdf_assert(Restr, RestrType, Value, SourceRef).


%% create_timepoint(+TimeStamp, -TimePoint) is det.
%
% Create a timepoint-identifier for the given time stamp
%
% @param T Time stamp as number (seconds since 1970)
% @param T TimePoint instance identifying the given time stamp
%
create_timepoint(TimeStamp, TimePoint) :-
  atom_concat('http://knowrob.org/kb/knowrob.owl#timepoint_', TimeStamp, TimePoint),
  rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint').

%% create_interval(+Start, -TimeInterval) is nondet.
%% create_interval(+Start, +End, -TimeInterval) is det.
%
% Create a interval-identifier for the given start and end time stamps
%
%

create_interval([Start], TimeInterval) :-
  atom(Start), time_term(Start, Start_),
  create_interval([Start_], TimeInterval), !.

create_interval([Start], TimeInterval) :-
  create_timepoint(Start, StartI),
  rdf_instance_from_class(knowrob:'TimeInterval', TimeInterval),
  rdf_assert(TimeInterval, rdf:type, knowrob:'TimeInterval'),
  rdf_assert(TimeInterval, knowrob:'startTime', StartI).

create_interval([Start, End], TimeInterval) :-
  atom(Start), time_term(Start, Start_),
  create_interval([Start_, End], TimeInterval), !.

create_interval([Start, End], TimeInterval) :-
  atom(End), time_term(End, End_),
  create_interval([Start, End_], TimeInterval), !.

create_interval([Start, End], TimeInterval) :-
  create_timepoint(Start, StartI),
  create_timepoint(End, EndI),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#TimeInterval',Start,End], '_', TimeInterval),
  rdf_assert(TimeInterval, rdf:type, knowrob:'TimeInterval'),
  rdf_assert(TimeInterval, knowrob:'startTime', StartI),
  rdf_assert(TimeInterval, knowrob:'endTime', EndI).

%% get_timepoint(-T) is det.
%
% Create a timepoint-identifier for the current time
%
% @param T TimePoint instance identifying the current time stamp
%
get_timepoint(T) :-
  set_prolog_flag(float_format, '%.12g'),
  get_time(Ts),
  create_timepoint(Ts, T).



%% get_timepoint(+Diff, -T) is det.
%
% Create a timepoint-identifier for the current time +/- Diff
%
% @param Diff Time difference to the current time
% @param T    TimePoint instance identifying the current time stamp
%
get_timepoint(Diff, Time) :-

  get_time(Ts),

  ((atom_concat('+', Dunit, Diff), atom_concat(DiffSeconds, 's', Dunit),term_to_atom(A, DiffSeconds)) -> (T is Ts + A) ;
   (atom_concat('+', Dunit, Diff), atom_concat(DiffMinutes, 'm', Dunit),term_to_atom(A, DiffMinutes)) -> (T is Ts + 60.0 * A) ;
   (atom_concat('+', Dunit, Diff), atom_concat(DiffHours,   'h', Dunit),term_to_atom(A, DiffHours))   -> (T is Ts + 3600.0 * A) ;

   (atom_concat('-', Dunit, Diff), atom_concat(DiffSeconds, 's', Dunit),term_to_atom(A, DiffSeconds)) -> (T is Ts - A) ;
   (atom_concat('-', Dunit, Diff), atom_concat(DiffMinutes, 'm', Dunit),term_to_atom(A, DiffMinutes)) -> (T is Ts - 60.0 * A) ;
   (atom_concat('-', Dunit, Diff), atom_concat(DiffHours,   'h', Dunit),term_to_atom(A, DiffHours))   -> (T is Ts - 3600.0 * A) ),


  atom_concat('http://knowrob.org/kb/knowrob.owl#timepoint_', T, Time),
  rdf_assert(Time, rdf:type, knowrob:'TimePoint').





%% class_properties(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom- and hasValue-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties(Class, Prop, Val) :-
  (class_properties_some(Class, Prop, Val);
   class_properties_value(Class, Prop, Val)).




%% class_properties_some(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_some(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_some(Class, Prop, Val).

class_properties_some(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_some(Super, Prop, Val).


class_properties_1_some(Class, Prop, Val) :-       % read all values for some_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, some_values_from(Val))).




%% class_properties_all(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of allValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_all(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_all(Class, Prop, Val).

class_properties_all(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_all(Super, Prop, Val).


class_properties_1_all(Class, Prop, Val) :-       % read all values for all_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, all_values_from(Val))) .




%% class_properties_value(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of hasValue-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_value(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_value(Class, Prop, Val).

class_properties_value(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_value(Super, Prop, Val).


class_properties_1_value(Class, Prop, Val) :-       % read all values for has_value restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, has_value(Val))) .


%% class_properties_nosup(?Class, ?Prop, ?Val) is nondet.
%
% Version of class_properties without considering super classes
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_nosup(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_nosup_1(Class, Prop, Val).

% class_properties_nosup(Class, Prop, Val) :-         % do not consider properties of superclasses
%   owl_subclass_of(Class, Super), Class\=Super,
%   class_properties_nosup_1(Super, Prop, Val).

class_properties_nosup_1(Class, Prop, Val) :-
  owl_direct_subclass_of(Class, Sup),
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  ( owl_restriction(Sup,restriction(SubProp, some_values_from(Val))) ;
    owl_restriction(Sup,restriction(SubProp, has_value(Val))) ).

%% class_properties_transitive_nosup(?Class, ?Prop, ?Val) is nondet.
%
% Transitive cersion of class_properties without considering super classes
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_transitive_nosup(Class, Prop, SubComp) :-
    class_properties_nosup(Class, Prop, SubComp).
class_properties_transitive_nosup(Class, Prop, SubComp) :-
    class_properties_nosup(Class, Prop, Sub),
    owl_individual_of(Prop, owl:'TransitiveProperty'),
    Sub \= Class,
    class_properties_transitive_nosup(Sub, Prop, SubComp).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% methods for querying OWL entities
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TASKS
%%%
%%%%%%%%%%%%% timepoint/interval entities
%%% [a, timepoint, [value, ?time]]
%%% 1. special handling of timepoint value (since it is not a property!)
%%% [an, interval, [start_time, [a, timepoint, [value, ?time]]]]
%%% 2. interval handling should just work
%%%
%%%
%%%%%%%%%%%%% query entities using computables
%%% [an, event, [comp_temporal:beforeI, [a, timepoint, [value, ?time]]]]
%%% [an, event, [comp_temporal:beforeI, [an, event, ..]]]
%%% 1. Should just work
%%%
%%%
%%%%%%%%%%%%% compute entites
%%% 1. Write multifile `entity_compute/2` predicate
%%%
%%% 1.1. Compute `Timepoint`/`TimeInterval` individuals
%%% 1.1.1 Interpret `value` key
%%% 1.1.2 Build name
%%% 1.1.3 Check if existing
%%% 1.1.4 Call `create_timepoint` if not
%%%
%%% 1.2. Compute mongo individuals
%%% 1.2.1. description to mongo query
%%% 1.2.1.1. map properties to mongo keys
%%% 1.2.1.2. map computables to prolog operators where possible (e.g., ?t0 `beforeI` ?t1)
%%% 1.2.2. iterate results
%%% 1.2.3. create mongo result description
%%%
%%% 1.2.4. handle objects (logged_designators)
%%% 1.2.4.1. individual selection by identity resolution (evolving designators, action logs, spatial reasoning?, similarity measure?)
%%% 1.2.4.2. if no existing individual found, assert new SpatialThing
%%% 1.2.4.3. create property fluents based on designator values (including rdf:type)
%%% 1.2.4.4. attach designator?
%%%
%%% 1.2.5. handle locations (logged_designators)
%%% TODO how are these (ideally) represented in designators
%%%
%%% 1.2.6. handle poses (tf)
%%% 1.2.6.1. call `entity_assert`
%%%
%%% 1.2.7. handle trajectories (tf)
%%% TODO this requires some more thoughts....
%%%
%%%
%%% 2. Call `entity_compute` in `entity` iff var(Entity)
%%%
%%%
%%% TODO Generic implementation using computable classes?
%%%    * How to limit computation?
%%%

%% entity(?Entity, +Descr) is nondet.
%
% Query for entity matching an entity description or build
% description for entity.
%
% @param Entity IRI of matching entity
% @param Descr An entity description, OWL individual or class IRI
%
entity(Entity, EntityClass) :-
  atom(EntityClass),
  rdfs_individual_of(Entity, EntityClass), !.

entity(Entity, Descr) :-
  var(Descr), !,
  rdfs_individual_of(Entity,owl:'Thing'),
  entity_head(Entity, [A,TypeBase], TypeIri),
  entity_generate(Entity, [A,TypeBase], TypeIri, Descr).

entity(Entity, Descr) :-
  entity_(Entity, Descr),
  % make sure it's an individual and not a class
  rdfs_individual_of(Entity, owl:'Thing').

entity_(Entity, [a, timepoint, Time]) :-
  number(Time), create_timepoint(Time, Entity), !.
entity_(Entity, [a, timepoint, [value,TimeValue]]) :-
  nonvar(TimeValue), create_timepoint(TimeValue, Entity),!.
entity_(Entity, [a, timepoint, [name,Name]]) :-
  nonvar(Name),
  rdf_global_term(Name, Entity),
  time_term(Entity, TimeValue),
  create_timepoint(TimeValue, Entity),!.

entity_(Entity, [an, interval, [Begin, End]]) :-
  create_interval([Begin,End],Entity), !.
entity_(Entity, [an, interval, [Begin]]) :-
  create_interval([Begin],Entity),
  interval(Entity, [Begin]), !.

entity_(Entity, [A|[Type|Descr]]) :-
  nonvar(A),nonvar(Type),
  entity_head(Entity, [A,Type], _),
  entity_(Entity, Descr) ; (
    var(Entity),
    entity_compute(Entity, [A|[Type|Descr]])
  ).

entity_(Entity, [[type,Type]|Descr]) :-
  nonvar(Type),
  entity_iri(TypeIri, Type, camelcase),
  ( rdfs_individual_of(Entity, TypeIri);
    Entity = TypeIri ),
  entity_(Entity, Descr).

entity_(Entity, [[name,Name]|Descr]) :-
  nonvar(Name),
  entity_iri(NameIri, Name, camelcase),
  Entity = NameIri,
  entity_(Entity, Descr).

%% Key-value property
entity_(Entity, [[Key,Value]|Descr]) :-
  nonvar(Key), nonvar(Value),
  entity_properties([[PropIri,PropValue]], [[Key,Value]]),
  holds(Entity,PropIri,PropValue),
  entity_(Entity, Descr).

%% Fluid properties that match temporal relation.
entity_(Entity, [[Key,Value,TemporalRelation,IntervalDescr]|Descr]) :-
  nonvar(Key), nonvar(Value),
  entity_properties([[PropIri,PropValue]], [[Key,Value]]),
  interval(IntervalDescr, Interval),
  holds(Entity,PropIri,PropValue,FluentInterval),
  interval_operator(TemporalRelation,FluentInterval,Interval),
  entity_(Entity, Descr).

entity_(_, []).

%% Read entity property value
entity_has(S,P,O) :-
  once((
    entity_has_(S,P,O) ; (
      atom(S),
      entity(S,Descr),
      entity_has_(Descr,P,O)
    )
  )).
entity_has_([], _, _) :- false.
entity_has_([[Key|[Val|_]]|_], Key, Val) :- !.
entity_has_([_|Descr], Key, Val) :- entity_has_(Descr,Key,Val).


%% entity_type(?Descr, ?Iri) is det.
%
% Maps ectity type description to IRI
%
% @param Descr The type description "[a,an] ?type_name"
% @param Iri The corresponding type iri
%
entity_type([a,timepoint],  'http://knowrob.org/kb/knowrob.owl#TimePoint').
entity_type([an,interval],  'http://knowrob.org/kb/knowrob.owl#TimeInterval').
entity_type([an,action],    'http://knowrob.org/kb/knowrob.owl#Action').
entity_type([an,event],     'http://knowrob.org/kb/knowrob.owl#Event').
entity_type([an,object],    'http://knowrob.org/kb/knowrob.owl#SpatialThing').
entity_type([a,location],   'http://knowrob.org/kb/knowrob.owl#Location').
entity_type([a,pose],       'http://knowrob.org/kb/knowrob.owl#Pose').
entity_type([a,trajectory], 'http://knowrob.org/kb/knowrob.owl#Trajectory').


%% entity_compute(?Entity, ?Descr) is nondet.
%
% Compute entities matching given description.
% This is actually supposed to modify the RDF triple store.
%
% @param Entity IRI of matching entity
% @param Descr An entity description
%
entity_compute(Entity, [a,timepoint,Time]) :-
  create_timepoint(Time, Entity), !.
entity_compute(Entity, [a|[timepoint|Descr]]) :-
  entity_has(Descr, value, Time),
  create_timepoint(Time, Entity). % FIXME: redundant results

entity_compute(Entity, [an|[interval|Descr]]) :-
  entity_has(Descr, start_time, StartDescr),
  entity_has(StartDescr, value, Start),
  
  entity_has(Descr, end_time, EndDescr),
  entity_has(EndDescr, value, End),
  
  create_interval([Start,End],Entity). % FIXME: redundant results

%%%%%% knowrob_mongo
%entity_compute(Entity, [an|[object|Tail]]) :- false. % TODO
%entity_compute(Entity, [a|[location|Tail]]) :- false. % TODO
%entity_compute(Entity, [a|[pose|Tail]]) :- false. % TODO
%entity_compute(Entity, [a|[trajectory|Tail]]) :- false. % TODO


%% entity_assert(-Entity, +Descr) is nondet.
%
% Assert entity description in RDF triple store as new individual.
%
% @param Entity IRI of matching entity
% @param Descr An entity description
%
entity_assert(Entity, [a, timepoint, Time]) :-
  number(Time), create_timepoint(Time, Entity), !.
entity_assert(Entity, [a, timepoint, [value, Time]]) :-
  create_timepoint(Time, Entity), !.

entity_assert(Entity, [an, interval, [Begin, End]]) :-
  create_interval([Begin, End], Entity), !.
entity_assert(Entity, [an, interval, [Begin]]) :-
  create_interval([Begin], Entity), !.
entity_assert(Entity, [an|[interval|Descr]]) :-
  entity_has(Descr, start_time, BeginDescr),
  entity(Begin, BeginDescr),
  (( entity_has(Descr, end_time, EndDescr),
     entity(End, EndDescr),
     create_interval([Begin, End], Entity) );
     create_interval([Begin], Entity) ), !.

entity_assert(Entity, [A|[Type|Descr]]) :-
  nonvar(Type),
  entity_type([A,Type], TypeIri_),
  (( entity_has(Descr,type,AType),
     entity_iri(AType, ATypeIri),
     rdf_reachable(ATypeIri, rdfs:subClassOf, TypeIri_) )
  -> TypeIri = ATypeIri
  ;  TypeIri = TypeIri_ ),
  (( entity_has(Descr, name, Entity_),
     rdf_global_term(Entity_, Entity), % TODO: fallback to knowrob prefix
     rdf_assert(Entity, rdf:type, TypeIri) );
     rdf_instance_from_class(TypeIri, Entity) ),
  entity_assert(Entity, Descr), !.

entity_assert(Entity, [[name,_]|Descr]) :- entity_assert(Entity, Descr), !.

entity_assert(Entity, [[type,TypeDescr]|Descr]) :-
  nonvar(Entity), nonvar(TypeDescr),
  entity_iri(TypeIri, TypeDescr, camelcase),
  rdf_assert(Entity, rdf:type, TypeIri),
  entity_assert(Entity, Descr), !.

entity_assert(Entity, [[Key,Value]|Descr]) :-
  nonvar(Entity), nonvar(Key), nonvar(Value),
  entity_iri(PropIri, Key, lower_camelcase),
  (  rdf_has(PropIri, rdf:type, owl:'ObjectProperty')
  ->  ( % nested entity
      entity_assert(ValueEntity, Value),
      rdf_assert(Entity, PropIri, ValueEntity)
  ) ; ( % data property
      rdf_has(PropIri, rdfs:range, Range),
      rdf_assert(Entity, PropIri, literal(type(Range,Value)))
  )),
  entity_assert(Entity, Descr).

entity_assert(Entity, [[Key,Value,during,IntervalDescr]|Descr]) :-
  nonvar(Entity), nonvar(Key), nonvar(Value),
  entity_iri(PropIri, Key, lower_camelcase),
  entity(Interval, IntervalDescr),
  (  rdf_has(PropIri, rdf:type, owl:'ObjectProperty')
  ->  ( % nested entity
      entity_assert(ValueEntity, Value),
      fluent_assert(Entity,PropIri,ValueEntity,Interval)
  ) ; ( % data property
      rdf_has(PropIri, rdfs:range, Range),
      fluent_assert(Entity,PropIri,literal(type(Range,Value)),Interval)
  )),
  entity_assert(Entity, Descr).

entity_assert(_, []).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% helper


interval_operator(during, I1, I2) :-   interval_during(I1,I2).
interval_operator(before, I1, I2) :-   interval_before(I1,I2).
interval_operator(after, I1, I2) :-    interval_after(I1,I2).
interval_operator(meets, I1, I2) :-    interval_meets(I1,I2).
interval_operator(starts, I1, I2) :-   interval_starts(I1,I2).
interval_operator(finishes, I1, I2) :- interval_finishes(I1,I2).
interval_operator(overlaps, I1, I2) :- interval_overlaps(I1,I2).


entity_properties([['http://www.w3.org/1999/02/22-rdf-syntax-ns#type',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#comment',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#subClassOf',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[inverse_of('http://knowrob.org/kb/knowrob.owl#temporalPartOf'),Fluent]|Tail],
                            [[Key,Value,during,IntervalDescr]|DescrTail]) :-
  fluent_has(Fluent, PropIri, PropValue, IntervalIri),
  entity_properties([[PropIri,PropValue]], [[Key,Value]]),
  entity(IntervalIri, IntervalDescr),
  entity_properties(Tail, DescrTail), !.

entity_properties([[inverse_of(_),_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,TimeIri]|Tail],
                  [[Key, [a,timepoint,Time]]|DescrTail]) :-
  rdfs_individual_of(TimeIri, knowrob:'TimePoint'),
  entity_iri(PropIri, Key, lower_camelcase),
  time_term(TimeIri,Time),
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,IntervalIri]|Tail],
                  [[Key, [an,interval,Interval]]|DescrTail]) :-
  interval(IntervalIri,Interval),
  entity_iri(PropIri, Key, lower_camelcase),
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,PropValue]|Tail], [[Key,Value]|DescrTail]) :-
  entity_iri(PropIri, Key, lower_camelcase),
  % match rdf value with description value
  (  rdf_has(PropIri, rdf:type, owl:'ObjectProperty')
  -> entity(PropValue, Value)
  ;  property_value(PropValue, Value)
  ),
  entity_properties(Tail, DescrTail).

entity_properties([], []).


% TODO: howto handle mongo entities here?
entity_generate(Entity, [a,timepoint], _, [a,timepoint,Time]) :-
  rdfs_individual_of(Entity, knowrob:'TimePoint'), time_term(Entity,Time), !.
entity_generate(Entity, [an,interval], _, [an,interval,Interval]) :-
  interval(Entity,Interval), !.
entity_generate(Entity, [A,TypeBase], TypeBaseIri, [A|[TypeBase|[[type,TypeName]|PropDescr]]]) :-
  (( rdf_has(Entity, rdf:type, Type),
     Type \= TypeBaseIri,
     rdf_reachable(Type, rdfs:subClassOf, TypeBaseIri) )
  -> TypeIri = Type
  ;  TypeIri = TypeBaseIri ),
  entity_iri(TypeIri, TypeName, camelcase),
  findall([PropIri,PropValue], rdf_has(Entity, PropIri, PropValue), Props),
  entity_properties(Props, PropDescr), !.


%% Match [a|an, ?entity_type]
entity_head(Entity, Type, TypeIri) :-
  entity_head_(Entity, Type, TypeIri),
  forall( entity_head_(Entity, _, TypeIri_), ( % TODO: do this faster
    ( TypeIri_ = TypeIri ) ; not(rdf_reachable(TypeIri_, rdfs:subClassOf, TypeIri))
  )), !.
entity_head_(Entity, [an,Type], TypeIri) :-
  entity_type([an,Type], TypeIri),
  entity_iri(Entity, TypeIri).
entity_head_(Entity, [a,Type], TypeIri) :-
  entity_type([a,Type], TypeIri),
  entity_iri(Entity, TypeIri).
entity_head_(_, [a,thing], 'http://www.w3.org/2002/07/owl#Thing').


entity_type(Entity, TypeBase, Entity) :-
  atom(Entity), rdf_reachable(Entity, rdfs:subClassOf, TypeBase), !.
entity_type(Entity, TypeBase, Type) :-
  rdf_has(Entity, rdf:type, Type),
  rdf_reachable(Type, rdfs:subClassOf, TypeBase).


%% Read entity namespace
entity_ns(Entity, NamespaceUri, EntityName) :-
  (  Entity =.. [':', NS, EntityName]
  ->  (
      rdf_current_ns(NS, NamespaceUri)
  ) ; ( % fallback to knowrob namespace
      EntityName = Entity,
      rdf_current_ns(knowrob, NamespaceUri)
  )), !.
entity_ns(Entity, NamespaceUri, EntityName) :-
  rdf_split_url(NamespaceUri, EntityName, Entity).


%% Converts between IRI representation and description representation
entity_iri(Iri, Descr, Formatter) :-
  var(Descr),
  rdf_split_url(NamespaceUri, Name, Iri),
  rdf_current_ns(Namespace, NamespaceUri),
  call(Formatter, NameUnderscore, Name),
  (  Namespace=knowrob
  -> Descr=NameUnderscore
  ;  Descr=Namespace:NameUnderscore
  ).
entity_iri(Iri, Description, Formatter) :-
  var(Iri),
  entity_ns(Description, NS, NameUnderscore),
  call(Formatter, NameUnderscore, Name),
  atom_concat(NS, Name, Iri).

entity_iri(Entity, Type) :-
  rdf_has(Entity, rdf:type, Type),
  rdf_split_url(Url, _, Type),
  rdf_current_ns(NS, Url),
  NS \= owl.
entity_iri(Entity, Type) :-
  rdfs_individual_of(Entity,Type).
