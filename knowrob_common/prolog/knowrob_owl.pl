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
      entity_iri/3,
      entity_write/1,
      entity_format/2,
      class_properties/3,
      class_properties_some/3,
      class_properties_all/3,
      class_properties_value/3,
      class_properties_nosup/3,
      class_properties_transitive_nosup/3,
      create_restr/6,
      rdf_instance_from_class/2,
      rdf_instance_from_class/3,
      rdf_phas/3,
      rdf_atom_no_ns/2,
      get_timepoint/1,
      get_timepoint/2,
      create_timepoint/2,
      create_interval/2,
      create_pose/2,
      create_tf_frame/2,
      inspect/3,
      rotmat_to_list/2,
      position_to_list/2,
      quaternion_to_list/2
    ]).

:- use_module(library('crypt')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl')).
:- use_module(library('knowrob_math')).

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
            rdf_atom_no_ns(r,?),
            create_timepoint(+,r),
            create_poset(+,r),
            get_timepoint(r),
            get_timepoint(+,r),
            create_restr(r, r, r, r, +, r),
            inspect(r,r,r),
            rotmat_to_list(r,-),
            position_to_list(r,-),
            quaternion_to_list(r,-),
            rdf_phas(r,r,o).

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


rdf_phas(Property, P, O) :-
        rdfs_subproperty_of(Property, Super),
        rdf_has(Super, P, O2), !,
        O = O2.

rdf_atom_no_ns(Iri, Name) :- rdf_split_url(_, Name, Iri).


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


% TODO: creat_* functions into knowrob_owl_factory module
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
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#TimeInterval',Start], '_', TimeInterval),
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

%% create_pose(+Term, -Iri) is nondet.
%
%
create_pose(pose([X,Y,Z], [QW,QX,QY,QZ]), Pose) :-
  create_pose(pose('http://knowrob.org/kb/knowrob.owl#MapFrame', [X,Y,Z], [QW,QX,QY,QZ]), Pose), !.

create_pose(pose(ReferenceObj, [X,Y,Z], [QW,QX,QY,QZ]), Pose) :-
  rdfs_individual_of(ReferenceObj, knowrob:'SpatialThing-Localized'),
  rdf_split_url(_, Ref, ReferenceObj),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#Pose'|[Ref,X,Y,Z,QW,QX,QY,QZ]], '_', Pose),
  atomic_list_concat([X,Y,Z], ' ', Translation),
  atomic_list_concat([QW,QX,QY,QZ], ' ', Quaternion),
  rdf_assert(Pose, rdf:type, knowrob:'Pose'),
  rdf_assert(Pose, knowrob:'translation', literal(type(string,Translation))),
  rdf_assert(Pose, knowrob:'quaternion', literal(type(string,Quaternion))),
  rdf_assert(Pose, knowrob:'relativeTo', ReferenceObj).

create_pose(mat(Data), Pose) :-
  matrix_translation(Data, Translation),
  matrix_rotation(Data, Rotation),
  create_pose(pose(Translation, Rotation), Pose).


create_tf_frame(URDFName, Frame) :-
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#FrameOfReference'|[URDFName]], '_', Frame),
  rdf_assert(Frame, rdf:type, knowrob:'FrameOfReference'),
  rdf_assert(Frame, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(type(xsd:string, URDFName))).


%% create_location(+Axioms, -Location) is nondet.
%
% Create a location-identifier for the given axioms
%
%
create_location(Axioms, Location) :-
  location_name(Axioms, Location),
  rdf_assert(Location, rdf:type, knowrob:'SpaceRegion'),
  forall( member([P,O], Axioms), rdf_assert(Location, P, O) ).

location_name(Axioms, Location) :-
  is_list(Axioms),
  location_name_args_(Axioms,Args),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#SpaceRegion'|Args], '_', Location).

location_name_args_([[P,O]|Axioms], [P_name|[O_name|Args]]) :-
  rdf_split_url(_, P_name, P), % FIXME: don't ignore namespace
  rdf_split_url(_, O_name, O),
  location_name_args_(Axioms, Args).
location_name_args_([], []).


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
  rdfs_individual_of(Class, owl:'Class'), % make sure Class is bound before calling owl_subclass_of
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





%% rotmat_to_list(+RotMatInstance, -PoseList) is nondet.
%
% Read the pose values for an instance of a rotation matrix
%
% @param Obj       Instance of a subclass of SpatialThing-Localized
% @param PoseList  Row-based representation of the object's 4x4 pose matrix as list[16]
% 
rotmat_to_list([M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33],
               [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :- !.

rotmat_to_list(Pose, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

    rdf_triple('http://knowrob.org/kb/knowrob.owl#m00',Pose,M00literal), strip_literal_type(M00literal, M00a), term_to_atom(M00, M00a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m01',Pose,M01literal), strip_literal_type(M01literal, M01a), term_to_atom(M01, M01a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m02',Pose,M02literal), strip_literal_type(M02literal, M02a), term_to_atom(M02, M02a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m03',Pose,M03literal), strip_literal_type(M03literal, M03a), term_to_atom(M03, M03a),

    rdf_triple('http://knowrob.org/kb/knowrob.owl#m10',Pose,M10literal), strip_literal_type(M10literal, M10a), term_to_atom(M10, M10a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m11',Pose,M11literal), strip_literal_type(M11literal, M11a), term_to_atom(M11, M11a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m12',Pose,M12literal), strip_literal_type(M12literal, M12a), term_to_atom(M12, M12a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m13',Pose,M13literal), strip_literal_type(M13literal, M13a), term_to_atom(M13, M13a),

    rdf_triple('http://knowrob.org/kb/knowrob.owl#m20',Pose,M20literal), strip_literal_type(M20literal, M20a), term_to_atom(M20, M20a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m21',Pose,M21literal), strip_literal_type(M21literal, M21a), term_to_atom(M21, M21a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m22',Pose,M22literal), strip_literal_type(M22literal, M22a), term_to_atom(M22, M22a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m23',Pose,M23literal), strip_literal_type(M23literal, M23a), term_to_atom(M23, M23a),

    rdf_triple('http://knowrob.org/kb/knowrob.owl#m30',Pose,M30literal), strip_literal_type(M30literal, M30a), term_to_atom(M30, M30a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m31',Pose,M31literal), strip_literal_type(M31literal, M31a), term_to_atom(M31, M31a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m32',Pose,M32literal), strip_literal_type(M32literal, M32a), term_to_atom(M32, M32a),
    rdf_triple('http://knowrob.org/kb/knowrob.owl#m33',Pose,M33literal), strip_literal_type(M33literal, M33a), term_to_atom(M33, M33a),!.

%% position_to_list(+Pose, -PositionList) is nondet.
%
% Read the translation values for an instance of a transformation
%
% @param Pose          Instance of a subclass of Transformation
% @param PositionList  list[3] that represents translation of an object
% 
position_to_list(Pose, [X,Y,Z]) :-
  rdf_triple('http://knowrob.org/kb/knowrob.owl#translation', Pose, literal(type(_,Translation))),
  parse_vector(Translation, [X,Y,Z]).

%% quaternion_to_list(+Pose, -QuaternionList) is nondet.
%
% Read the rotation values for an instance of a transformation
%
% @param Pose          Instance of a subclass of Transformation
% @param PositionList  list[4] that represents rotation of an object. First list element is the w component of the quaternion.
% 
quaternion_to_list(Pose, [QW,QX,QY,QZ]) :-
  rdf_triple('http://knowrob.org/kb/knowrob.owl#quaternion', Pose, literal(type(_,Quaternion))),
  parse_vector(Quaternion, [QW,QX,QY,QZ]).


comp_designatedThing(Designator, Obj) :-
  rdf_has(TemporalParts, knowrob:designator, Designator),
  rdf_has(Obj, knowrob:temporalParts, TemporalParts).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% methods for querying OWL entities

%% concept(?Concept, +Descr) is nondet.
%
% Query for concept matching an concept description or build
% description for concept.
%
% @param Concept IRI of matching entity
% @param Descr An concept description or OWL class IRI
%
% TODO: implement
concept(_, _) :- fail.

%% entity(?Entity, +Descr) is nondet.
%
% Query for entity matching an entity description or build
% description for entity.
%
% @param Entity IRI of matching entity
% @param Descr An entity description, OWL individual or class IRI
%
% TODO: support for trajectory entities
%    - [a, trajectory, [urdf, 'base_link'], [temporal_extend, [an, interval, [40.0,80.0]]]]
entity(Entity, EntityClass) :-
  atom(EntityClass),
  owl_individual_of(Entity, EntityClass), !.

entity(Entity, Descr) :-
  var(Descr), !,
  rdfs_individual_of(Entity,owl:'Thing'),
  once(entity_head(Entity, [A,TypeBase], _, TypeIri)),
  entity_generate(Entity, [A,TypeBase], TypeIri, Descr).

entity(Entity, Descr) :-
  entity_(Entity, Descr),
  % make sure it's an individual and not a class
  once(owl_individual_of(Entity, owl:'Thing')),
  \+ rdfs_individual_of(Entity, knowrob:'TemporalPart').

%% Time point entities
entity_(Entity, [a, timepoint, Time]) :-
  number(Time), create_timepoint(Time, Entity), !.
entity_(Entity, [a, timepoint, [value,TimeValue]]) :-
  nonvar(TimeValue), create_timepoint(TimeValue, Entity),!.
entity_(Entity, [a, timepoint, [name,Name]]) :-
  nonvar(Name),
  rdf_global_term(Name, Entity),
  time_term(Entity, TimeValue),
  create_timepoint(TimeValue, Entity),!.

%% Time interval entities
entity_(Entity, [an, interval, [Begin, End]]) :-
  create_interval([Begin,End],Entity), !.
entity_(Entity, [an, interval, [Begin]]) :-
  create_interval([Begin],Entity),
  interval(Entity, [Begin]), !.
entity_(Entity, [an, interval|Descr]) :-
  entity_has(Descr, start_time, BeginDescr),
  entity(Begin, BeginDescr),
  (( entity_has(Descr, end_time, EndDescr),
     entity(End, EndDescr),
     create_interval([Begin, End], Entity) );
     create_interval([Begin], Entity) ), !.

%% Pose entities
entity_(Entity, [a, pose, [X,Y,Z], [QW,QX,QY,QZ]]) :-
  create_pose(pose([X,Y,Z], [QW,QX,QY,QZ]), Entity), !.
entity_(Entity, [a|[pose|Descr]]) :-
  entity_has(Descr, translation, [X,Y,Z]),
  entity_has(Descr, quaternion, [QW,QX,QY,QZ]),
  (  entity_has(Descr, reference_frame, Frame)
  -> create_pose(pose(Frame, [X,Y,Z], [QW,QX,QY,QZ]), Entity)
  ;  create_pose(pose([X,Y,Z], [QW,QX,QY,QZ]), Entity) ), !.

%% Location entities
entity_(Entity, [a, location|Descr]) :-
  entity_axioms(Descr, 'http://knowrob.org/kb/knowrob.owl#spatiallyRelated', Axioms),
  length(Axioms, L), (L > 0),
  create_location(Axioms, Entity), !.

%% Entity type description `a|an pose|location|...`
entity_(Entity, [a, Type|Descr]) :-
  nonvar(Type), !,
  entity_name(Descr, Entity),
  entity_body(Entity, [a, Type|Descr]).
entity_(Entity, [an, Type|Descr]) :-
  nonvar(Type), !,
  entity_name(Descr, Entity),
  entity_body(Entity, [an, Type|Descr]).

%% name of the OWL individual
entity_(Entity, [[name,EntityName]|Descr]) :-
  entity_name([name,EntityName], Entity),
  entity_(Entity, Descr).

%% ignore type, types are handled in `entity_head`
entity_(Entity, [[type,Type|_]|Descr]) :-
  nonvar(Type), !,
  entity_(Entity, Descr).

%% key-value property
% TODO: support specification of property units
%    - [an, object, [height, 20, qudt:meter, during [0.0,20.0]]]
entity_(Entity, [[Key,Value|ValDescr]|Descr]) :-
  nonvar(Key),
  entity_iri(PropIri, Key, lower_camelcase),
  
  % use temporal axioms to restrict Interval var before calling holds
  (nonvar(ValDescr)
  -> (
    entity_interval_axioms(ValDescr, Axioms),
    entity_intersection_interval(Axioms, RestrictedInterval),
    (RestrictedInterval=[] -> true ; Interval=RestrictedInterval)
  ) ; true),
  
  (nonvar(Value)
  -> (
    rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
    -> (
      strip_literal_type(Value, X),
      holds(Entity,PropIri,PropValue,Interval),
      % ignore literal(type(..)) terms, just extract value
      strip_literal_type(PropValue, X)
    ) ; (
      entity_object_value(PropValue, Value),
      holds(Entity,PropIri,PropValue,Interval)
    )
  ) ; (
    holds(Entity,PropIri,PropValue,Interval), (
    rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
    -> strip_literal_type(PropValue, Value)
    ;  Value = PropValue)
  )),
  
  entity_(Entity, Descr).

entity_(_, []).


%% Nested object description
entity_object_value(Iri, Value) :-
  is_list(Value), !,
  entity(Iri, Value).
%% Object iri
entity_object_value(Iri, Value) :-
  once( var(Iri) ; atom(Iri) ),
  entity_ns(Value, NS, ValueUnderscore),
  atom(NS), atom(ValueUnderscore),
  camelcase(ValueUnderscore, ValueCamel),
  atom_concat(NS, ValueCamel, Iri), !.
%% Object iri
entity_object_value(Iri, Value) :-
  rdf_global_term(Value, Iri), !.


%% Compute entity from external source such as a database
entity_body(Entity, [A, Type|Descr]) :-
  var(Entity),
  % TODO: skip if existing before computing
  % in that case computing still makes sense in order to update
  % with latest designators
  once(entity_compute(Entity, [A, Type|Descr])).
%% Handle description
entity_body(Entity, [A, Type|Descr]) :-
  entity_head(Entity, [A,Type], Descr, _),
  entity_(Entity, Descr),
  % check general type last because it matches many entities
  entity_type([A,Type], TypeIri),
  once(owl_individual_of(Entity, TypeIri)).


%% "name" keys in the description
entity_name(Descr, Entity) :-
  entity_has(Descr,name,Name),
  !, % names must match!
  rdf_global_term(Name, Entity),
  (   rdf_has(Entity, _, _)
  -> true
  ;  entity_iri(Entity, Name, camelcase) ), !.
entity_name(_, _).


%% Read entity description value
entity_has([[Key,Val|_]|_], Key, Val).
entity_has([_|Tail], Key, Val) :- entity_has(Tail, Key, Val).


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
entity_type([an,object],    'http://knowrob.org/kb/knowrob.owl#EnduringThing-Localized').
entity_type([a,location],   'http://knowrob.org/kb/knowrob.owl#SpaceRegion').
entity_type([a,pose],       'http://knowrob.org/kb/knowrob.owl#Pose').
entity_type([a,trajectory], 'http://knowrob.org/kb/knowrob.owl#Trajectory').

entity_type(Entity, TypeBase, Entity) :-
  atom(Entity), rdf_reachable(Entity, rdfs:subClassOf, TypeBase), !.

entity_type(Entity, TypeBase, Type) :-
  rdf_has(Entity, rdf:type, Type),
  rdf_reachable(Type, rdfs:subClassOf, TypeBase).


%% entity_compute(?Entity, ?Descr) is nondet.
%
% Compute entities matching given description.
% This is actually supposed to modify the RDF triple store.
%
% @param Entity IRI of matching entity
% @param Descr An entity description
%
entity_compute(_, [a|[timepoint|_]]) :- fail.
entity_compute(_, [an|[interval|_]]) :- fail.


%% entity_assert(-Entity, +Descr) is nondet.
%
% Assert entity description in RDF triple store as new individual.
%
% @param Entity IRI of matching entity
% @param Descr An entity description
%
entity_assert(Entity, [a,timepoint|Descr]) :- entity(Entity, [a,timepoint|Descr]), !. 
entity_assert(Entity, [an,interval|Descr]) :- entity(Entity, [an,interval|Descr]), !.
entity_assert(Entity, [a,location|Descr]) :-  entity(Entity, [a,location|Descr]), !.
% FIXME: above does not allow to call entity_assert for nested entites

entity_assert(Entity, [a, pose, [X,Y,Z], [QW,QX,QY,QZ]]) :-
  create_pose(pose([X,Y,Z], [QW,QX,QY,QZ]), Entity), !.
entity_assert(Entity, [a, pose|Descr]) :-
  entity_has(Descr, translation, [X,Y,Z]),
  entity_has(Descr, quaternion, [QW,QX,QY,QZ]),
  (  entity_has(Descr, relative_to, RelObjDescr)
  -> (
    entity(RelObj, RelObjDescr),
    create_pose(pose(RelObj, [X,Y,Z], [QW,QX,QY,QZ]), Entity)
  ) ; (
    create_pose(pose([X,Y,Z], [QW,QX,QY,QZ]), Entity) )
  ), !.

entity_assert(Entity, [A,Type|Descr]) :-
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
      entity(ValueEntity, Value), % TODO: support recursive option (call enity_assert instead)
      rdf_assert(Entity, PropIri, ValueEntity)
  ) ; ( % data property
      rdf_has(PropIri, rdf:type, owl:'DatatypeProperty'),
      rdf_phas(PropIri, rdfs:range, Range), % FIXME: what if range unspecified
      rdf_assert(Entity, PropIri, literal(type(Range,Value)))
  )),
  entity_assert(Entity, Descr).

entity_assert(Entity, [[Key,Value,during,IntervalDescr]|Descr]) :-
  nonvar(Entity), nonvar(Key), nonvar(Value),
  entity_iri(PropIri, Key, lower_camelcase),
  entity(Interval, IntervalDescr),
  (  rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
  ->  ( % data property
      rdf_phas(PropIri, rdfs:range, Range), % FIXME: what if range unspecified
      create_fluent(Entity, Fluent, Interval),
      fluent_assert(Fluent, PropIri, literal(type(Range,Value)))
  ) ; ( % nested entity
      rdf_has(PropIri, rdf:type, owl:'ObjectProperty'),
      entity(ValueEntity, Value),
      create_fluent(Entity, Fluent, Interval),
      fluent_assert(Fluent, PropIri, ValueEntity)
  )),
  entity_assert(Entity, Descr).

entity_assert(_, []).


%% entity_format(+Descr, -String) is nondet.
%
% Format entity for pretty printing.
%
% @param Descr An entity description
% @param String Formatted string
%
entity_format(Descr, String) :-
  with_output_to(string(String), entity_write(Descr)).

%% entity_write(+Descr) is nondet.
%
% Write entity to current output stream.
%
% @param Descr An entity description
%
entity_write(Descr) :- entity_write(Descr, '').

entity_write([A,Type|Tail], Spaces) :-
  member(A, [a,an]),
  writef('[%w, %w,\n', [A,Type]),
  atom_concat(Spaces, '  ', SpacesNext),
  entity_write(Tail, SpacesNext),
  write(']\n').

entity_write([[Key,Val|ValDescr]|Tail], Spaces) :-
  (is_list(Val)
  -> (
    Val=[A,Type|ValTail],
    writef('%w[%w, [%w, %w\n', [Spaces,Key,A,Type]),
    atom_concat(Spaces, '  ', SpacesNext),
    entity_write(ValTail, SpacesNext),
    writef('%w]],\n', [Spaces])
  ) ; (
    writef('%w%w,\n', [Spaces,[Key,Val|ValDescr]])
  )),
  entity_write(Tail, Spaces).

entity_write([Key|Tail], Spaces) :-
  writef('%w, %w\n', [Spaces,Key]),
  entity_write(Tail, Spaces).

entity_write([], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% helper


interval_operator(during, interval_during)   :- !.
interval_operator(before, interval_before)   :- !.
interval_operator(after,  interval_after)    :- !.

entity_interval_axioms([Operator,Value|Axioms], [[Operator,Interval_val]|Tail]) :-
  interval_operator(Operator, _), !,
  entity(Interval, Value),
  interval(Interval, Interval_val),
  entity_interval_axioms(Axioms, Tail).
entity_interval_axioms([_|Axioms], Tail) :-
  entity_interval_axioms(Axioms, Tail).
entity_interval_axioms([], []).

entity_intersection_interval([[during,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_intersect(I1, I_Tail, Intersection).
entity_intersection_interval([[after,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_end(I1, End),
  interval_intersect([End], I_Tail, Intersection).
entity_intersection_interval([[before,I1]|Tail], Intersection) :-
  entity_intersection_interval(Tail, I_Tail),
  interval_start(I1, Begin),
  interval_intersect([0.0,Begin], I_Tail, Intersection).
entity_intersection_interval([], []).

interval_intersect([], I, I).
interval_intersect(I, [], I).
interval_intersect([Begin0,End0], [Begin1,End1], [Begin,End]) :-
  Begin is max(Begin0, Begin1),
  End is min(End0, End1).
interval_intersect([Begin0], [Begin1,End], [Begin,End]) :-
  Begin is max(Begin0, Begin1).
interval_intersect([Begin0,End], [Begin1], [Begin,End]) :-
  Begin is max(Begin0, Begin1).


%% Fluid properties that match temporal relation.
entity_temporally_holds(_, []).
entity_temporally_holds(FluentInterval, [TemporalRelation,IntervalDescr|Tail]) :-
  (  interval_operator(TemporalRelation,Operator)
  -> (
    % TODO: also support entitiy descriptions!!
    interval(IntervalDescr, IntervalDescr_val),
    call(Operator, FluentInterval, IntervalDescr_val)
  ) ; true ),
  entity_temporally_holds(FluentInterval, [IntervalDescr|Tail]).


entity_properties([['http://www.w3.org/1999/02/22-rdf-syntax-ns#type',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,_]|Tail], DescrTail) :-
  rdf_has(PropIri, rdf:type, owl:'AnnotationProperty'),
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#comment',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://www.w3.org/2000/01/rdf-schema#subClassOf',_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([['http://knowrob.org/kb/knowrob.owl#temporalParts',Fluent]|Tail], Descr) :-
  findall([Key,Value,during,IntervalDescr], (
    fluent_has(Fluent, PropIri, PropValue, IntervalIri),
    ( rdf_equal(PropIri, rdf:type)
    -> (
      Key=type,
      entity_iri(PropValue, Value, camelcase)
    ) ; (
      entity_properties([[PropIri,PropValue]], [[Key,Value]])
    )),
    entity(IntervalIri, IntervalDescr)
  ), FluentDescr),
  entity_properties(Tail, DescrTail),
  append(FluentDescr, DescrTail, Descr), !.

entity_properties([[inverse_of(_),_]|Tail], DescrTail) :-
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,TimeIri]|Tail],
                  [[Key, [a,timepoint,Time]]|DescrTail]) :-
  once((nonvar(TimeIri);nonvar(Time))),
  rdfs_individual_of(TimeIri, knowrob:'TimePoint'),
  entity_iri(PropIri, Key, lower_camelcase),
  time_term(TimeIri,Time),
  entity_properties(Tail, DescrTail), !.

entity_properties([[PropIri,IntervalIri]|Tail],
                  [[Key, [an,interval,Interval]]|DescrTail]) :-
  once((nonvar(IntervalIri);nonvar(Interval))),
  interval(IntervalIri,Interval),
  entity_iri(PropIri, Key, lower_camelcase),
  entity_properties(Tail, DescrTail), !. % FIXME: does this disable to have events as value keys?

entity_properties([[PropIri,PropValue]|Tail], [[Key,Value]|DescrTail]) :-
  entity_iri(PropIri, Key, lower_camelcase),
  % match rdf value with description value
  (  rdf_has(PropIri, rdf:type, owl:'DatatypeProperty')
  -> (var(Value) -> strip_literal_type(PropValue, Value) ; rdf_global_term(Value, PropValue))
  ;  entity(PropValue, Value)
  ),
  entity_properties(Tail, DescrTail).

entity_properties([], []).


entity_axioms([[P_descr,O_desc|_]|Descr], AxiomIri, [[P,O]|Axioms]) :-
  entity_iri(P, P_descr, lower_camelcase),
  rdfs_subproperty_of(P, AxiomIri),
  entity(O, O_desc),
  entity_axioms(Descr, AxiomIri, Axioms).

entity_axioms([], _, []).


entity_generate(Entity, [a,timepoint], _, [a,timepoint,Time]) :-
  rdfs_individual_of(Entity, knowrob:'TimePoint'),
  time_term(Entity,Time), !.

entity_generate(Entity, [an,interval], _, [an,interval,Interval]) :-
  rdfs_individual_of(Entity, knowrob:'TemporalThing'),
  interval(Entity,Interval), !.

entity_generate(Pose, [a, pose], _, [a, pose, [X,Y,Z], [QW,QX,QY,QZ]]) :-
  position_to_list(Pose, [X,Y,Z]),
  quaternion_to_list(Pose, [QW,QX,QY,QZ]), !.

entity_generate(Entity, [A,TypeBase], TypeBaseIri, [A,TypeBase|[[type,TypeName]|PropDescr]]) :-
  (( rdf_has(Entity, rdf:type, Type),
     Type \= TypeBaseIri,
     rdf_reachable(Type, rdfs:subClassOf, TypeBaseIri) )
  -> TypeIri = Type
  ;  TypeIri = TypeBaseIri ),
  entity_iri(TypeIri, TypeName, camelcase),
  findall([PropIri,PropValue], rdf(Entity, PropIri, PropValue), Props),
  entity_properties(Props, PropDescr), !.


%% Match [a|an, ?entity_type]
entity_head(Entity, _, Descr, TypeIri) :-
  var(Entity),
  
  % TODO: match all entities that ever were classified with Type?
  % TODO: handle during?
  %current_time(Instant),
  %once((
  %  rdfs_individual_of(Entity, TypeIri);
  %  fluent_has(Entity, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', TypeIri, Instant);
  %  Entity = TypeIri )),
  
  findall(TypeIri, (
    entity_has(Descr, type, TypeDescr), once(
    entity_iri(TypeIri, TypeDescr, camelcase) ;
    rdf_global_term(TypeDescr, TypeIri) )
    % TODO: ensure it's really a type
  ), Types),
  
  ( Types=[]
  -> true
  ; (
    findall(E, (
      owl_individual_of_all(E, Types),
      \+ owl_individual_of(E, knowrob:'TemporalPart')
    ), Entities),
    % avoid redundant results of owl_individual_of
    list_to_set(Entities, EntitiesUnique),
    member(Entity, EntitiesUnique)
  )).

entity_head(Entity, [A,Type], _, TypeIri) :-
  nonvar(Entity),
  findall([A,Type,TypeIri], (
    ( entity_type([A,Type], TypeIri), rdfs_individual_of(Entity, TypeIri) );
    [A,Type,TypeIri]=[a,thing,'http://www.w3.org/2002/07/owl#Thing']
  ), Types),
  member([A,Type,TypeIri], Types),
  % pick only the most special types
  forall( member([_,BType,BTypeIri], Types), (
    BType = Type ; (
      not(rdf_reachable(BTypeIri, rdfs:subClassOf, TypeIri))
    )
  )).

owl_individual_of_all(Individual, [TypeIri|Types]) :-
  owl_individual_of(Individual, TypeIri),
  owl_individual_of_all(Individual, Types).
owl_individual_of_all(_, []).


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
  atom(Entity),
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
