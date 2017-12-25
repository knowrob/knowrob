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
      owl_instance_of/2,
      owl_triple/3,             % ?Subject, ?Predicate, ?Object
      owl_class_properties/3,
      owl_class_properties_some/3,
      owl_class_properties_all/3,
      owl_class_properties_value/3,
      owl_class_properties_nosup/3,
      owl_class_properties_transitive_nosup/3,
      owl_inspect/3,
      owl_write_readable/1,
      owl_readable/2,
      owl_computable_db/1,
map_frame/1,
map_frame_name/1,
rotmat_to_list/2,
position_to_list/2,
quaternion_to_list/2,
      knowrob_instance_from_class/2,
      knowrob_instance_from_class/3
    ]).

:- use_module(library('crypt')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_rdfs')).
:- use_module(library('owl')).
:- use_module(library('knowrob_math')).

% define meta-predicates and allow the definitions
% to be in different source files
%:- meta_predicate knowrob_instance_from_class(0,+,+,-).
%:- multifile knowrob_instance_from_class/3.

:- rdf_meta owl_instance_of(r, t),
            owl_triple(r, r, o),
            owl_class_properties(r,r,t),
            owl_class_properties_some(r,r,t),
            owl_class_properties_all(r,r,t),
            owl_class_properties_value(r,r,t),
            owl_class_properties_nosup(r,r,r),
            owl_class_properties_transitive_nosup(r,r,r),
            owl_inspect(r,r,r),
            owl_readable(r,-),
            owl_write_readable(r),
            owl_computable_db(-),
rotmat_to_list(r,-),
position_to_list(r,-),
quaternion_to_list(r,-),
            knowrob_instance_from_class(r,-),
            knowrob_instance_from_class(r,t,-).

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

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

%% map_frame
map_frame('http://knowrob.org/kb/knowrob.owl#MapFrame').
%% map_frame_name
map_frame_name(FrameName) :-
  rdf_has(knowrob:'MapFrame', knowrob:'frameName', literal(type(_,FrameName))), !.
%map_frame_name(FrameName) :-
  %ros_param_get_string('knowrob/map_frame', FrameName)
map_frame_name('map').

		 /*******************************
		 *		  ABOX reasoning		*
		 *******************************/

%% owl_computable_db
owl_computable_db(db(rdfs_computable_has,rdfs_instance_of)).

%%  owl_instance_of(?Resource, +Description) is nondet.
%
% Test  or  generate  the  resources    that  satisfy  Description
% according the the OWL-Description entailment rules.

owl_instance_of(Resource, Description) :-
  owl_computable_db(DB),
  owl_individual_of(Resource, Description, DB).

%%	owl_has(?Subject, ?Predicate, ?Object)
%
%	True if this relation is specified or can be deduced using OWL
%	inference rules.  It adds transitivity to owl_has_direct/3.

owl_triple(S, P, O) :-
  owl_computable_db(DB),
  owl_has(S, P, O, DB).

		 /*******************************
		 *		  TBOX reasoning		*
		 *******************************/

%% owl_class_properties(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom- and hasValue-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties(Class, Prop, Val) :-
  rdfs_individual_of(Class, owl:'Class'), % make sure Class is bound before calling owl_subclass_of
  (owl_class_properties_some(Class, Prop, Val);
   owl_class_properties_value(Class, Prop, Val)).

%% owl_class_properties_some(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties_some(Class, Prop, Val) :-         % read directly asserted properties
  owl_class_properties_1_some(Class, Prop, Val).

owl_class_properties_some(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  owl_class_properties_1_some(Super, Prop, Val).

owl_class_properties_1_some(Class, Prop, Val) :-       % read all values for some_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, some_values_from(Val))).

%% owl_class_properties_all(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of allValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties_all(Class, Prop, Val) :-         % read directly asserted properties
  owl_class_properties_1_all(Class, Prop, Val).

owl_class_properties_all(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  owl_class_properties_1_all(Super, Prop, Val).

owl_class_properties_1_all(Class, Prop, Val) :-       % read all values for all_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, all_values_from(Val))) .

%% owl_class_properties_value(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of hasValue-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties_value(Class, Prop, Val) :-         % read directly asserted properties
  owl_class_properties_1_value(Class, Prop, Val).

owl_class_properties_value(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  owl_class_properties_1_value(Super, Prop, Val).

owl_class_properties_1_value(Class, Prop, Val) :-       % read all values for has_value restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, has_value(Val))) .

%% owl_class_properties_nosup(?Class, ?Prop, ?Val) is nondet.
%
% Version of owl_class_properties without considering super classes
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties_nosup(Class, Prop, Val) :-         % read directly asserted properties
  owl_class_properties_nosup_1(Class, Prop, Val).

% owl_class_properties_nosup(Class, Prop, Val) :-         % do not consider properties of superclasses
%   owl_subclass_of(Class, Super), Class\=Super,
%   owl_class_properties_nosup_1(Super, Prop, Val).

owl_class_properties_nosup_1(Class, Prop, Val) :-
  owl_direct_subclass_of(Class, Sup),
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  ( owl_restriction(Sup,restriction(SubProp, some_values_from(Val))) ;
    owl_restriction(Sup,restriction(SubProp, has_value(Val))) ).

%% owl_class_properties_transitive_nosup(?Class, ?Prop, ?Val) is nondet.
%
% Transitive cersion of owl_class_properties without considering super classes
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
owl_class_properties_transitive_nosup(Class, Prop, SubComp) :-
    owl_class_properties_nosup(Class, Prop, SubComp).
owl_class_properties_transitive_nosup(Class, Prop, SubComp) :-
    owl_class_properties_nosup(Class, Prop, Sub),
    owl_individual_of(Prop, owl:'TransitiveProperty'),
    Sub \= Class,
    owl_class_properties_transitive_nosup(Sub, Prop, SubComp).

%% owl_inspect(?Thing, ?P, ?O).
%
% Read information stored for Thing. Basically a wrapper around owl_has and
% owl_class_properties that aggregates their results. The normal use case is to
% have Thing bound and ask for all property/object pairs for this class or
% individual.
%
% @param Thing  RDF identifier, either an OWL class or an individual
% @param P      OWL property identifier 
% @param O      OWL class, individual or literal value specified as property P of Thing
% 
owl_inspect(Thing, P, O) :-
  rdf_has(Thing, rdf:type, owl:'NamedIndividual'),
  owl_has(Thing, P, Olit),
  strip_literal_type(Olit, O).
  
owl_inspect(Thing, P, O) :-
  rdf_has(Thing, rdf:type, owl:'Class'),
  owl_class_properties(Thing, P, Olit),
  strip_literal_type(Olit, O).

		 /*******************************
		 *		  Input-Output			*
		 *******************************/

%% owl_write_readable(+Resource) is semidet
% 
% Writes human readable description of Resource.
%
% @param Resource OWL resource
%
owl_write_readable(Resource) :- owl_readable(Resource,Readable), write(Readable).

%% owl_readable(+Resource, -Readable).
%
% Utility predicate to convert RDF terms into a readable representation.
%
owl_readable(class(Cls),Out) :- owl_readable_internal(Cls,Out), !.
owl_readable(Descr,Out) :-
  (is_list(Descr) -> X=Descr ; Descr=..X),
  findall(Y_, (member(X_,X), once(owl_readable_internal(X_,Y_))), Y),
  Out=Y.
owl_readable_internal(P,P_readable) :-
  atom(P), rdf_has(P, owl:inverseOf, P_inv),
  owl_readable_internal(P_inv, P_inv_),
  atomic_list_concat(['inverse_of(',P_inv_,')'], '', P_readable), !.
owl_readable_internal(class(X),Y) :- owl_readable_internal(X,Y).
owl_readable_internal(X,Y) :- atom(X), rdf_split_url(_, Y, X).
owl_readable_internal(X,X) :- atom(X).
owl_readable_internal(X,Y) :- compound(X), rdf_readable(X,Y).

		 /*******************************
		 *		  ABOX ASSERTIONS		*
		 *******************************/

knowrob_instance_from_class(Class, Instance) :-
  ( knowrob_instance_from_class(Class, [], Instance);
    rdf_instance_from_class(Class, Instance)), !.

%%%%%%%%%%%%%%%%%%%
%% knowrob:TimePoint

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimePoint', [], TimePoint) :- !,
  current_time(T),
  knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimePoint', [instant=T], TimePoint).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimePoint', [instant=T], TimePoint) :- !,
  time_term(T,T_value),
  atom_concat('http://knowrob.org/kb/knowrob.owl#timepoint_', T_value, TimePoint),
  rdf_assert(TimePoint, rdf:type, knowrob:'TimePoint').
  
%%%%%%%%%%%%%%%%%%%
%% knowrob:TimeInterval

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', [begin=Start], TimeInterval) :- !,
  time_term(Start, Start_v), 
  knowrob_instance_from_class(knowrob:'TimePoint', [instant=Start_v], StartI),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#TimeInterval',Start_v], '_', TimeInterval),
  rdf_assert(TimeInterval, rdf:type, knowrob:'TimeInterval'),
  rdf_assert(TimeInterval, knowrob:'startTime', StartI).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', [begin=Start,end=End], TimeInterval) :- !,
  time_term(Start, Start_v), time_term(End, End_v), 
  knowrob_instance_from_class(knowrob:'TimePoint', [instant=Start_v], StartI),
  knowrob_instance_from_class(knowrob:'TimePoint', [instant=End_v], EndI),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#TimeInterval',Start_v,End_v], '_', TimeInterval),
  rdf_assert(TimeInterval, rdf:type, knowrob:'TimeInterval'),
  rdf_assert(TimeInterval, knowrob:'startTime', StartI),
  rdf_assert(TimeInterval, knowrob:'endTime', EndI).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', I, TimeInterval) :-
  number(I), !,
  knowrob_instance_from_class(knowrob:'TimeInterval', [begin=I], TimeInterval).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', I, TimeInterval) :-
  interval(I, [Start,End]), !,
  knowrob_instance_from_class(knowrob:'TimeInterval', [begin=Start,end=End], TimeInterval).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#TimeInterval', I, TimeInterval) :-
  interval(I, [Start]), !,
  knowrob_instance_from_class(knowrob:'TimeInterval', [begin=Start], TimeInterval).

%%%%%%%%%%%%%%%%%%%
%% knowrob:Pose

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=(Frame,[X,Y,Z],[QW,QX,QY,QZ])], Pose) :- !,
  rdfs_individual_of(Frame, knowrob:'SpatialThing-Localized'),
  rdf_split_url(_, Ref, Frame),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#Pose'|[Ref,X,Y,Z,QW,QX,QY,QZ]], '_', Pose),
  atomic_list_concat([X,Y,Z], ' ', Translation),
  atomic_list_concat([QW,QX,QY,QZ], ' ', Quaternion),
  rdf_assert(Pose, rdf:type, knowrob:'Pose'),
  rdf_assert(Pose, knowrob:'translation', literal(type(string,Translation))),
  rdf_assert(Pose, knowrob:'quaternion', literal(type(string,Quaternion))),
  rdf_assert(Pose, knowrob:'relativeTo', Frame).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=(Pos,Rot)], Pose) :- !,
  knowrob_instance_from_class(knowrob:'Pose',
      [pose=(knowrob:'MapFrame', Pos, Rot)], Pose).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [mat=(Data)], Pose) :- !,
  matrix_translation(Data, Pos),
  matrix_rotation(Data, Rot),
  knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=(Pos, Rot)], Pose).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=pose(A,B,C)], Pose) :- !,
  knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=(A,B,C)], Pose).

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=pose(A,B)], Pose) :- !,
  knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose', [pose=(A,B)], Pose).

%%%%%%%%%%%%%%%%%%%
%% knowrob:'FrameOfReference'

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#FrameOfReference', [urdf=Name], Frame) :- !,
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#FrameOfReference'|[Name]], '_', Frame),
  rdf_assert(Frame, rdf:type, knowrob:'FrameOfReference'),
  rdf_assert(Frame, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(type(xsd:string, Name))).

%%%%%%%%%%%%%%%%%%%
%% knowrob:'SpaceRegion'

knowrob_instance_from_class('http://knowrob.org/kb/knowrob.owl#SpaceRegion', [axioms=Axioms], SpaceRegion) :- !,
  location_name_args_(Axioms,Args),
  atomic_list_concat(['http://knowrob.org/kb/knowrob.owl#SpaceRegion'|Args], '_', SpaceRegion),
  rdf_assert(SpaceRegion, rdf:type, knowrob:'SpaceRegion'),
  forall( member([P,O], Axioms), rdf_assert(SpaceRegion, P, O) ).

location_name_args_([[P,O]|Axioms], [P_name|[O_name|Args]]) :-
  rdf_split_url(_, P_name, P),
  rdf_split_url(_, O_name, O),
  location_name_args_(Axioms, Args).
location_name_args_([], []).
