/*
  Copyright (C) 2013 Moritz Tenorth
  Copyright (C) 2015 Daniel Beßler
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

:- module(mongo_designators,
    [
      mng_designator/2,
      mng_designator_type/2,
      mng_designator_timestamp/2,
      mng_designator_interval/2,
      mng_designator_location/2,
      mng_designator_object_type/2,
      mng_designator_property/3,
      mng_designator_resolve/2
    ]).
/** <module> Looking up designators in a mongo DB

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/mongo')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:-  rdf_meta
    mng_designator(r,?),
    mng_designator_type(r,?),
    mng_designator_timestamp(r,?),
    mng_designator_interval(r,?),
    mng_designator_location(r,?),
    mng_designator_object_type(r,r),
    mng_designator_property(r,+,+),
    mng_designator_resolve(r,r).

%% mng_designator(+Designator, -DesigJava) is semidet
% 
% Read object that corresponds to Designator into
% a JAVA object DesigJava.
% Designator is either a java mongo DB object,
% or an OWL individual of type knowrob:'Designator'.
%
% @param Designator The designator DB object or OWL individual
% @param DesigJava Java designator object
% 
mng_designator(Designator, DesigJava) :-
  jpl_is_object(Designator), !,
  mng_db_call('designator', [Designator], DesigJava).

mng_designator(Designator, DesigJava) :-
  designator_id(Designator, DesigID),
  mng_query('logged_designators', one(DBObj),
           [['designator._id', 'is', DesigID]]),
  mng_db_call('designator', [DBObj], DesigJava).

jpl_designator(Designator, DesigJava) :-
  atom(Designator), !,
  mng_designator(Designator, DesigJava).
jpl_designator(DesigJava, DesigJava) :-
  jpl_is_object(DesigJava).

%% mng_designator_type(+Designator, ?Type) is semidet
%
% True if Type is the type of Designator.
% This is usually "object", "action", or "location".
% 
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Type        Type of the designator
% 
mng_designator_type(Designator, Type) :-
  jpl_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'getType', [], Type).

%% mng_designator_timestamp(+Designator, ?Instant) is semidet
%
% True if Instant is the time at which Designator was created.
%
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Instant     Floating point value representing the time
% 
mng_designator_timestamp(Designator, Instant) :-
  jpl_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'getInstant', [], Date),
  \+ jpl_null(Date),
  jpl_call(Date, 'getTime', [], Secs),
  Instant is Secs / 1000.0.

%% mng_designator_interval(+Designator, ?Interval) is nondet.
%
% True if Interval is the time interval during which Designator 
% was the most recent representation of an entity.
%
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Interval    [float begin, end] or [float begin]
% 
mng_designator_interval(Designator, Interval) :-
  atom(Designator),
  mng_designator_timestamp(Designator, Begin),
  ( rdf_has(Designator, knowrob:successorDesignator, Succ) -> (
    mng_designator_timestamp(Succ, End),
    Interval = [Begin,End] ) ;
    Interval = [Begin]
  ).

%% mng_designator_location(+Designator, ?Pose) is semidet
%
% True if Pose is the location of Designator.
%
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Pose        The location pose in form of pose([x,y,z],[float qx,qy,qz,qw])
% 
mng_designator_location(Designator, pose(Pos,Rot)) :-
  jpl_designator(Designator, DesigJava),
  mng_db_call('location', [DesigJava], StampedMat),
  jpl_call('tfjava.Utils', 'stampedMatrix4dToPoseArray',
          [StampedMat], TransformationArray),
  jpl_array_to_list(TransformationArray, Transformation),
  matrix(Transformation,Pos,Rot).

%% mng_designator_object_type(+Designator, ?Type) is nondet.
%
% Read the type of a logged designator by its ID
% 
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Type        The type of the object that corresponds to Designator
% 
mng_designator_object_type(Designator, Type) :-
  jpl_designator(Designator, DesigJava),
  (( mng_designator_property(DesigJava, 'TYPE', TypeName) )
  ;( mng_designator_property(DesigJava, 'RESPONSE', TypeName) )
  ;( mng_designator_property(DesigJava, 'DETECTION.TYPE', TypeName) )),
  object_type_to_iri(TypeName, Type), !.

object_type_to_iri(TypeName, Type) :-
  rdf_global_term(TypeName, Type),
  rdf_has(TypeName, rdf:type, _).
object_type_to_iri(TypeName, Type) :-
  atom_concat('http://knowrob.org/kb/knowrob.owl#', TypeName, Type),
  rdf_has(Type, rdf:type, _).
object_type_to_iri(TypeName, Type) :-
  lowercase(TypeName, TypeLower),
  camelcase(TypeLower, TypeCamel),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', TypeCamel, Type),
  rdf_has(Type, rdf:type, _).

%% mng_designator_property(+Designator, ?Key, ?Value) is nondet
%
% Read the properties of a logged designator by its ID
%
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Key         List of property keys for nested designators
% @param Value       The property value
% 
mng_designator_property(Designator, Key, Value) :-
  atom(Key), !,
  atomic_list_concat(KeyList,'.',Key),
  mng_designator_property(Designator, KeyList, Value).
  
mng_designator_property(Designator, Key, Value) :-
  var(Key), !,
  jpl_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'keySet', [], KeySet),
  jpl_set_element(KeySet, X),
  mng_designator_property(Designator, [X], Y),
  ( jpl_ref_to_type(Y,class([org,knowrob,interfaces,mongo,types],['Designator'])) -> (
    mng_designator_property(Y, Xs, Value),
    Key=[X|Xs], Value=Y
  );(
    Key=[X], Value=Y
  )).
  
mng_designator_property(Designator, [Key], Value) :-
  jpl_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'get', [Key], ValIn),
  designator_property_value(ValIn, Value), !.

mng_designator_property(Designator, [Key|SubKeys], Value) :-
  jpl_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'get', [Key], ChildDesigJava),
  \+ jpl_null(ChildDesigJava),
  mng_designator_property(ChildDesigJava, SubKeys, Value).

%% mng_designator_resolve(+Designator, ?Entity) is nondet.
%
% True if Entity is a valid representation for Designator.
%
% @param Designator  Java designator object or OWL individual of knowrob:Designator
% @param Entity      OWL indiviudal that represents the designator
%
mng_designator_resolve(Designator, Entity) :-
  jpl_designator(Designator, DesigJava),
  mng_designator_type(DesigJava, Type),
  mng_designator_resolve(Type, DesigJava, Entity).

% TODO(daniel): improve how designators are resolved to symbols.
%			- match objects only if their pose is close to the designator location.
%			  also consider reusing beliefstate code for identity resolution.
%			- handle actions: try to match existing or else create new facts.
%			                  what is the action designator format?
%			- handle locations: try to match existing or else create new facts.
%			                    what is the location designator format?
mng_designator_resolve(object, Designator, Entity) :-
  mng_designator_object_type(Designator, ObjType),
  rdfs_individual_of(Entity, ObjType).
mng_designator_resolve(action, _Designator, _Entity) :-
  fail.
mng_designator_resolve(location, _Designator, _Entity) :-
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% utility predicates

mng_db_call(Method, Args, Result) :-
  mng_interface(DB),
  jpl_call(DB, Method, Args, Result).

designator_id(Designator, DesigID) :-
  atom(Designator),
  rdf_url_namespace(Designator, Ns), not( Ns = '' ),
  rdf_split_url(_, DesigID, Designator).

designator_property_value(In, [X, Y, Z]) :-
  jpl_ref_to_type(In,  class([javax,vecmath],['Vector3d'])),
  jpl_get(In, x, X),
  jpl_get(In, y, Y),
  jpl_get(In, z, Z), !.
designator_property_value(X, X).
