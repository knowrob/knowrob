/** 

  Copyright (C) 2013 Moritz Tenorth, 2015 Daniel Beßler
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

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/

:- module(knowrob_mongo_designators,
    [
      mng_designator/2,
      mng_designator/3,
      mng_designator/4,
      mng_designator_before/3,
      mng_designator_before/4,
      mng_designator_type/2,
      mng_desig_object/2,
      mng_object_type/2,
      mng_designator_location/2,
      mng_designator_location/3,
      mng_designator_timestamp/2,
      mng_designator_interval/2,
      mng_designator_props/3,
      mng_designator_props/4,
      mng_desig_matches/2,
      mng_obj_pose_by_desig/2,
      mng_object_pose_at_time/4,
      mng_designator_distinct_values/2,
      mng_decision_tree/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_mongo')).
:- use_module(library('knowrob_mongo_interface')).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:-  rdf_meta
    mng_designator(r,?),
    mng_object_type(r,r),
    mng_designator_type(r,?),
    mng_designator(r,+,?),
    mng_designator(r,+,?,+),
    mng_designator_before(r,-,r),
    mng_designator_before(r,-,r,+),
    mng_designator_distinct_values(+,-),
    mng_designator_location(r,?),
    mng_designator_location(r,?,r),
    mng_desig_matches(r, +),
    mng_desig_object(r, r),
    mng_obj_pose_by_desig(r,r),
    mng_object_pose_at_time(r,+,r,?),
    mng_designator_props(r,?),
    mng_designator_props(r,+,+,?),
    mng_designator_type(r,?),
    mng_decision_tree(-).

mng_db_call(Method, Args, Result) :-
  mongo_interface(DB),
  jpl_call(DB, Method, Args, Result).

mng_designator_id(Designator, DesigID) :-
  atom(Designator),
  rdf_url_namespace(Designator, Ns), not( Ns = '' ),
  rdf_split_url(_, DesigID, Designator).

%% mng_designator(+Designator, -DesigJava) is nondet.
%% mng_designator(+DBObj, -DesigJava) is nondet.
%% mng_designator(+DBObj, -DesigJava, +Pattern) is nondet.
%% mng_designator(+DBObj, -DesigJava, +Pattern, +IdKey) is nondet.
% 
% Read object that corresponds to Designator into
% a JAVA object DesigJava.
% 
% FIXME: cache computed designators!!
mng_designator(DBObj, DesigJava) :-
  jpl_is_object(DBObj),
  mng_db_call('designator', [DBObj], DesigJava).

mng_designator(Designator, DesigJava) :-
  mng_designator(Designator, DesigJava, []).

mng_designator(Designator, DesigJava, Pattern) :-
  mng_designator(Designator, DesigJava, Pattern, 'designator._id').

mng_designator(Designator, DesigJava, Pattern, IdKey) :-
  mng_designator_id(Designator, DesigID), % FIXME: what if var(Designator)
  mng_query('logged_designators', one(DBObj), [[IdKey, 'is', DesigID]|Pattern]),
  mng_designator(DBObj, DesigJava).

mng_designator_before(Designator, DesigJava, Time) :-
  mng_designator(Designator, DesigJava, Time, []).

mng_designator_before(Designator, DesigJava, Time, Pattern) :-
  mng_designator_id(Designator, DesigID), % FIXME: what if var(Designator)
  mng_query_latest('logged_designators', one(DBObj), '__recorded',
    Time, [['designator._id', 'is', DesigID]|Pattern]),
  mng_designator(DBObj, DesigJava).

%% mng_designator_distinct_values( +Key, -Values) is nondet.
% 
% Determine distinct field values of designators
%
% @param Key    The field key
% @param Values List of distinct values
% 
mng_designator_distinct_values(Key, Values) :-
  mng_db_call('distinctValues', ['logged_designators', Key], ValuesArr),
  jpl_array_to_list(ValuesArr, Values).

%% mng_obj_pose_by_desig(+Obj, -Pose) is nondet.
% 
% Determine object pose based on the POSE property of a linked designator
%
% @param Obj   Object instance
% @param Pose  Instance of a Perception
% 
mng_obj_pose_by_desig(Obj, Pose) :-
  rdf_has(Obj, knowrob:designator, Designator),
  mng_designator_props(Designator, 'POSE', Pose).

%% mng_designator_type(+Designator, ?Type) is nondet.
%
% Read the type of a logged designator by its ID
% 
% @param Designator  Instance of a designator, having its ID as local part of the IRI
% @param Type        Type of the designator
% 
mng_designator_type(Designator, Type) :-
  atom(Designator),
  mng_designator(Designator, DesigJava),
  not( jpl_null(DesigJava) ),
  jpl_call(DesigJava, 'getType', [], Type).

mng_designator_type(DesigJava, Type) :-
  jpl_is_object(DesigJava),
  jpl_call(DesigJava, 'getType', [], Type).

%% mng_object_type(+Designator, ?Type) is nondet.
%
% Read the type of a logged designator by its ID
% 
% @param Designator  Instance of a designator, having its ID as local part of the IRI
% @param TypeIri     Type IRI of the designator
% 
mng_object_type(Designator, TypeIri) :-
  mng_designator(Designator, DesigJava),
  mng_object_type(Designator, DesigJava, TypeIri).

mng_object_type(Designator, _, TypeIri) :-
  (( mng_designator_props(Designator, 'TYPE', Type) )
  ;( mng_designator_props(Designator, 'RESPONSE', Type) )
  ;( mng_designator_props(Designator, 'DETECTION.TYPE', Type) )),
  mng_type_to_iri(Type, TypeIri), !.

mng_type_to_iri(Type, TypeIri) :-
  rdf_global_term(Type, TypeIri),
  rdf_has(Type, rdf:type, _).
mng_type_to_iri(Type, TypeIri) :-
  atom_concat('http://knowrob.org/kb/knowrob.owl#', Type, TypeIri),
  rdf_has(TypeIri, rdf:type, _), !.
mng_type_to_iri(Type, TypeIri) :-
  lowercase(Type, TypeLower),
  camelcase(TypeLower, TypeCamel),
  atom_concat('http://knowrob.org/kb/knowrob.owl#', TypeCamel, TypeIri),
  rdf_has(TypeIri, rdf:type, _).


%% mng_designator_props(+Designator, ?PropertyPath, ?Value) is nondet.
%
% Read the properties of a logged designator by its ID
%
% @param Designator   Instance of a designator, having its ID as local part of the IRI
% @param PropertyPath Sequence of property keys for nested designators
% @param Value        Value slot of the designator
% 
mng_designator_props(Designator, Prop, Value) :-
  atom(Designator),
  mng_designator(Designator, DesigJava),
  mng_designator_props(Designator, DesigJava, Prop, Value).

%% mng_designator_props(+Designator, +DesigJava, +PropertyPath, ?Value) is nondet.
% 
% Read the properties of a logged designator.
% 
% @param Designator   Instance of a designator, having its ID as local part of the IRI
% @param DesigJava    JAVA instance of the designator
% @param PropertyPath Sequence of property keys for nested designators
% @param Value        Value slot of the designator
% 

mng_designator_props(Designator, DesigJava, PropertyPath, Value) :-
  atom(PropertyPath),
  atomic_list_concat(PropertyPathList,'.',PropertyPath),
  mng_designator_props(Designator, DesigJava, PropertyPathList, Value).

mng_designator_props(Designator, DesigJava, [Prop|Tail], Value) :-
  not( jpl_null(DesigJava) ),
  jpl_call(DesigJava, 'keySet', [], PropsSet),
  jpl_set_element(PropsSet, Prop),
  jpl_call(DesigJava, 'get', [Prop], ChildDesigJava),
  jpl_ref_to_type(ChildDesigJava,  class([org,knowrob,interfaces,mongo,types],['Designator'])),
  mng_designator_props(Designator, ChildDesigJava, Tail, Value).
  
mng_designator_props(Designator, DesigJava, [Prop], Value) :-
  mng_designator_props_value(Designator, DesigJava, Prop, Value).
  
mng_designator_props_value(Designator, DesigJava, Prop, Value) :-
  not( jpl_null(DesigJava) ),
  jpl_call(DesigJava, 'keySet', [], PropsSet),
  jpl_set_element(PropsSet, Prop),
  jpl_call(DesigJava, 'get', [Prop], ValIn),
  once(mng_desig_get_value(Designator, DesigJava, ValIn, Value)).

%% mng_designator_location(+Designator, ?Matrix) is nondet.
%
% Check designator transformation matrix
%
% @param Designator  Instance of a designator, having its ID as local part of the IRI
% @param Matrix      4x4 matrix that represents the designator transformation
% 
mng_designator_location(Designator, Mat) :-
  atom(Designator),
  mng_designator(Designator, DesigJava),
  mng_designator_location(DesigJava, Mat).
  
mng_designator_location(DesigJava, Mat) :-
  jpl_is_object(DesigJava),
  mng_db_call('location', [DesigJava], JplMat),
  jpl_is_object(JplMat),
  jpl_matrix_list(JplMat, Mat).

mng_designator_location(Designator, Mat, T) :-
  atom(Designator),
  mng_designator(Designator, DesigJava, T),
  mng_designator_location(DesigJava, Mat).

%% mng_designator_timestamp(+Designator, ?Timestamp) is nondet.
%
% Check designator timestamp
%
% @param Designator  Instance of a designator, having its ID as local part of the IRI
% @param Timestamp   Floating point value representing the time
% 
mng_designator_timestamp(Designator, Timestamp) :-
  atom(Designator),
  once((
    rdf_has(Obj, knowrob:designator, Designator),
    rdf_has(Ev, knowrob:objectActedOn, Obj),
    rdfs_individual_of(Ev, knowrob:'Event'),
    interval_start(Ev, Timestamp)
  ) ; (
    mng_designator(Designator, DesigJava),
    mng_designator_timestamp(DesigJava, Timestamp)
  )).
  
mng_designator_timestamp(DesigJava, Timestamp) :-
  jpl_is_object(DesigJava),
  jpl_call(DesigJava, 'getInstant', [], Date),
  not(Date = @(null)),
  jpl_call(Date, 'getTime', [], Time),
  Timestamp is Time / 1000.0.
  
mng_designator_interval(Designator, Interval) :-
  atom(Designator),
  mng_designator(Designator, DesigJava),
  mng_designator_interval(Designator, DesigJava, Interval), !.

mng_designator_interval(Designator, DesigJava, Interval) :-
  jpl_is_object(DesigJava),
  mng_designator_timestamp(Designator, Begin),
  (  rdf_has(Designator, knowrob:successorDesignator, Succ)
  ->  (
      mng_designator_timestamp(Succ, End),
      Interval = [Begin,End]
      )
  ;   Interval = [Begin] ).

% TODO remove
mng_decision_tree(DesigJava) :-
  mng_query('resulting_decision_tree', one(DBObj)),
  mng_designator(DBObj, DesigJava).

%% mng_desig_get_value(?Designator, +DesigJava, +Prop, -Value).
% 
% Internal helper method: handle the different kinds of designator values
%
% @param Designator  Designator instance
% @param DesigJava   Java handle of a org.knowrob.mongo.types.Designator object
% @param Prop        Designator property
% @param Value       Value extracted from the designator, e.g. a primitive value
%                    (string, float etc), the handle of a nested designator object,
%                    or an instance of a RotationMatrix3D of an object pose
%

% create designator instance for child-designators
%mng_desig_get_value(_Designator, DesigJava, ValIn, Value) :-
%  jpl_ref_to_type(ValIn,  class([org,knowrob,interfaces,mongo,types],['Designator'])),
%  Value=ValIn. % TODO

% NOTE(daniel): Commented because
%    - PoseStamp is not used anymore in Designator class, so this will never be the case
%    - A getter function should not assert anything
% create observation of the object to which the designator is attached.
%mng_desig_get_value(Designator, DesigJava, PoseStamped, Pose) :-
%
%  jpl_ref_to_type(PoseStamped,  class([org,knowrob,interfaces,mongo,types],['PoseStamped'])),
%
%  % find out which object we are talking about
%  rdf_has(Obj, knowrob:designator, Designator),
%
%  % get pose time
%  jpl_get(PoseStamped, 'header', Header),
%  jpl_get(Header, 'stamp', Stamp),
%  jpl_get(Stamp,  'secs', TimeSecs),
%  term_to_atom(TimeSecs, TimeSecsAtom),
%  atom_concat('http://knowrob.org/kb/knowrob_mongo.owl#timepoint_', TimeSecsAtom, PoseTimePoint),
%
%  % transform into /map
%  jpl_call(PoseStamped, 'getMatrix4d', [], PoseMatrix4d),
%  jpl_get(Header, 'frame_id', SourceFrame),
%  knowrob_coordinates:matrix4d_to_list(PoseMatrix4d, PoseListIn),
%  mng_transform_pose(PoseListIn, SourceFrame, '/map', PoseTimePoint, PoseListOut),
%  create_pose(PoseListOut, Pose),
%
%  % determine detection type (e.g. perception)
%  jpl_call(DesigJava, 'getDetectionType', [], DetectionType),
%
%  % create perception instance attached to the object this designator belongs to
%  atom_concat('http://knowrob.org/kb/knowrob.owl#', DetectionType, DClass),
%  rdf_instance_from_class(DClass, Detection),
%  set_object_perception(Obj, Detection),
%  rdf_assert(Detection, knowrob:eventOccursAt, Pose),
%
%  rdf_assert(PoseTimePoint, rdf:type, 'http://knowrob.org/kb/knowrob.owl#TimePoint'),
%  rdf_assert(Detection, knowrob:startTime, PoseTimePoint).

mng_desig_get_value(_Designator, _DesigJava, Vec, Vector) :-
  jpl_ref_to_type(Vec,  class([javax,vecmath],['Vector3d'])),
  jpl_get(Vec, x, X), jpl_get(Vec, y, Y), jpl_get(Vec, z, Z),
  Vector = [X, Y, Z].

% just return the value for other properties
mng_desig_get_value(_Designator, _DesigJava, ValIn, Value) :-
  Value = ValIn.


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% designator matches predicate 
%


%% mng_desig_matches(?Designator, +QueryPattern) is nondet.
%
% This predicate allows to retrieve designators from the log database that
% match a query pattern given as nested lists of key-value pairs. An example
% of such a query pattern may look like 
% [an, action, [type, navigation], [goal, [a, location, [to, see], [object_acted_on, [type, 'PANCAKEMIX']]]]]
%
% @param Designator    Designator instance that matches the pattern
% @param QueryPattern  Query pattern as nested lists
% 
mng_desig_matches(Designator, QueryPattern) :-
  % convert query pattern into list of query strings suitable for MongoDB queries
  desig_list_to_query(QueryPattern, 'designator', QueryStrings),
  pairs_keys_values(QueryStrings, QueryKeys, QueryValues),

  jpl_list_to_array(QueryKeys, QueryKeysArr),
  jpl_list_to_array(QueryValues, QueryValuesArr),
  length(QueryKeys, N), N > 0,
  
  % send MongoDB query:
  mng_db_call('queryDesignatorsByPattern', [QueryKeysArr, QueryValuesArr], DesigJavaArr),
  not(DesigJavaArr = @(null)),

  jpl_array_to_list(DesigJavaArr, DesigJavaList),
  
  member(DesigJava, DesigJavaList),
  not(DesigJava = @(null)),
  jpl_call(DesigJava, 'get', ['_ID'], DesigID),
  not(DesigID = @(null)),
  mng_desig_iri(DesigID, Designator).


mng_desig_iri(DesigID, Designator) :-
  rdf_split_url('http://knowrob.org/kb/log.owl#', DesigID, Designator),
  rdfs_individual_of(Designator, knowrob:'Designator').
mng_desig_iri(DesigID, Designator) :-
  % fallback case
  rdf_split_url('http://knowrob.org/kb/cram_log.owl#', DesigID, Designator).


%% desig_list_to_query(+ConstrList, +Prefix, -QueryStringList)
%
% Generate a list of query strings that can be used to send queries
% to MongoDB. The keys are chained hierarchically using the dot
% notation. Both keys and values are converted by the lispify_desig
% predicate that, by default, converts them to UPPERCASE.
%
% @param ConstrList       List of constraints of the form [Key, Val], while Val may either be an atom or a nested list
% @param Prefix           Prefix to be used for constructing the resulting query strings
% @param QueryStringList  List of key-value pairs to be used in a MongoDB query, e.g.  'producer.company'-'ABC123'
%

% special list starts:
desig_list_to_query(DesigList, Prefix, QueryStringList) :-

    once( (member(Pre, [[an, action], [an, object], [a, location]]),
           append(Pre, Rest, DesigList)) ),

    findall(QSL, (member(Desig, Rest),
                  once(desig_list_to_query(Desig, Prefix, QSL))), QueryStringLists),

    flatten(QueryStringLists, QueryStringList).


% simple case: normal key/value pair
desig_list_to_query([Key, Val], Prefix, Str-LispVal) :-
    atom(Key), atom(Val), !,
    
    % FIXME: more generic solution
    not(member(Key, [
      'path_to_cad_model',
      'described_in_map',
      'visually_above'
    ])),

    once(lispify_desig(Key, LispKey)),
    once(lispify_desig(Val, LispVal)),

    atomic_list_concat([Prefix, '.', LispKey], Str).


% recursive case: value is a list, we have to iterate
desig_list_to_query([Key, Val], Prefix, QueryStringList) :-
    atom(Key), is_list(Val), !,

    once(lispify_desig(Key, LispKey)),
    atomic_list_concat([Prefix, '.', LispKey], NewPrefix),
    
    desig_list_to_query(Val, NewPrefix, QueryStringList).
    


%% lispify_desig(?QueryVal, ?LispVal) is det.
%
% Convert values in the query language to the corresponding Lisp
% identifiers. Special transforms can be defined, while the default
% is just to convert the values to UPPERCASE.
%
% @param QueryVal  Identifier in the query language
% @param LispVal   Identifier used in Lisp and in the MongoDB logs
%

lispify_desig('object_acted_on', 'OBJ').

% default: do not modify value
lispify_desig(A, CapA) :-
  upcase_atom(A, CapA).

jpl_matrix_list(JplMat, [X00, X01, X02, X03,
                         X10, X11, X12, X13,
                         X20, X21, X22, X23,
                         X30, X31, X32, X33]) :-
  jpl_call(JplMat, 'getElement', [0,0], X00),
  jpl_call(JplMat, 'getElement', [0,1], X01),
  jpl_call(JplMat, 'getElement', [0,2], X02),
  jpl_call(JplMat, 'getElement', [0,3], X03),
  jpl_call(JplMat, 'getElement', [1,0], X10),
  jpl_call(JplMat, 'getElement', [1,1], X11),
  jpl_call(JplMat, 'getElement', [1,2], X12),
  jpl_call(JplMat, 'getElement', [1,3], X13),
  jpl_call(JplMat, 'getElement', [2,0], X20),
  jpl_call(JplMat, 'getElement', [2,1], X21),
  jpl_call(JplMat, 'getElement', [2,2], X22),
  jpl_call(JplMat, 'getElement', [2,3], X23),
  jpl_call(JplMat, 'getElement', [3,0], X30),
  jpl_call(JplMat, 'getElement', [3,1], X31),
  jpl_call(JplMat, 'getElement', [3,2], X32),
  jpl_call(JplMat, 'getElement', [3,3], X33).




mng_object_pose_at_time(Object, Instant, Pose, I) :-
  nonvar(Instant),
  rdf_has(Object, srdl2comp:'urdfName', literal(UrdfName)),
  \+ rdfs_individual_of(Object, knowrob:'TemporalPart'),
  atom_ensure_prefix(UrdfName, '/', UrdfNameResolved),
  % FIXME: /map frame 
  mng_lookup_transform('/map', UrdfNameResolved, Instant, Pose),
  !.

mng_object_pose_at_time(Object, Instant, Pose, I) :-
  nonvar(Object),
  \+ rdfs_individual_of(Object, knowrob:'TemporalPart'),
  %not( object_pose_specified(Object) ), % TODO not specified at time?
  entity(Object, Descr),
  mng_object_compute(Object, Descr),
  holds(Object, knowrob:pose, Pose, Instant),
  holds(Object, knowrob:pose, Pose, I), !.



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob_owl entity descriptions


% TODO(daniel): howto unit test?
% ?- register_ros_package(knowrob_mongo).
% ?- mng_db('Pick-and-Place_pr2-general-pick-and-place_0').
% ?- entity(Object, [an, object, [type, spatula]]).

knowrob_owl:entity_compute(Object, [an,object|Descr]) :-
  mng_object_compute(Object, [an,object|Descr]),
  % Make sure object matches description
  once(entity(Object, [an,object|Descr])).

mng_object_compute(Object, [an,object|Descr]) :-
  % Query for designators matching the pattern
  % Some statements such as `during` or computable properties are ignored,
  % but objects are matched exactly with input query later
  once(mng_desig_matches(Designator, [an,object|Descr])), % FIXME: once save here?
  mng_desig_chain(Designator, Designators),
  % Only proceed for first designator in chain
  Designators=[[_,FirstDesignator]|_],
  % Find OWL individual for designator
  % FIXME: is it the same for all designators in the chain?
  ( var(Object) -> mng_desig_object(FirstDesignator, Object) ; true ),
  % Assert perceptions
  forall(member([_,D], Designators), assert_object_perception(Object, D)).


% TODO: merge designators if confident that they describe the same object
mng_desig_chain(Designator, Designators) :-
  rdf_reachable(FirstDesig, knowrob:'successorDesignator', Designator),
  \+ rdf_has(_, knowrob:'successorDesignator', FirstDesig),
  mng_desig_chain_symbolic(FirstDesig, Designators),
  length(Designators,L), L>1, !.

mng_desig_chain(Designator, Designators) :-
  mng_designator(Designator, DesigJava),
  jpl_call(DesigJava, 'get', ['NAME'], Name), Name \= @(null),
  findall([Instant,D], (
    mng_desig_matches(D, [an,object,[name, Name]]),
    mng_designator_timestamp(D, Instant)
  ), DesignatorsUnsorted),
  sort(DesignatorsUnsorted, Designators),
  assert_successor_chain(Designators), !.

mng_desig_chain(Designator, [Instant, Designator]) :-
  mng_designator_timestamp(Designator, Instant).


mng_desig_chain_symbolic(First, [[Instant,First]|Tail]) :-
  rdf_has(First, knowrob:'successorDesignator', Next)
  -> (
    mng_designator_timestamp(First, Instant),
    mng_desig_chain(Next, [[NextInstant,Next]|Tail_]),
    Tail=[[NextInstant,Next]|Tail_]
  ) ; (
    Tail=[]
  ).


assert_successor_chain([[_,D0],[T1,D1]|Tail]) :-
  rdf_assert(D0, rdf:'type', knowrob:'Designator'),
  rdf_assert(D0, knowrob:'successorDesignator', D1),
  assert_successor_chain([[T1,D1]|Tail]).

assert_successor_chain([[_,D]]) :-
  rdf_assert(D, rdf:'type', knowrob:'Designator').

  
assert_object_perception(Object, Designator) :-
  rdf_has(Object, knowrob:'temporalParts', Part),
  rdf_has(Part, knowrob:'designator', Designator), !.

assert_object_perception(Object, Designator) :-
  mng_designator(Designator, DesigJava),
  mng_designator_interval(Designator, DesigJava, I),
  mng_object_type(Designator, DesigJava, TypeIri),
  % TODO: handle relative poses
  mng_designator_location(DesigJava, DesigPose),
  
  % assert perceived object properties
  create_fluent(Object, Fluent, I),
  create_pose(mat(DesigPose), Pose),
  rdf_assert(Fluent, knowrob:designator, Designator),
  
  % TODO: more generic object property handling
  %         - consider all keys that can be mapped to properties
  % TODO: assert bounding box, shape, color fluent
  fluent_assert(Fluent, rdf:type, nontemporal(TypeIri)),
  fluent_assert(Fluent, knowrob:pose, nontemporal(Pose)).


mng_desig_object(Designator, Object) :-
  rdf_has(Object, knowrob:temporalParts, Part),
  \+ rdfs_individual_of(Object, knowrob:'TemporalPart'),
  rdf_has(Part, knowrob:designator, D),
  (rdf_reachable(D, knowrob:'successorDesignator', Designator);
   rdf_reachable(Designator, knowrob:'successorDesignator', D)), !.

mng_desig_object(Designator, Object) :-
  mng_designator(Designator, DesigJava),
  mng_designator_interval(Designator, DesigJava, I),
  % TODO: handle relative poses
  mng_designator_location(DesigJava, DesigPose),
  mng_object_type(Designator, DesigJava, TypeIri),
  mng_desig_find_object(Designator, TypeIri, DesigPose, I, Object), !.

mng_desig_object(_, Object) :-
  rdf_instance_from_class(knowrob:'EnduringThing-Localized', Object).


% Find object with matching pose
%mng_desig_find_object(_, _, DesigPose, I, Object) :-
%  object_pose_at_time(Object, I, pose([OX,OY,OZ], _)),
%  matrix_translation(DesigPose, [DX,DY,DZ]),
%  % TODO: Use additianal informartion
%  %     - did the object moved?
%  %     - was there an object at the begin/end of the interval at this location?
%  %     - did actions occured that acted on objects located at this locatoin during the interval?
%  % compare poses, use epsilon=3cm
%  =<( abs( DX - OX), 0.03),
%  =<( abs( DY - OY), 0.03),
%  =<( abs( DZ - OZ), 0.03), !.

% FIXME: something not working below
% Find object with unspecified pose
mng_desig_find_object(_, TypeIri, _, _, Object) :-
  holds(Object, rdf:type, TypeIri),
  \+ rdfs_individual_of(Object, knowrob:'TemporalPart'),
  \+ object_pose_specified(Object), !.
% TODO: add case for designator temporal part?
