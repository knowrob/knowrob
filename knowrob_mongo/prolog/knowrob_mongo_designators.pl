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
      mng_designator_location/2,
      mng_designator_location/3,
      mng_designator_props/3,
      mng_designator_props/4,
      mng_desig_matches/2,
      mng_obj_pose_by_desig/2,
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
    mng_designator(r,+,?),
    mng_designator(r,+,?,+),
    mng_designator_before(r,-,r),
    mng_designator_before(r,-,r,+),
    mng_designator_distinct_values(+,-),
    mng_designator_location(r,?),
    mng_designator_location(r,?,r),
    mng_desig_matches(r, +),
    mng_obj_pose_by_desig(r,r),
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
mng_designator(DBObj, DesigJava) :-
  jpl_is_object(DBObj),
  mng_db_call('designator', [DBObj], DesigJava).

mng_designator(Designator, DesigJava) :-
  mng_designator(Designator, DesigJava, []).

mng_designator(Designator, DesigJava, Pattern) :-
  mng_designator(Designator, DesigJava, Pattern, 'designator._id').

mng_designator(Designator, DesigJava, Pattern, IdKey) :-
  mng_designator_id(Designator, DesigID),
  mng_query('logged_designators', one(DBObj), [[IdKey, 'is', DesigID]|Pattern]),
  mng_designator(DBObj, DesigJava).

mng_designator_before(Designator, DesigJava, Time) :-
  mng_designator(Designator, DesigJava, Time, []).

mng_designator_before(Designator, DesigJava, Time, Pattern) :-
  mng_designator_id(Designator, DesigID),
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
  jpl_call(DesigJava, 'getType', [], Type).

mng_designator_type(DesigJava, Type) :-
  jpl_is_object(DesigJava),
  jpl_call(DesigJava, 'getType', [], Type).


%% mng_designator_props(+Designator, ?PropertyPath, ?Value) is nondet.
%
% Read the properties of a logged designator by its ID
%
% @param Designator   Instance of a designator, having its ID as local part of the IRI
% @param PropertyPath Sequence of property keys for nested designators
% @param Value        Value slot of the designator
% 
mng_designator_props(Designator, Prop, Value) :-
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
  
  % send MongoDB query:
  mng_db_call('queryDesignatorsByPattern', [QueryKeysArr, QueryValuesArr], DesigJavaArr),
  not(DesigJavaArr = @(null)),

  jpl_array_to_list(DesigJavaArr, DesigJavaList),
  
  member(DesigJava, DesigJavaList),
  not(DesigJava = @(null)),
  jpl_call(DesigJava, 'get', ['_ID'], DesigID),
  not(DesigID = @(null)),
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
    atom(Key), atom(Val),

    once(lispify_desig(Key, LispKey)),
    once(lispify_desig(Val, LispVal)),

    atomic_list_concat([Prefix, '.', LispKey], Str).


% recursive case: value is a list, we have to iterate
desig_list_to_query([Key, Val], Prefix, QueryStringList) :-
    atom(Key), is_list(Val),

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