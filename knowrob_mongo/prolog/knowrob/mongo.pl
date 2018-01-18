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
:- module(knowrob_mongo,
    [
      mng_interface/1,      % get handle to the java object of the mongo client
      mng_db/1,             % set the database state
      mng_timestamp/2,
      mng_distinct_values/3,
      mng_value_object/2,   % converts between mongo and prolog representations
      mng_query/2,          % querying the db based on patterns of data records
      mng_query/3,
      mng_query_latest/4,
      mng_query_latest/5,
      mng_query_earliest/4,
      mng_query_earliest/5,
      mng_query_incremental/3,
      mng_cursor/3,         % get a cursor to your query for some advanced processing beyond the mng_query* predicates
      mng_cursor_read/2,
      mng_cursor_process/2,
      mng_cursor_descending/3,
      mng_cursor_ascending/3,
      mng_cursor_limit/3,
      mng_republisher/1,    % get handle to the java object of the mongo republisher
      mng_republish/3,
      mng_republish/5,
      mng_ros_message/2,
      mng_ros_message/4
    ]).
/** <module> Integration of mongo data into symbolic reasoning in KnowRob

@author Moritz Tenorth
@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('jpl')).

:-  rdf_meta
    mng_interface(-),
    mng_republisher(-),
    mng_db(+),
    mng_timestamp(r,r),
    mng_distinct_values(+,+,-),
    mng_query_latest(+,?,+,r),
    mng_query_latest(+,?,+,r,+),
    mng_query_earliest(+,?,+,r),
    mng_query_earliest(+,?,+,r,+),
    mng_query(+,?),
    mng_query(+,?,+),
    mng_query_incremental(+,+,+),
    mng_value_object(+,-),
    mng_ros_message(t,-),
    mng_ros_message(+,+,+,-),
    mng_republish(t,+,-),
    mng_republish(+,+,+,+,-).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% querying the mongo database

%% mng_interface(-Mongo) is det
%
% Get handle to the java object of mongo client.
%
mng_interface(Mongo) :-
    (\+ current_predicate(v_mng_interface, _)),
    jpl_new('org.knowrob.interfaces.mongo.MongoDBInterface', [], Mongo),
    assert(v_mng_interface(Mongo)),!.
mng_interface(Mongo) :-
    current_predicate(v_mng_interface, _),
    v_mng_interface(Mongo).

%% mng_db(+DBName) is nondet.
%
% Change mongo database state used by KnowRob.
% Note: This is currently not threadsafe!
%
% @param DBName  The name of the db (e.g., 'roslog')
%
mng_db(DBName) :-
  mng_interface(Mongo),
  jpl_call(Mongo, 'setDatabase', [DBName], _).

%% mng_timestamp(+Date, -Stamp) is nondet.
%
% Computes a timestamp that corresponds to the specified date.
% date format must be as follows: "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'"
%
% @param Date        String representation of a date
% @param Stamp       Floating point timestamp that represents the date
%
mng_timestamp(Date, Stamp) :-
  mng_interface(DB),
  jpl_call(DB, 'getMongoTimestamp', [Date], Stamp).

%% mng_distinct_values(+Collection, +Key, -Values) is nondet.
% 
% Determine distinct values of the records with Key in Collection.
%
% @param Collection The name of the MONGO DB collection
% @param Key    The field key
% @param Values List of distinct values
% 
mng_distinct_values(Collection, Key, Values) :-
  mng_interface(DB),
  jpl_call(DB, 'distinctValues', [Collection,Key], ValuesArr),
  jpl_array_to_list(ValuesArr, Values).

%% mng_cursor(+Collection, +Pattern, -DBCursor)
%
% Create a DB cursor of query results for records in
% Collection.
% The resulting DB object(s) match the query pattern Pattern.
%
% @param Collection The name of the MONGO DB collection
% @param DBCursor The resulting DB cursor
% @param Pattern The query pattern
%
mng_cursor(Collection, Pattern, DBCursor) :-
  mng_interface(DB),
  findall(Key, member([Key,_,_],Pattern), Keys),
  findall(Rel, member([_,Rel,_],Pattern), Relations),
  findall(Obj, (
      member([_,_,Val], Pattern),
      once(mng_value_object(Val, Obj))
  ), Values),
  jpl_list_to_array(Keys, KeysArray),
  jpl_list_to_array(Relations, RelationsArray),
  jpl_list_to_array(Values, ValuesArray),
  
  jpl_call(DB, 'query', [Collection, KeysArray, RelationsArray, ValuesArray], DBCursor),
  not(DBCursor = @(null)).

%% mng_query_latest(+Collection, -DBObj, +TimeKey, +TimeValue).
%% mng_query_latest(+Collection, -DBObj, +TimeKey, +TimeValue, +Pattern).
%% mng_query_earliest(+Collection, -DBObj, +TimeKey, +TimeValue).
%% mng_query_earliest(+Collection, -DBObj, +TimeKey, +TimeValue, +Pattern).
%% mng_query(+Collection, -DBObj).
%% mng_query(+Collection, -DBObj, +Pattern).
%
% Query for DB object in Collection.
% If a query pattern is given then the resulting DB object(s) must match this pattern.
% DBObj is a term of the form: one(X), all(X) or some(X,Count) where Count is an integer.
% The results are sorted ascending or descending w.r.t. the key TimeKey if given.
%
% @param Collection The name of the MONGO DB collection
% @param DBObj The resulting DB object(s)
% @param TimeKey The DB key used for sorting
% @param TimeValue DB objects earlier/later are ignored (depending on sort mode)
% @param Pattern The query pattern
%
mng_query_latest(Collection, DBObj, TimeKey, TimeValue) :-
  mng_query_latest(Collection, DBObj, TimeKey, TimeValue, []).

mng_query_latest(Collection, DBObj, TimeKey, TimeValue, Pattern) :-
  mng_cursor(Collection, [[TimeKey, '<', date(TimeValue)]|Pattern], DBCursor),
  mng_cursor_descending(DBCursor, TimeKey, DBCursorDescending),
  mng_cursor_read(DBCursorDescending, DBObj).

mng_query_earliest(Collection, DBObj, TimeKey, TimeValue) :-
  mng_query_earliest(Collection, DBObj, TimeKey, TimeValue, []).

mng_query_earliest(Collection, DBObj, TimeKey, TimeValue, Pattern) :-
  mng_cursor(Collection, [[TimeKey, '>', date(TimeValue)]|Pattern], DBCursor),
  mng_cursor_ascending(DBCursor, TimeKey, DBCursorAscending),
  mng_cursor_read(DBCursorAscending, DBObj).

mng_query(Collection, DBObj) :-
  mng_interface(DB),
  jpl_call(DB, 'query', [Collection], DBCursor),
  not(DBCursor = @(null)),
  mng_cursor_read(DBCursor, DBObj).

mng_query(Collection, DBObj, Pattern) :-
  mng_cursor(Collection, Pattern, DBCursor),
  mng_cursor_read(DBCursor, DBObj).

%% mng_query_incremental(+Collection, +Goal, +Pattern) is semidet
%
% Incrementally compute DB objects matching Pattern, and
% call Goal for each result.
% The DBObject is appended as last argument to Goal.
% For example `Goal=my_predicate(X)` turns to `call(my_predicate(X,DBObject))`
% for each resulting DB object.
%
mng_query_incremental(Collection, Goal, Pattern) :-
  mng_cursor(Collection, Pattern, DBCursor),
  mng_cursor_process(DBCursor, Goal).

%% mng_cursor_descending(+In, +Key, -Out).
%% mng_cursor_ascending(+In, +Key, -Out).
%
% Sorts the DB cursor In w.r.t. Key, the sorted collection
% can be accessed via the new DB cursor Out.
%
% @param In Input DB cursor
% @param Key The sort key
% @param Out Output DB cursor
%
mng_cursor_descending(DBCursor, Key, DBCursorDescending) :-
  mng_interface(DB),
  jpl_call(DB, 'descending', [DBCursor, Key], DBCursorDescending).

mng_cursor_ascending(DBCursor, Key, DBCursorAscending) :-
  mng_interface(DB),
  jpl_call(DB, 'ascending', [DBCursor, Key], DBCursorAscending).

%% mng_cursor_limit(+In:javaobject, +N:integer, -Out:javaobject).
%
% Out is the same mongo DB cursor as In but limited to N results.
%
mng_cursor_limit(DBCursor, N, DBCursorLimited) :-
  mng_interface(DB),
  jpl_call(DB, 'limit', [DBCursor, N], DBCursorLimited).

%% mng_cursor_read(+DBCursor, -DBObj)
%
% Read DB objects from cursor.
%
% @param DBCursor The DB cursor
% @param DBObj The resulting DB object(s)
%
mng_cursor_read(DBCursor, DBObj) :-
  (  mng_db_object(DBCursor, DBObj)
  -> ( % close cursor and succeed
    jpl_call(DBCursor, 'close', [], _)
  ) ; ( % close cursor and fail
     jpl_call(DBCursor, 'close', [], _),
     fail
  )).

%% mng_cursor_process(+DBCursor, +Goal) is semidet
%
% Incrementally compute DB objects from DBCursor, and
% call Goal for each result.
% The DBObject is appended as last argument to Goal.
% For example `Goal=my_predicate(X)` turns to `call(my_predicate(X,DBObject))`
% for each resulting DB object.
%
mng_cursor_process(DBCursor, Goal) :-
  repeat,
  (( mng_db_object(DBCursor, one(DBObj)),
     call(Goal, DBObj) )
  -> fail % jump back to repeat
  ;  !    % continue and don't jump back again
  ),
  jpl_call(DBCursor, 'close', [], _).

%% mng_cursor_read(+DBCursor, +DBObjects).
%
% Read DB objects from cursor.
% DBObjects is one of one(DBObj), all(DBObj), or some(DBObj,Count).
%
% @param DBCursor The DB cursor
% @param DBObj The resulting DB object(s)
%
mng_db_object(DBCursor, one(DBObj)) :-
  mng_interface(DB),
  mng_cursor_limit(DBCursor, 1, DBCursorLimited),
  jpl_call(DB, 'one', [DBCursorLimited], DBObj),
  not(DBObj = @(null)).

mng_db_object(DBCursor, some(DBObjs, Count)) :-
  mng_interface(DB),
  mng_cursor_limit(DBCursor, Count, DBCursorLimited),
  jpl_call(DB, 'some', [DBCursorLimited, Count], DBObjsArray),
  not(DBObjsArray = @(null)),
  jpl_array_to_list(DBObjsArray, DBObjs).

mng_db_object(DBCursor, all(DBObjs)) :-
  mng_interface(DB),
  jpl_call(DB, 'all', [DBCursor], DBObjsArray),
  not(DBObjsArray = @(null)),
  jpl_array_to_list(DBObjsArray, DBObjs).

%% mng_value_object(+Val, ObjJava).
%
% Convert value to Java type compatible with MONGO queries.
%
% @param Val The value
% @param ObjJava Java value compatible with MONGO queries
%
mng_value_object(date(Val), ObjJava) :-
  atom(Val), time_term(Val,T),
  mng_value_object(date(T), ObjJava), !.

mng_value_object(date(Val), ObjJava) :-
  number(Val),
  Miliseconds is Val * 1000.0,
  jpl_new('java.lang.Double', [Miliseconds], MilisecondsDouble), 
  jpl_call(MilisecondsDouble, 'longValue', [], MilisecondsLong),
  jpl_new('org.knowrob.interfaces.mongo.types.ISODate', [MilisecondsLong], ISODate),
  jpl_call(ISODate, 'getDate', [], ObjJava), !.

mng_value_object(Val, ObjJava) :-
  integer(Val),
  jpl_new('java.lang.Long', [Val], ObjJava), !.

mng_value_object(Val, ObjJava) :-
  number(Val),
  jpl_new('java.lang.Double', [Val], ObjJava), !.

mng_value_object(Val, Val) :-
  (atom(Val) ; jpl_is_object(Val)), !.

mng_value_object(Val, _) :-
  print_message(warning, domain_error(mng_value_object, [Val])), fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% republishing of stored messages

%% mng_republisher(-Republisher) is det
%
% Get handle to the java object of mongo republisher.
%
mng_republisher(Republisher) :-
    (\+ current_predicate(v_mng_republisher, _)),
    jpl_call('org.knowrob.interfaces.mongo.MongoMessages', get, [], Republisher),
    jpl_list_to_array(['org.knowrob.interfaces.mongo.MongoMessages'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Republisher, Arr], _),
    assert(v_mng_republisher(Republisher)),!.
mng_republisher(Republisher) :-
    current_predicate(v_mng_republisher, _),
    v_mng_republisher(Republisher).

%% mng_republish(+TypedDBObj, +Topic, -Msg).
%% mng_republish(+DBObj, +TypeJava, +TypeString, +Topic, -Msg).
%
% Generate a ROS message based on Mongo DB object
% and publish the message on a specified topic.
% TypedDBObj is one of bool(DBObj), str(DBObj), float32(DBObj),
% float64(DBObj), int32(DBObj), int64(DBObj), image(DBObj),
% pcl(DBObj), camera(DBObj), or tf(DBObj).
%
% @param TypedDBObj The DB object (result of a query)
% @param Topic The message topic
% @param TypeJava The Java class of the message (e.g., 'std_msgs.Bool')
% @param TypeString The message type identifier (e.g., 'std_msgs/Bool')
% @param Msg The generated message
%
mng_republish(bool(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.Bool', 'std_msgs/Bool', Topic, Msg).

mng_republish(str(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.String', 'std_msgs/String', Topic, Msg).

mng_republish(float32(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.Float32', 'std_msgs/Float32', Topic, Msg).

mng_republish(float64(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.Float64', 'std_msgs/Float64', Topic, Msg).

mng_republish(int32(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.Int32', 'std_msgs/Int32', Topic, Msg).

mng_republish(int64(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'std_msgs.Int64', 'std_msgs/Int64', Topic, Msg).

mng_republish(image(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'sensor_msgs.Image', 'sensor_msgs/Image', Topic, Msg).

mng_republish(compressed_image(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'sensor_msgs.CompressedImage', 'sensor_msgs/CompressedImage', Topic, Msg).

mng_republish(pcl(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'sensor_msgs.PointCloud', 'sensor_msgs/PointCloud', Topic, Msg).

mng_republish(camera(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'sensor_msgs.CameraInfo', 'sensor_msgs/CameraInfo', Topic, Msg).

mng_republish(tf(DBObj), Topic, Msg) :-
  mng_republish(DBObj, 'tf.tfMessage', 'tf/tfMessage', Topic, Msg).

mng_republish(DBObj, TypeJava, TypeString, '', Msg) :- !,
  mng_republisher(Republisher),
  jpl_classname_to_class(TypeJava, MsgClass),
  jpl_call(Republisher, 'create', [DBObj,MsgClass,TypeString], Msg).

mng_republish(DBObj, TypeJava, TypeString, Topic, Msg) :-
  mng_republisher(Republisher),
  jpl_classname_to_class(TypeJava, MsgClass),
  jpl_call(Republisher, 'publish', [DBObj,MsgClass,TypeString,Topic], Msg).

%% mng_ros_message(+DBObj, -Msg).
%% mng_ros_message(+DBObj, +TypeJava, +TypeString, -Msg).
%
% Generate a ROS message based on Mongo DB object.
%
% @param DBObj The DB object (result of a query)
% @param TypeJava The Java class of the message (e.g., 'std_msgs.Bool')
% @param TypeString The message type identifier (e.g., 'std_msgs/Bool')
% @param Msg The generated message
%
mng_ros_message(DBObj, Msg) :-
  mng_republish(DBObj, '', Msg).

mng_ros_message(DBObj, TypeJava, TypeString, Msg) :-
  mng_republish(DBObj, TypeJava, TypeString, '', Msg).
