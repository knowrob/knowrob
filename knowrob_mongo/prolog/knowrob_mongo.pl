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
:- module(knowrob_mongo,
    [
      mng_db/1,
      mng_timestamp/2,
      mng_query_latest/4,
      mng_query_latest/5,
      mng_query_earliest/4,
      mng_query_earliest/5,
      mng_query/2,
      mng_query/3,
      obj_blocked_by_in_camera/4,
      obj_visible_in_camera/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).
:- use_module(library('knowrob_objects')).
:- use_module(library('knowrob_perception')).
:- use_module(library('knowrob_coordinates')).
:- use_module(library('knowrob_mongo_interface')).
:- use_module(library('srdl2')).


:-  rdf_meta
    mng_db(+),
    mng_timestamp(r,r),
    mng_query_latest(+,?,+,r),
    mng_query_latest(+,?,+,r,+),
    mng_query_earliest(+,?,+,r),
    mng_query_earliest(+,?,+,r,+),
    mng_query(+,?),
    mng_query(+,?,+),
    obj_blocked_by_in_camera(r, r, r, r),
    obj_visible_in_camera(r, r, r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).


%% mng_db(+DBName) is nondet.
%
% Change mongo database used for future queries
%
% @param DBName  The name of the db (e.g., 'roslog')
%
mng_db(DBName) :-
  mongo_interface(Mongo),
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
  mongo_interface(DB),
  jpl_call(DB, 'getMongoTimestamp', [Date], Stamp).

%% mng_db_cursor(+Collection, +Pattern, -DBCursor)
%
% Query for DB cursor in collection @Collection.
% The resulting DB object(s) must match the query pattern @Pattern.
%
% @param Collection The name of the MONGO DB collection
% @param DBCursor The resulting DB cursor
% @param Pattern The query pattern
%
mng_db_cursor(Collection, Pattern, DBCursor) :-
  mongo_interface(DB),
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

%% mng_query_latest(+Collection, -DBObj, +TimeKey, +TimeValue)
%% mng_query_latest(+Collection, -DBObj, +TimeKey, +TimeValue, +Pattern)
%% mng_query_earliest(+Collection, -DBObj, +TimeKey, +TimeValue)
%% mng_query_earliest(+Collection, -DBObj, +TimeKey, +TimeValue, +Pattern)
%% mng_query(+Collection, -DBObj)
%% mng_query(+Collection, -DBObj, +Pattern)
%
% Query for DB object in collection @Collection.
% If a query pattern is given then the resulting DB object(s) must match this pattern.
% @DBObj is a term of the form: one(X), all(X) or some(X,Count) where Count is an integer.
% The results are sorted ascending or descending w.r.t. the key @TimeKey if given.
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
  mng_db_cursor(Collection, [[TimeKey, '<', date(TimeValue)]|Pattern], DBCursor),
  mng_descending(DBCursor, TimeKey),
  mng_read_cursor(DBCursor, DBObj).

mng_query_earliest(Collection, DBObj, TimeKey, TimeValue) :-
  mng_query_latest(Collection, DBObj, TimeKey, TimeValue, []).

mng_query_earliest(Collection, DBObj, TimeKey, TimeValue, Pattern) :-
  mng_db_cursor(Collection, [[TimeKey, '>', date(TimeValue)]|Pattern], DBCursor),
  mng_ascending(DBCursor, TimeKey),
  mng_read_cursor(DBCursor, DBObj).

mng_query(Collection, DBObj) :-
  mng_query(Collection, DBObj, []).

mng_query(Collection, DBObj, Pattern) :-
  mng_db_cursor(Collection, Pattern, DBCursor),
  mng_read_cursor(DBCursor, DBObj).

%% mng_descending(+DBCursor, +Key)
%% mng_ascending(+DBCursor, +Key)
%
% Sorts the DB cursor @DBCursor w.r.t. the DB object key @Key.
%
% @param DBCursor The DB cursor
% @param Key The sort key
%
mng_descending(DBCursor, Key) :-
  mongo_interface(DB),
  jpl_call(DB, 'descending', [DBCursor, Key], _).

mng_ascending(DBCursor, Key) :-
  mongo_interface(DB),
  jpl_call(DB, 'ascending', [DBCursor, Key], _).

%% mng_read_cursor(+DBCursor, -DBObj)
%
% Read DB objects from cursor.
%
% @param DBCursor The DB cursor
% @param DBObj The resulting DB object(s)
%
mng_read_cursor(DBCursor, DBObj) :-
  (  mng_db_object(DBCursor, DBObj)
  -> ( % close cursor and succeed
    jpl_call(DBCursor, 'close', [], _)
  ) ; ( % close cursor and fail
     jpl_call(DBCursor, 'close', [], _),
     fail
  )).

%% mng_read_cursor(+DBCursor, one(-DBObj))
%% mng_read_cursor(+DBCursor, all(-DBObj))
%% mng_read_cursor(+DBCursor, some(-DBObj))
%
% Read DB objects from cursor.
%
% @param DBCursor The DB cursor
% @param DBObj The resulting DB object(s)
%
mng_db_object(DBCursor, one(DBObj)) :-
  mongo_interface(DB),
  jpl_call(DB, 'one', [DBCursor], DBObj),
  not(DBObj = @(null)).

mng_db_object(DBCursor, some(DBObjs, Count)) :-
  mongo_interface(DB),
  jpl_call(DB, 'some', [DBCursor, Count], DBObjsArray),
  not(DBObjsArray = @(null)),
  jpl_list_to_array(DBObjs, DBObjsArray).

mng_db_object(DBCursor, all(DBObjs)) :-
  mongo_interface(DB),
  jpl_call(DB, 'all', [DBCursor], DBObjsArray),
  not(DBObjsArray = @(null)),
  jpl_list_to_array(DBObjs, DBObjsArray).

%% mng_value_object(date(+Val), ObjJava)
%% mng_value_object(+Val, ObjJava)
%
% Convert value to Java type compatible with MONGO queries.
%
% @param Val The value
% @param ObjJava Java value compatible with MONGO queries
%
mng_value_object(date(Val), ObjJava) :-
  Miliseconds is Val * 1000.0,
  jpl_new('java.lang.Double', [Miliseconds], MilisecondsDouble), 
  jpl_call(MilisecondsDouble, 'longValue', [], MilisecondsLong),
  jpl_new('org.knowrob.interfaces.mongo.types.ISODate', [MilisecondsLong], ISODate),
  jpl_call(ISODate, 'getDate', [], ObjJava).

mng_value_object(Val, ObjJava) :-
  integer(Val),
  jpl_new('java.lang.Long', [Val], ObjJava).

mng_value_object(Val, ObjJava) :-
  float(Val),
  jpl_new('java.lang.Double', [Val], ObjJava).

mng_value_object(Val, Val) :- atom(Val).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Higher-level reasoning methods
%
% TODO(daniel): Move to another module

%% obj_visible_in_camera(+Obj, ?Camera, +TimePoint) is nondet.
%
% Check if Obj is visible by Camera at time TimePoint by reading the camera
% properties from the robot's SRDL description and computing whether the
% object center is inside the view frustrum.
%
% @param Obj        Instance of an object in the scene
% @param Camera     Instance of an srdl2comp:Camera
% @param TimePoint  Instance of a knowrob:TimePoint at which the scene is to be evaluated
% 
obj_visible_in_camera(Obj, Camera, TimePoint) :-

  findall(Camera, owl_individual_of(Camera, srdl2comp:'Camera'), Cameras),
  member(Camera, Cameras),

  % Read camera properties: horizontal field of view, aspect ratio -> vertical field of view
  once(owl_has(Camera, srdl2comp:hfov, literal(type(_, HFOVa)))),
  term_to_atom(HFOV, HFOVa),

  once(owl_has(Camera, srdl2comp:imageSizeX, literal(type(_, ImgXa)))),
  term_to_atom(ImgX, ImgXa),

  once(owl_has(Camera, srdl2comp:imageSizeY, literal(type(_, ImgYa)))),
  term_to_atom(ImgY, ImgYa),

  VFOV is ImgY / ImgX * HFOV,


  % Read object pose w.r.t. camera
  once(owl_has(Camera, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(CamFrameID))),
  atom_concat('/', CamFrameID, CamFrame),

  % TODO: mng_latest_designator_before_time does not refer to Obj
  (object_pose_at_time(Obj, TimePoint, PoseListObj); mng_latest_designator_before_time(TimePoint, 'object', PoseListObj)),
  mng_transform_pose(PoseListObj, '/map', CamFrame, TimePoint, RelObjPose),

  RelObjPose = [_,_,_,ObjX,_,_,_,ObjY,_,_,_,ObjZ,_,_,_,_],

  BearingX is atan2(ObjY, ObjX),
  BearingY is atan2(ObjZ, ObjX),

  abs(BearingX) < HFOV/2,
  abs(BearingY) < VFOV/2.




%% obj_blocked_by_in_camera(?Obj, ?Blocker, ?Camera, +TimePoint) is nondet.
% 
% Check if the view on Obj from Camera at time TimePoint is blocked by object
% Blocker by reading the camera properties from the robot's SRDL description
% and by computing whether the difference in bearing between the two objects'
% center points from the camera viewpoint is less than ten degrees.
%
% @param Obj        Instance of an object in the scene
% @param Blocker    Instance of an object in the scene
% @param Camera     Instance of an srdl2comp:Camera
% @param TimePoint  Instance of a knowrob:TimePoint at which the scene is to be evaluated
% 
obj_blocked_by_in_camera(Obj, Blocker, Camera, TimePoint) :-

  findall(Camera, owl_individual_of(Camera, srdl2comp:'Camera'), Cameras),
  member(Camera, Cameras),

  % Read camera frame ID
  once(owl_has(Camera, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(CamFrameID))),
  atom_concat('/', CamFrameID, CamFrame),


  % Read object pose w.r.t. camera
  (object_pose_at_time(Obj, TimePoint, PoseListObj); mng_latest_designator_before_time(TimePoint, 'object', PoseListObj)),
  mng_transform_pose(PoseListObj, '/map', CamFrame, TimePoint, ObjPoseInCamFrame),
  ObjPoseInCamFrame = [_,_,_,ObjX,_,_,_,ObjY,_,_,_,ObjZ,_,_,_,_],
  ObjBearingX is atan2(ObjY, ObjX),
  ObjBearingY is atan2(ObjZ, ObjX),

  % debug
%   ObjXDeg is ObjBearingX /2 /pi * 360,
%   ObjYDeg is ObjBearingY /2 /pi * 360,
%%% PR2 ???? what is this doing here???
%%% 
  % Read poses of blocking robot parts w.r.t. camera
  sub_component('http://knowrob.org/kb/PR2.owl#PR2Robot1', Blocker),
  rdfs_individual_of(Blocker, 'http://knowrob.org/kb/srdl2-comp.owl#UrdfLink'),
  owl_has(Blocker, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(PartFrameID)),
  atom_concat('/', PartFrameID, PartFrame),

%   print(PartFrame),
  % transform identity pose from robot part frame to camera frame
  mng_transform_pose([1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1], PartFrame,
                     CamFrame, TimePoint, BlockerPoseInCamFrame),
  BlockerPoseInCamFrame = [_,_,_,BlkX,_,_,_,BlkY,_,_,_,BlkZ,_,_,_,_],
  BlkBearingX is atan2(BlkY, BlkX),
  BlkBearingY is atan2(BlkZ, BlkX),

  % debug
%   BlkXDeg is BlkBearingX /2 /pi * 360,
%   BlkYDeg is BlkBearingY /2 /pi * 360,

  abs(ObjBearingX - BlkBearingX) < 10/360 * 2 * pi,
  abs(ObjBearingY - BlkBearingY) < 10/360 * 2 * pi.



