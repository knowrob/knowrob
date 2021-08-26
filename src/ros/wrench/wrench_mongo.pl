:- module(wrench_mongo,
	[ wrench_mng_store/3,       % defined in wrench.cpp
	  wrench_mng_lookup/6,
	  wrench_mng_trajectory/4,
	  wrench_mng_drop/0
	]).

:- use_foreign_library('libwrench_knowrob.so').

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('db/mongo/client')).

% register wrench_raw/8 as a fluent predicate
% FIXME: need to update collection name if collection prefix is changed
:- mng_get_db(_DB, CollectionName, 'wrench'),
   mongolog_add_fluent(wrench_raw,
	   	% predicate arguments
		[	+'header.frame_id'          % +Frame
		,	-'wrench.force.x'           % -FX
		,	-'wrench.force.y'           % -FY
		,	-'wrench.force.z'           % -FZ
		,	-'wrench.torque.x'          % -TX
		,	-'wrench.torque.y'          % -TY
		,	-'wrench.torque.z'          % -TZ
		],
   		% time field
   		'header.stamp',
		% options
		[	collection(CollectionName)
		]).

% add *wrench* as a mongolog command such that it can be
% used in mongolog queries.
:- mongolog:add_command(wrench).

% wrench/3
lang_query:step_expand(
		wrench(Frame, [FX,FY,FZ], [TX,TY,TZ]),
		wrench_raw(Frame, FX, FY, FZ, TX, TY, TZ)).

lang_query:step_expand(
		assert(wrench(Frame, [FX,FY,FZ], [TX,TY,TZ])),
		assert(wrench_raw(Frame, FX, FY, FZ, TX, TY, TZ))).

% wrench/2
lang_query:step_expand(
		wrench(Frame, [[FX,FY,FZ], [TX,TY,TZ]]),
		wrench_raw(Frame, FX, FY, FZ, TX, TY, TZ)).

lang_query:step_expand(
		assert(wrench(Frame, [[FX,FY,FZ], [TX,TY,TZ]])),
		assert(wrench_raw(Frame, FX, FY, FZ, TX, TY, TZ))).

%%
wrench_db(DB, Name) :-
	mng_get_db(DB, Name, 'wrench').

%% wrench_mng_drop is det.
%
% Drops all documents in the wrench database collection.
%
wrench_mng_drop :-
	wrench_db(DB, Name),
	mng_drop(DB,Name).

%% wrench_mng_lookup(+ObjFrame, +QuerySince, +QueryUntil, -PoseData, -FactSince, -FactUntil) is nondet.
%
% Retrieve all transforms of frame within some time interval.
%
wrench_mng_lookup(
		ObjFrame, Query_Since, Query_Until,
		WrenchData, Fact_Since, Fact_Until) :-
	% create a query scope
	time_scope(=<(Query_Since), >=(Query_Until), QueryScope),
	mongolog_call(
		wrench(ObjFrame, WrenchData),
		[ scope(QueryScope),
		  user_vars([['v_scope',FactScope]])
		]),
	% read values from FactScope
	time_scope(double(Fact_Since), double(Fact_Until), FactScope).

%% wrench_mng_lookup(+ObjFrame, -PoseData) is nondet.
%
% Retrieve the latest transform of a frame.
%
wrench_mng_lookup(ObjFrame, WrenchData) :-
	get_time(Now),
	wrench_mng_lookup(ObjFrame, Now, Now, WrenchData, _, _).

%% wrench_mng_store(+ObjFrame, +PoseData, +Stamp) is det.
%
% Store a transform in mongo DB.
%
% Defined in wrench.cpp


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % trajectories

%% wrench_mng_trajectory(+Obj, +Begin, +End, -Trajectory) is det.
%
% Read all transforms associated to a frame in given time interval.
%
wrench_mng_trajectory(Obj,Stamp0,Stamp1,Trajectory) :-
	rdf_split_url(_,ObjFrame,Obj),
	findall(Stamp-Data,
		( wrench_mng_lookup(ObjFrame,Stamp0,Stamp1,Data,Stamp,_),
		  Stamp >= Stamp0,
		  Stamp =< Stamp1
		),
		Trajectory0
	),
	reverse(Trajectory0,Trajectory).


%%
% Convert mongo document to wrench term.
%
wrench_mng_doc_pose(Doc,ObjFrame,Time,[[FX,FY,FZ],[TX,TY,TZ]]) :-
	get_dict(header,Doc,
		[ _, stamp-time(Time), frame_id-string(ObjFrame) ]
	),
	get_dict(transform,Doc,[
		translation-[ x-double(FX), y-double(FY), z-double(FZ) ],
		rotation-[ x-double(TX), y-double(TY), z-double(TZ) ]
	]).
