:- module(tf_mongo,
	[ tf_mng_store/3,
	  tf_mng_lookup/6,
	  tf_mng_trajectory/4,
	  tf_mng_drop/0
	]).

:- use_foreign_library('libtf_plugin.so').

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('db/mongo/client')).

% register tf_raw/9 as a fluent predicate
:- mng_get_db(_DB, CollectionName, 'tf'),
   mongolog_add_fluent(tf_raw,
	   	% predicate arguments
		[	+'child_frame_id'           % +ChildFrame
		,	-'header.frame_id'          % -ParentFrame
		,	-'transform.translation.x'  % -X
		,	-'transform.translation.y'  % -Y
		,	-'transform.translation.z'  % -Z
		,	-'transform.rotation.x'     % -QX
		,	-'transform.rotation.y'     % -QY
		,	-'transform.rotation.z'     % -QZ
		,	-'transform.rotation.w'     % -QW
		],
   		% time field
   		'header.stamp',
		% options
		[	collection(CollectionName)
		]).

%%
:- mongolog:add_command(tf).

%%
tf_db(DB, Name) :- 
	mng_get_db(DB, Name, 'tf').

%%
tf_mng_drop :-
	tf_db(DB, Name),
	mng_drop(DB,Name).

%%
mongolog:step_expand(
		tf(ChildFrame, ParentFrame, [X,Y,Z], [QX,QY,QZ,QW]),
		tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW),
		_Context).

mongolog:step_expand(
		tf(ChildFrame, [ParentFrame, [X,Y,Z], [QX,QY,QZ,QW]]),
		tf_raw(ChildFrame, ParentFrame, X, Y, Z, QX, QY, QZ, QW),
		_Context).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % trajectories

%%
%
tf_mng_trajectory(Obj,Stamp0,Stamp1,Trajectory) :-
	rdf_split_url(_,ObjFrame,Obj),
	findall(Stamp-Data,
		( tf_mng_lookup(ObjFrame,Stamp0,Stamp1,Data,Stamp,_),
		  Stamp >= Stamp0,
		  Stamp =< Stamp1
		),
		Trajectory0
	),
	reverse(Trajectory0,Trajectory).

tf_mng_lookup(
		ObjFrame,
		Query_Since, Query_Until,
		PoseData,
		Fact_Since, Fact_Until) :-
	time_scope(
		=<(Query_Since),
		>=(Query_Until), QueryScope),
	mongolog_call(
		tf(ObjFrame, PoseData),
		[ mode(ask),
		  scope(QueryScope),
		  user_vars([['v_scope',FactScope]])
		]),
	time_scope(
		double(Fact_Since),
		double(Fact_Until),
		FactScope).
