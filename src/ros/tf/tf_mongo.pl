:- module(tf_mongo,
	[ tf_mng_store/3,
	  tf_mng_lookup/6,
	  tf_mng_trajectory/4,
	  tf_mng_range/6,
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
		[	collection(CollectionName),
			indices([
				['child_frame_id','header.stamp'],
				['header.stamp']
			])
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

%tf_mng_lookup(ObjFrame,QSince,QUntil,PoseData,FSince,FUntil) :-
%	mng_db_name(DB),
%	tf_db(DB, Name),
%	mng_cursor_create(DB,Name,Cursor),
%	mng_cursor_descending(Cursor,'header.stamp'),
%	mng_cursor_filter(Cursor, ['child_frame_id', string(ObjFrame)]),
%	mng_cursor_filter(Cursor, ['header.stamp', ['$lte', time(QUntil)]]),
%	%mng_cursor_filter(Cursor, ['header.stamp', ['$gte', time(Stamp0)]]),
%	setup_call_cleanup(
%		true,
%		tf_mng_lookup1(Cursor,QSince,QUntil,_,PoseData,FSince,FUntil),
%		mng_cursor_destroy(Cursor)
%	).

tf_mng_range(QSince,QUntil,ObjFrame,PoseData,FSince,FUntil) :-
	mng_db_name(DB),
	tf_db(DB, Name),
	mng_cursor_create(DB,Name,Cursor),
	mng_cursor_filter(Cursor, ['header.stamp', ['$lt', time(QUntil)]]),
	mng_cursor_filter(Cursor, ['header.stamp', ['$gte', time(QSince)]]),
	mng_cursor_descending(Cursor,'header.stamp'),
	setup_call_cleanup(
		true,
		tf_mng_lookup1(Cursor,QSince,QUntil,ObjFrame,PoseData,FSince,FUntil),
		mng_cursor_destroy(Cursor)
	),
	% TODO: read frame name
	true.

tf_mng_lookup1(Cursor,MinStamp,MaxStamp,ObjFrame,PoseData,FSince,FUntil) :-
	mng_cursor_next(Cursor,First),
	mng_get_dict(header,First,Header),
	mng_get_dict(stamp,Header,double(FirstStamp)),
	( FirstStamp > MaxStamp
	->	( FirstUntil=FirstStamp,
	      mng_cursor_next(Cursor,Doc)
		)
	;	( FirstUntil='Infinity',
		  Doc=First
		)
	),
	tf_mng_lookup2(Cursor,Doc,MinStamp,FirstUntil,ObjFrame,PoseData,FSince,FUntil).

tf_mng_lookup2(Cursor,Next,MinStamp,LastStamp,ObjFrame,PoseData,FSince,FUntil) :-
	tf_mng_doc_pose(Next,_,Stamp,PoseData0),
	(	( PoseData=PoseData0,
		  FSince=Stamp,
		  FUntil=LastStamp,
		  mng_get_dict(child_frame_id,Next,ObjFrame)
		)
	;	( Stamp > MinStamp,
		  mng_cursor_next(Cursor,X),
		  tf_mng_lookup2(Cursor,X,MinStamp,Stamp,ObjFrame,PoseData,FSince,FUntil)
		)
	).

%%
% Convert mongo document to pose term.
%
tf_mng_doc_pose(Doc,ObjFrame,Time,[ParentFrame,[TX,TY,TZ],[QX,QY,QZ,QW]]) :-
	get_dict(child_frame_id,Doc,
		string(ObjFrame)
	),
	get_dict(header,Doc,[
		_,
		stamp-double(Time),
		frame_id-string(ParentFrame)
	]),
	get_dict(transform,Doc,[
		translation-[
			x-double(TX),
			y-double(TY),
			z-double(TZ)
		],
		rotation-[
			x-double(QX),
			y-double(QY),
			z-double(QZ),
			w-double(QW)
		]
	]).

