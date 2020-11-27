:- module(tf_plugin,
	[ tf_set_pose/3,
	  tf_get_pose/4,
	  tf_get_trajectory/4,
	  tf_mem_set_pose/3,
	  tf_mem_get_pose/3,
	  tf_mng_store/3,
	  tf_mng_lookup/6,
	  tf_mng_range/6,
	  tf_mng_whipe/0,
	  tf_mng_remember/1,
	  tf_mng_memorize/1,
	  tf_republish_set_pose/2,
	  tf_republish_set_goal/2,
	  tf_republish_set_loop/1,
	  tf_republish_set_realtime_factor/1,
	  tf_logger_enable/0,
	  tf_logger_disable/0
	]).

:- use_foreign_library('libtf_plugin.so').

:- use_module(library(settings)).
:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('utility/algebra'),
	[ transform_between/3,
	  transform_interpolate/4,
	  transform_multiply/3
	]).
:- use_module(library('utility/filesystem'),
	[ path_concat/3
	]).
:- use_module(library('db/scope'),
	[ scope_intersect/3,
	  subscope_of/2
	]).
:- use_module(library('lang/scopes/temporal'),
	[ time_scope/5,
	  time_scope_data/2
	]).
:- use_module(library('db/mongo/client')).

% define some settings
:- setting(use_logger, boolean, true,
	'Toggle whether TF messages are logged into the mongo DB.').

tf_db(DB, Name) :- 
	mng_get_db(DB, Name, 'tf').

%%
tf_republish_set_goal(Time_min, Time_max) :-
	tf_db(DBName, CollectionName),
	tf_republish_set_goal(DBName, CollectionName, Time_min, Time_max).

%%
tf_republish_load_transforms(Time) :-
	tf_tree:lookup_transforms_(Time,Transforms),
	forall(
	    (   member([Ref,Frame,Pos,Rot],Transforms),
	        % FIXME avoid this elsewhere
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf_plugin:tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	).

%%
tf_mng_whipe :-
	mng_db_name(DB),
	tf_db(DB, Name),
	mng_drop(DB,Name).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % import/export

%%
tf_mng_remember(Directory) :-
	mng_db_name(DB),
	tf_mng_whipe,
	tf_logger_set_db_name(DB),
	path_concat(Directory,ros_tf,TFDir),
	mng_restore(DB,TFDir),
	mng_index_create(DB,tf,['child_frame_id','header.stamp']).

%%
tf_mng_memorize(Directory) :-
	%mng_db_name(DB),
	path_concat(Directory,ros_tf,TFDir),
	mng_export_collection(tf,TFDir).

%%
lang_export:remember_hook(Directory) :- tf_mng_remember(Directory).
lang_export:memorize_hook(Directory) :- tf_mng_memorize(Directory).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % is_at

%% is_at(?Object,-PoseData) is nondet.
%
is_at(Obj,[RefFrame,Pos,Rot]) +>
	fact_scope(FS),
	{ tf_set_pose(Obj,[RefFrame,Pos,Rot],FS) },
	notify(object_changed(Obj)).

is_at(Obj,[RefFrame,Pos,Rot]) ?>
	query_scope(QS),
	{ tf_get_pose(Obj,[RefFrame,Pos,Rot],QS,FS) },
	fact_scope(FS).

%%
tf_set_pose(Obj,PoseData,FS) :-
	rdf_split_url(_,ObjFrame,Obj),
	time_scope_data(FS,[Since,_Until]),
	tf_mem_set_pose(ObjFrame,PoseData,Since),
	tf_mng_store(ObjFrame,PoseData,Since).

%%
tf_get_pose(Obj,PoseQuery,QS,FS) :-
	rdf_split_url(_,ObjFrame,Obj),
	% lookup direct position data without doing transformations
	is_at_direct(ObjFrame,PoseData,QS,FS0),
	% then try to unify the query
	(  PoseQuery=PoseData
	-> FS=FS0
	% else try to compute the position in the requested frame
	;  is_at_indirect(ObjFrame,PoseQuery,PoseData,QS,FS1)
	-> scope_intersect(FS0,FS1,FS)
	;  fail
	).
  
%%
is_at_direct(ObjFrame,PoseData,QS,FS) :-
	% get local pose data and scope
	tf_mem_get_pose(ObjFrame,PoseData,Since),
	get_time(Now),
	time_scope(Since,=,Now,=,FS),
	% make sure there is an overlap with the query scope
	time_scope_data(QS,[QSince,QUntil]),
	strip_operator_(QSince,QSince0),
	strip_operator_(QUntil,QUntil0),
	time_scope(QSince0,=,QUntil0,=,QS0),
	scope_intersect(FS,QS0,_),
	% skip other results in case the fact scope is a superscope of the query scope
	(  subscope_of(QS,FS)
	-> !
	;  true
	).

is_at_direct(ObjFrame,PoseData,QS,FS) :-
	time_scope_data(QS,[QSince,QUntil]),
	strip_operator_(QSince,QSince0),
	strip_operator_(QUntil,QUntil0),
	tf_mng_lookup(ObjFrame,QSince0,QUntil0,PoseData,FSince,FUntil),
	time_scope(FSince,=,FUntil,=,FS).

% FIXME redundant
strip_operator_( <(X),X) :- !.
strip_operator_( >(X),X) :- !.
strip_operator_(=<(X),X) :- !.
strip_operator_(>=(X),X) :- !.
strip_operator_( =(X),X) :- !.
strip_operator_( X,X) :- !.
  
%%
is_at_indirect(ObjFrame,PoseQuery,DirectPose,QS,FS) :-
	% get object pose in world frame
	world_pose1(ObjFrame,_,DirectPose,WorldPose,QS,FS0),
	is_at_indirect1(ObjFrame,PoseQuery,WorldPose,FS0,QS,FS).

is_at_indirect1(_ObjFrame,
		[WorldFrame,T_query,Q_query],
		[WorldFrame,T0,Q0],
		FS, _QS, FS) :-
	!,
	% pose was requested in world frame
	T_query = T0,
	Q_query = Q0.
	
is_at_indirect1(ObjFrame,
		[RefFrame_query,T_query,Q_query],
		[WorldFrame,T0,Q0],
		FS0, QS, FS) :-
	% get requested frame in world frame
	world_pose(RefFrame_query,WorldFrame,
		[WorldFrame,T1,Q1],
		QS, FS1),
	% 
	scope_intersect(FS0,FS1,FS),
	% compute transform from requested frame
	% to object frame
	transform_between(
		[WorldFrame,     ObjFrame,       T0,Q0],
		[WorldFrame,     RefFrame_query, T1,Q1],
		[RefFrame_query, ObjFrame,       T_query,Q_query]
	).

%%
world_pose(ObjFrame,WorldFrame,MapPose,QS,FS) :-
	is_at_direct(ObjFrame,ObjPose,QS,FS0),
	world_pose1(ObjFrame,WorldFrame,ObjPose,MapPose,QS,FS1),
	scope_intersect(FS0,FS1,FS).

world_pose1(ObjFrame, WorldFrame,
		[ParentFrame,T0,Q0],
		[WorldFrame,T,Q],
		QS, FS) :-
	world_pose(ParentFrame, WorldFrame, [WorldFrame,T1,Q1], QS, FS),
	!,
	transform_multiply(
		[WorldFrame,ParentFrame,T1,Q1],
		[ParentFrame,ObjFrame,T0,Q0],
		[WorldFrame,ObjFrame,T,Q]
	).

world_pose1(_, WorldFrame,
	[WorldFrame|Rest],
	[WorldFrame|Rest], _, _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % trajectories

%%
%
tf_get_trajectory(Obj,Stamp0,Stamp1,Trajectory) :-
	rdf_split_url(_,ObjFrame,Obj),
	findall(Stamp-Data,
		( tf_mng_lookup(ObjFrame,Stamp0,Stamp1,Data,Stamp,_),
		  Stamp >= Stamp0,
		  Stamp =< Stamp1
		),
		Trajectory0
	),
	reverse(Trajectory0,Trajectory).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % mongo backend
% %
% % TODO: maybe better do the lookup in C++ for performance reasons? 
% %

%%
%
tf_mng_init :-
	mng_db_name(DB),
	tf_db(DB, Name),
	mng_index_create(DB,Name,[+'child_frame_id',-'header.stamp']),
	mng_index_create(DB,Name,[+'header.stamp']),
	%%
	(	setting(tf_plugin:use_logger,false)
	->	true
	;	tf_logger_set_db_name(DB)
	).
%%
:- tf_mng_init.

%%
%
tf_mng_lookup(ObjFrame,QSince,QUntil,PoseData,FSince,FUntil) :-
	mng_db_name(DB),
	tf_db(DB, Name),
	mng_cursor_create(DB,Name,Cursor),
	mng_cursor_descending(Cursor,'header.stamp'),
	mng_cursor_filter(Cursor, ['child_frame_id', string(ObjFrame)]),
	mng_cursor_filter(Cursor, ['header.stamp', ['$lte', time(QUntil)]]),
	%mng_cursor_filter(Cursor, ['header.stamp', ['$gte', time(Stamp0)]]),
	setup_call_cleanup(
		true,
		tf_mng_lookup1(Cursor,QSince,QUntil,_,PoseData,FSince,FUntil),
		mng_cursor_destroy(Cursor)
	).

%%
%
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
