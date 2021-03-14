:- module(tf_plugin,
	[ tf_set_pose/3,
	  tf_get_pose/4,
	  tf_mem_set_pose/3,
	  tf_mem_get_pose/3,
  	  tf_mem_clear/0,
	  tf_republish_set_pose/2,
	  tf_republish_set_goal/2,
	  tf_republish_set_time/1,
	  tf_republish_set_progress/1,
	  tf_republish_set_loop/1,
	  tf_republish_set_realtime_factor/1,
	  tf_republish_clear/0,
	  tf_logger_enable/0,
	  tf_logger_disable/0
	]).

:- use_foreign_library('libtf_plugin.so').

:- use_module(library(settings)).
:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('utility/algebra'),
	[ transform_between/3, transform_multiply/3 ]).
:- use_module(library('lang/scope'),
	[ scope_intersect/3,
	  subscope_of/2,
	  time_scope/3,
	  time_scope_data/2,
	  current_scope/1
	]).
:- use_module('tf_mongo',
	[ tf_mng_lookup/6,
	  tf_mng_lookup_all/2
	]).

% define some settings
:- setting(use_logger, boolean, true,
	'Toggle whether TF messages are logged into the mongo DB.').

%%
:-	mng_db_name(DB),
	(	setting(tf_plugin:use_logger,false)
	->	true
	;	tf_logger_set_db_name(DB)
	).

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% TF REPUBLISHER
%%%%%%%%%%%%%%%%%%%%%%%

%%
tf_republish_set_goal(Time_min, Time_max) :-
	tf_db(DBName, CollectionName),
	( number(Time_min)
	->	Min is Time_min
	;	atom_number(Time_min,Min)
	),
	( number(Time_max)
	->	Max is Time_max
	;	atom_number(Time_max,Max)
	),
	% initialize poses to last pose before Min.
	% TODO: might be faster to do this in C++ code of republisher.
	tf_republish_load_transforms(Min),
	% start republishing range [Min,Max]
	tf_republish_set_goal(DBName, CollectionName, Min, Max).

%%
tf_republish_load_transforms(Time) :-
	tf_mng_lookup_all(Transforms, Time),
	forall(
	    (   member([Ref,Frame,Pos,Rot],Transforms),
	        % FIXME avoid this elsewhere
	        Ref \= Frame,
	        \+ atom_concat('/',Ref,Frame),
	        \+ atom_concat('/',Frame,Ref)
	    ),
		tf_republish_set_pose(Frame,[Ref,Pos,Rot])
	).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % is_at

%% is_at(?Object,-PoseData) is nondet.
%
%is_at(Obj,[RefFrame,Pos,Rot]) +>
%	fact_scope(FS),
%	{ tf_set_pose(Obj,[RefFrame,Pos,Rot],FS) },
%	notify(object_changed(Obj)).

is_at(Obj,[RefFrame,Pos,Rot]) :-
	current_scope(QS),
	tf_get_pose(Obj,[RefFrame,Pos,Rot],QS,_FS).

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
	time_scope(=(Since),=(Now),FS),
	% make sure there is an overlap with the query scope
	time_scope_data(QS,[QSince,QUntil]),
	mng_strip_operator(QSince, _, QSince0),
	mng_strip_operator(QUntil, _, QUntil0),
	time_scope(QSince0,=(QUntil0),=(QS0)),
	scope_intersect(FS,QS0,_),
	% skip other results in case the fact scope is a superscope of the query scope
	(  subscope_of(QS,FS)
	-> !
	;  true
	).

is_at_direct(ObjFrame,PoseData,QS,FS) :-
	time_scope_data(QS,[QSince,QUntil]),
	mng_strip_operator(QSince, _, QSince0),
	mng_strip_operator(QUntil, _, QUntil0),
	tf_mng_lookup(ObjFrame,QSince0,QUntil0,PoseData,FSince,FUntil),
	time_scope(=(FSince),=(FUntil),FS).
  
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

