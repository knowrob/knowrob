:- module(wrench,
	[ wrench_set/3,
	  wrench_get/4,
	  wrench_mem_set/3,			% defined in wrench.cpp
	  wrench_mem_get/3, 		% defined in wrench.cpp
  	  wrench_mem_clear/0,		% defined in wrench.cpp
	  wrench_logger_enable/0,	% defined in wrench.cpp
	  wrench_logger_disable/0% defined in wrench.cpp
	]).

:- use_foreign_library('libwrench_knowrob.so').

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
:- use_module(library('lang/computable'),
	[ add_computable_predicate/2 ]).
:- use_module('wrench_mongo',
	[ wrench_mng_lookup/6 ]).

%%
:-	mng_db_name(DB), tf_logger_set_db_name(DB).

%% wrench_set(+Obj, +Data, +Scope) is det.
%
% Update the wrench acting on an object.
% The wrench is updated in local memory, but also
% written into mongo database.
%
wrench_set(Obj,WrenchData,FS) :-
	rdf_split_url(_,ObjFrame,Obj),
	time_scope_data(FS,[Since,_Until]),
	wrench_mem_set(ObjFrame,WrenchData,Since),
	wrench_mng_store(ObjFrame,WrenchData,Since).

%%
wrench_get(Obj,[Force,Torque]) :-
	current_scope(QS),
	wrench_get(Obj,[Force,Torque],QS,_FS).

%% wrench_get(+Obj, ?WrenchQuery, +QueryScope, +FactScope) is semidet.
%
% Retrieve the wrench acting on an object.
%
wrench_get(Obj,WrenchQuery,QS,FS) :-
	rdf_split_url(_,ObjFrame,Obj),
	% lookup direct position data without doing transformations
	has_wrench_direct(ObjFrame,WrenchData,QS,FS0),
	% then try to unify the query
	(  WrenchQuery=WrenchData
	-> FS=FS0
	% else try to compute the position in the requested frame
	;  has_wrench_indirect(ObjFrame,WrenchQuery,WrenchData,QS,FS1)
	-> scope_intersect(FS0,FS1,FS)
	;  fail
	).

%% has_wrench_direct(+ObjFrame, ?WrenchData, +QS, ?FS)
%
% Retrieve the wrench acting on an object from the memory without transforming it.
%
has_wrench_direct(ObjFrame,WrenchData,QS,FS) :-
	% get local wrench data and scope
	wrench_mem_get(ObjFrame,WrenchData,Since),
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

%% has_wrench_direct(+ObjFrame, ?WrenchData, +QS, ?FS)
%
% Retrieve the wrench acting on an object from the mongoDB database without transforming it.
%
has_wrench_direct(ObjFrame,WrenchData,QS,FS) :-
	time_scope_data(QS,[QSince,QUntil]),
	mng_strip_operator(QSince, _, QSince0),
	mng_strip_operator(QUntil, _, QUntil0),
	wrench_mng_lookup(ObjFrame,QSince0,QUntil0,WrenchData,FSince,FUntil),
	time_scope(=(FSince),=(FUntil),FS).

%%%%% Wrench transformations (not supported yet)
%%%%% This is where lever/gravity compensation would be done

% has_wrench_indirect(ObjFrame,WrenchQuery,DirectWrench,QS,FS) :-
% 	% get wrench in world frame
% 	world_wrench1(ObjFrame,_,DirectWrench,WorldWrench,QS,FS0),
% 	has_wrench_indirect1(ObjFrame,WrenchQuery,WorldWrench,FS0,QS,FS).

% has_wrench_indirect1(_ObjFrame,
% 		[WorldFrame,T_query,Q_query],
% 		[WorldFrame,T0,Q0],
% 		FS, _QS, FS) :-
% 	!,
% 	% pose was requested in world frame
% 	T_query = T0,
% 	Q_query = Q0.

% has_wrench_indirect1(ObjFrame,
% 		[RefFrame_query,T_query,Q_query],
% 		[WorldFrame,T0,Q0],
% 		FS0, QS, FS) :-
% 	% get requested frame in world frame
% 	world_pose(RefFrame_query,WorldFrame,
% 		[WorldFrame,T1,Q1],
% 		QS, FS1),
% 	%
% 	scope_intersect(FS0,FS1,FS),
% 	% compute transform from requested frame
% 	% to object frame
% 	transform_between(
% 		[WorldFrame,     ObjFrame,       T0,Q0],
% 		[WorldFrame,     RefFrame_query, T1,Q1],
% 		[RefFrame_query, ObjFrame,       T_query,Q_query]
% 	).

has_wrench_indirect(ObjFrame, WrenchQuery, DirectWrench, QS, FS) :- fail.

% world_pose(ObjFrame,WorldFrame,MapPose,QS,FS) :-
% 	has_wrench_direct(ObjFrame,ObjPose,QS,FS0),
% 	world_pose1(ObjFrame,WorldFrame,ObjPose,MapPose,QS,FS1),
% 	scope_intersect(FS0,FS1,FS).

% world_pose1(ObjFrame, WorldFrame,
% 		[ParentFrame,T0,Q0],
% 		[WorldFrame,T,Q],
% 		QS, FS) :-
% 	world_pose(ParentFrame, WorldFrame, [WorldFrame,T1,Q1], QS, FS),
% 	!,
% 	transform_multiply(
% 		[WorldFrame,ParentFrame,T1,Q1],
% 		[ParentFrame,ObjFrame,T0,Q0],
% 		[WorldFrame,ObjFrame,T,Q]
% 	).

% world_pose1(_, WorldFrame,
% 	[WorldFrame|Rest],
% 	[WorldFrame|Rest], _, _).

