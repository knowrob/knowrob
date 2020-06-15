:- module(lang_is_at,
	[ is_at(r,t)  % ?Object, -PoseData
	]).
/** <module> The is_at predicate.

@author Daniel Beßler
@license BSD
*/

:- op(1000, xfx, user:is_at).

:- use_module(library('utility/algebra'),
	[ transform_between/3,
	  transform_multiply/3
	]).
:- use_module(library('db/scope'),
	[ scope_intersect/3,
	  subscope_of/2
	]).
:- use_module(library('lang/query'),
	[ ask/2 ]).
:- use_module(library('lang/scopes/temporal'),
	[ time_scope/5,
	  time_scope_data/2
	]).
:- use_module(library('lang/terms/holds'),
	[ holds/3 ]).
:- use_module(library('model/DUL/Region'),
	[ has_region/2 ]).
:- use_module(library('model/EASE/OBJ'),
	[ object_localization/2 ]).

:- dynamic pose_data_/3.

%% is_at(?Object,-PoseData) is nondet.
%
% Query the pose where the object is located.
% PoseData is a term `[Frame,[X,Y,Z],[QX,QY,QZ,QW]]`.
%
% @param Object an object instance
% @param PoseData the position data
%
is_at(Obj,PoseData) +>
	fact_scope(FS),
	{	time_scope_data(FS,[Since,_Until]),
		( ground(Since) -> true ; get_time(Since) )
	},
	{ set_current_pose(Obj,PoseData,Since) },
	notify(object_changed(Obj)).

is_at(Obj,PoseQuery) ?>
  { writeln(is_at) },
	% read query scope
	query_scope(QS),
	% do the query
	{ is_at_(Obj,PoseQuery,QS,FS) },
	% set fact scope
	fact_scope(FS).

%%
is_at_(Obj,PoseQuery,QS,FS) :-
	% lookup direct position data without doing transformations
	is_at_direct(Obj,PoseData,QS,FS0),
	% then try to unify the query
	(  query_unify_(PoseQuery,PoseData)
	-> FS=FS0
	% else try to compute the position in the requested frame
	;  is_at_indirect(Obj,PoseQuery,PoseData,QS,FS1)
	-> scope_intersect(FS0,FS1,FS)
	;  fail
	).
  
%%
is_at_direct(Obj,PoseData,QS,FS) :-
	% get local pose data and scope
	current_pose_data(Obj,PoseData,FS),
	% make sure there is an overlap with the query scope
	scope_intersect(FS,QS,_),
	% skip other results in case the fact scope is a superscope of the query scope
	(  subscope_of(QS,FS)
	-> !
	;  true
	).

is_at_direct(Obj,PoseData,QS,FS) :-
	% get pose data from RDF model
	object_localization(Obj,Localization),
	ask([ has_region(Localization,SpaceRegion),
          holds(SpaceRegion,knowrob:hasPoseData,PoseData)
		],
		[[],QS]->FS
	).
  
%%
is_at_indirect(Obj,
		[RefFrame_query,T_query,Q_query],
		[RefFrame_comp,T_comp,Q_comp],
		QS, FS) :-
	object_frame_(Obj,ObjFrame),
	% get object frame in world frame
	world_pose1(ObjFrame,WorldFrame,
		[RefFrame_comp,T_comp,Q_comp],
		[WorldFrame,T0,Q0],
		QS, FS0),
	% get requested frame in world frame
	world_pose(RefFrame_query,WorldFrame,
		[WorldFrame,T1,Q1],
		QS, FS1),
	% 
	scope_intersect(FS0,FS1,FS),
	% compute transform from requested frame
	% to object frame
	transform_between(
		[MapFrame,       RefFrame_query, T1,Q1],
		[MapFrame,       ObjFrame,       T0,Q0],
		[RefFrame_query, ObjFrame,       T_query,Q_query]
	).

%%
world_pose(ObjFrame,MapFrame,MapPose,QS,FS) :-
	object_frame_(Obj,ObjFrame),
	is_at_direct(Obj,ObjPose,QS,FS0),
	world_pose1(ObjFrame,MapFrame,ObjPose,MapPose,QS,FS1),
	scope_intersect(FS0,FS1,FS).

world_pose1(ObjFrame, MapFrame,
		[ParentFrame,T0,Q0],
		[MapFrame,T,Q],
		QS, FS) :-
	world_pose(ParentFrame, MapFrame, [MapFrame,T1,Q1], QS, FS),
	!,
	transform_multiply(
		[MapFrame,ParentFrame,T1,Q1],
		[ParentFrame,ObjFrame,T0,Q0],
		[MapFrame,ObjFrame,T,Q]
	).

world_pose1(_, Map, [Map|Rest], [Map|Rest], _, _).

%%
query_unify_(Query,Fact) :-
	Query=Fact.

%%
% FIXME: should be somewhere else
object_frame_(Obj,ObjFrame) :-
	ground(Obj),!,
	object_localization(Obj,Localization),
	has_region(Localization,SpaceRegion),
	holds(SpaceRegion,'http://knowrob.org/kb/knowrob.owl#frameName',ObjFrame).

object_frame_(Obj,ObjFrame) :-
	holds(SpaceRegion,'http://knowrob.org/kb/knowrob.owl#frameName',ObjFrame),
	has_region(Localization,SpaceRegion),
	object_localization(Obj,Localization).

%%
current_pose_data(Obj,PoseData,TimeScope) :-
	pose_data_(Obj,PoseData,Since),
	get_time(Now),
	time_scope(Since,=,Now,=,TimeScope).

%%
set_current_pose(Obj,Pose) :-
	get_time(Now),
	set_current_pose(Obj,Pose,Now).

set_current_pose(Obj,_,Stamp) :-
	pose_data_(Obj,_,LatestStamp),
	%LatestStamp >= Stamp, !.
	LatestStamp > Stamp, !.

set_current_pose(Obj,Pose,Stamp) :-
	retractall(pose_data_(Obj,_,_)),
	asserta(pose_data_(Obj,Pose,Stamp)).
