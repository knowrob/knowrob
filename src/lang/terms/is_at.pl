:- module(lang_is_at,
    [ is_at(r,r),
      op(1000, xfx, is_at)
    ]).
/** <module> The *is_at* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('utility/algebra'),
        [ transform_between/3, transform_multiply/3 ]).
:- use_module(library('model/DUL/Region'),
        [ has_region/2 ]).
:- use_module(library('model/EASE/OBJ'),
        [ object_map/3, object_localization/2, object_frame_name/2 ]).
:- use_module(library('lang/scopes/temporal'),
        [ time_scope_data/2 ]).
:- use_module(library('comm/notify'),
        [ notify/1 ]).

:- dynamic pose_data_/3.

%% is_at(?Object,?Pose) is nondet.
%
%
is_at(Obj,Pose) ?>
  time_scope_data([Since,_Until]),
  { pose_data_(Obj,Pose,PoseStamp) },
  { once(ground(Since); get_time(Since)) },
  % NOTE: here we assume that pose_data_ is always up-to-date!
  % FIXME: this seems dangerous, who takes care of this?
  %           - e.g. in case of robot state publisher no tell() would be
  %             called. so what is the difference between perceived objects
  %             and the robot?
  { Since >= PoseStamp },
  { ! }.
  
is_at(Obj,PoseQuery) ?>
  object_localization(Obj,Localization),
  % TODO should region be required? or also allowed to set the
  % pose data on the quality/object ?
  has_region(Localization,SpaceRegion),
  % TODO: could be that DB interface can directly give
  %       transformation in requested frame.
  holds(SpaceRegion,knowrob:pose,PoseComp),
  is_at1(Obj,PoseQuery,PoseComp).

is_at(Obj,[Frame,Position,Quaternion]) +>
  time_scope_data([Since,_Until]),
  { once(ground(Since); get_time(Since)) },
  { pose_update_(Obj,[Frame,Position,Quaternion],Since) },
  { ! }.
  
%%
is_at1(Obj,Pose,Pose) ?> { ! }.

is_at1(Obj,[RefFrame,T0,Q0],RelPose0) ?>
  % get object frame in map
  world_pose1(ObjFrame,WorldFrame,RelPose0,[MapFrame,T0,Q0]),
  % get requested frame in map
  world_pose(RefFrame,MapFrame,[MapFrame,T1,Q1]),
  % compute transform from requested frame
  % to object frame
  { transform_between(
        [MapFrame,RefFrame,T1,Q1],
        [MapFrame,ObjFrame,T0,Q0],
        [RefFrame,ObjFrame,T,Q]) }.

%%
world_pose(ObjFrame,MapFrame,MapPose) ?>
  % find object with frame
  { object_frame_name(Obj,ObjFrame) },
  is_at(Obj,ObjPose),
  world_pose1(ObjFrame,MapFrame,ObjPose,MapPose).

world_pose1(ObjFrame, MapFrame,
        [ParentFrame,T0,Q0],
        [MapFrame,T,Q]) ?>
  world_pose(ParentFrame, MapFrame, [MapFrame,T1,Q1]),
  { ! },
  { transform_multiply(
        [MapFrame,ParentFrame,T1,Q1],
        [ParentFrame,ObjFrame,T0,Q0],
        [MapFrame,ObjFrame,T,Q]) }.

world_pose1(_ObjFrame, MapFrame,
        [MapFrame|Rest],
        [MapFrame|Rest]) ?> { true }.

%%
pose_update_(Object,Pose,Since) :-
  retractall( pose_data_(Object,_,_) ),
  asserta(    pose_data_(Object,Pose,Since) ),
  notify(object_changed(Object)).
