:- module(lang_is_at,
    [ is_at(r,r)  % ?Object, -PoseData
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
:- use_module(library('model/DUL/Region'),
    [ has_region/2 ]).
:- use_module(library('model/EASE/OBJ'),
    [ object_localization/2 ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

%% is_at(?Object,-PoseData) is nondet.
%
% Query the pose where the object is located.
% PoseData is a term `[Frame,[X,Y,Z],[QX,QY,QZ,QW]]`.
%
% @param Object an object instance
% @param PoseData the position data
%
is_at(Obj,PoseQuery) ?>
  object_localization(Obj,Localization),
  has_region(Localization,SpaceRegion),
  holds(SpaceRegion,knowrob:frameName,ObjFrame),
  % TODO: could be that DB interface can directly give
  %       transformation in requested frame.
  holds(SpaceRegion,knowrob:hasPoseData,PoseComp),
  is_at1(ObjFrame,PoseQuery,PoseComp).

%is_at(Obj,PoseQuery) ?>
  %holds(Obj,knowrob:hasPoseData,PoseComp),
  %is_at1(Obj,PoseQuery,PoseComp).

is_at(Obj,PoseData) +>
  { object_localization(Obj,Localization) },
  { has_region(Localization,SpaceRegion) },
  holds(SpaceRegion,knowrob:hasPoseData,update(PoseData)),
  notify(object_changed(Obj)).
  
%%
is_at1(_,Pose,Pose) ?> { ! }.

is_at1(ObjFrame,[RefFrame_query,T_query,Q_query],
                [RefFrame_comp,T_comp,Q_comp]) ?>
  % get object frame in map
  world_pose1(ObjFrame,WorldFrame,
    [RefFrame_comp,T_comp,Q_comp],
    [WorldFrame,T0,Q0]),
  % get requested frame in map
  world_pose(RefFrame_query,WorldFrame,
    [WorldFrame,T1,Q1]),
  % compute transform from requested frame
  % to object frame
  { transform_between(
        [MapFrame,       RefFrame_query, T1,Q1],
        [MapFrame,       ObjFrame,       T0,Q0],
        [RefFrame_query, ObjFrame,       T_query,Q_query]) }.

%%
world_pose(ObjFrame,MapFrame,MapPose) ?>
  holds(SpaceRegion,knowrob:frameName,ObjFrame),
  holds(SpaceRegion,knowrob:hasPoseData,ObjPose),
  world_pose1(ObjFrame,MapFrame,ObjPose,MapPose),
  { ! }.

world_pose1(ObjFrame, MapFrame,
        [ParentFrame,T0,Q0],
        [MapFrame,T,Q]) ?>
  world_pose(ParentFrame, MapFrame, [MapFrame,T1,Q1]),
  { ! },
  { transform_multiply(
        [MapFrame,ParentFrame,T1,Q1],
        [ParentFrame,ObjFrame,T0,Q0],
        [MapFrame,ObjFrame,T,Q]) }.

world_pose1(_, MapFrame,
        [MapFrame|Rest],
        [MapFrame|Rest]) ?> { true }.
