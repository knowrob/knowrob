:- module(lang_is_at,
    [ is_at(r,r),
      op(1000, xfx, is_at)
    ]).
/** <module> The *is_at* predicate.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('utility/algebra'),
    [ transform_between/3,
      transform_multiply/3
    ]).
:- use_module(library('model/DUL/Region'),
    [ has_region/2
    ]).
:- use_module(library('model/EASE/OBJ'),
    [ object_map/3,
      object_localization/2
    ]).
:- use_module(library('comm/notify'),
    [ notify/1
    ]).

%% is_at(?Object,?Pose) is nondet.
%
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
is_at1(_ObjFrame,Pose,Pose) ?> { ! }.

is_at1(ObjFrame,[RefFrame,T0,Q0],RelPose0) ?>
  % get object frame in map
  world_pose1(ObjFrame,WorldFrame,RelPose0,[WorldFrame,T0,Q0]),
  % get requested frame in map
  world_pose(RefFrame,WorldFrame,[WorldFrame,T1,Q1]),
  % compute transform from requested frame
  % to object frame
  { transform_between(
        [MapFrame,RefFrame,T1,Q1],
        [MapFrame,ObjFrame,T0,Q0],
        [RefFrame,ObjFrame,T,Q]) }.

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

world_pose1(_ObjFrame, MapFrame,
        [MapFrame|Rest],
        [MapFrame|Rest]) ?> { true }.
