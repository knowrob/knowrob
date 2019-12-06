/*
  Copyright (C) 2011-2014 Moritz Tenorth
  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(knowrob_objects,
    [
      object_assert/3,
      object_lifetime/2,
      object_pose/2,
      object_pose/3,
      current_object_pose/2,
      current_object_pose_stamp/2,
      object_pose_update/2,
      object_pose_update/3,
      object_trajectory/4,
      object_distance/3,
      object_frame_name/2,
      object_state/2,
      object_state/3,
      object_color/2,
      object_quality/3,
      object_feature/2,
      object_disposition/2,
      object_disposition/3,
      object_localization/2,
      object_dimensions/4,
      object_mesh_path/2,
      object_assert_dimensions/4,
      object_assert_color/2,
      object_assert_frame_name/1,
      storage_place_for/2,
      storage_place_for_because/3,
      object_set_lifetime_begin/2,
      object_set_lifetime_end/2,
      object_is_alive/1,
      mark_dirty_objects/1,
      %%
      disposition_trigger_type/2
    ]).
/** <module> Utilities for reasoning about objects
  
  @author Moritz Tenorth
  @author Daniel Beßler
  @license BSD
*/

:- use_foreign_library('libknowrob_objects.so').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/temporal')).

:-  rdf_meta
    current_object_pose(r,-),
    object_lifetime(r,?),
    object_pose(r,+,r),
    object_pose(r,+),
    object_assert(r,r,+),
    object_pose_update(r,+),
    object_pose_update(r,+,+),
    object_trajectory(r,t,+,-),
    object_distance(r,r,-),
    object_dimensions(r, ?, ?, ?),
    object_state(r,t),
    object_state(r,t,t),
    object_color(r, ?),
    object_localization(r,r),
    object_frame_name(r,?),
    object_mesh_path(r, ?),
    object_assert_dimensions(r, +, +, +),
    object_assert_color(r, +),
    object_assert_frame_name(r),
    object_feature(r,r),
    object_quality(r,r,r),
    object_disposition(r,r),
    object_disposition(r,r,r),
    storage_place_for(r,r),
    storage_place_for_because(r,r,r),
    object_is_alive(r),
    object_set_lifetime_begin(r,+),
    object_set_lifetime_end(r,+),
    object_aspect_(r,r,r,r),
    disposition_trigger_type(r,r).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:- dynamic object_pose_data/3,
           current_object_pose_stamp/2.

object_assert_frame_name(Obj) :-
  rdf_split_url(_, ObjName, Obj),
  ( rdf_has(Obj, knowrob:'frameName', _) ;
    kb_assert(Obj, knowrob:'frameName', literal(ObjName)) ), !.

%% object_assert(+ObjType:iri, -Obj:iri, +Graph) is det.
%
% Asserts a new object to the named RDF graph.
%
% @param ObjType the type of the new object
% @param Obj the object asserted
%
object_assert(ObjType, Obj, Graph) :-
  kb_create(ObjType, Obj, _{graph:Graph}),
  % set TF frame to object name
  rdf_split_url(_, ObjName, Obj),
  kb_assert(Obj, knowrob:'frameName', literal(ObjName)).

%%
object_is_alive(Obj) :-
  object_lifetime_(Obj,LT),
  \+ rdf_has(LT,ease:hasIntervalEnd,_).

%%
object_lifetime(Obj,Interval) :-
  object_lifetime_(Obj,LT),
  inteval(LT,Interval).

object_lifetime_(Obj,LT) :-
  rdf_has(Obj,dul:hasTimeInterval,LT),!.

object_lifetime_(Obj,LT) :-
  once(rdf(Obj,rdf:type,_,G)),
  kb_create(dul:'TimeInterval',LT,_{graph:G}),
  rdf_assert(Obj,dul:hasTimeInterval,LT,G).

%%
object_set_lifetime_begin(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalBegin,Stamp,_{graph:G}).

%%
object_set_lifetime_end(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalEnd,Stamp,_{graph:G}).

%%
%% TODO: also assert Localization region to trigger
%%        mem to store it?
object_pose_update(Obj,Pose) :-
  current_time(Now),
  object_pose_update(Obj,Pose,Now).

object_pose_update(Obj,_,Stamp) :-
  object_pose_data(Obj,_,LatestStamp),
  LatestStamp >= Stamp, !.

object_pose_update(Obj,Pose,Stamp) :-
  ground(Pose),
  Pose = [_,_,_,_],
  %%
  (( current_object_pose_stamp(Obj,Latest), Latest >= Stamp ) -> true ; (
     retractall(current_object_pose_stamp(Obj,_)),
     asserta(current_object_pose_stamp(Obj,Stamp))
  )),
  %%
  retractall(object_pose_data(Obj,_,_)),
  asserta(object_pose_data(Obj,Pose,Stamp)),
  mark_dirty_objects([Obj]), !.

object_pose_update(Obj,Pose,_) :-
  print_message(warning, pose_update_failed(Obj,Pose)),
  fail.

%% current_object_pose(+Obj:iri, -Pose:list) is semidet
%
% Get the current pose of an object.
%
% @param Obj   Instance of SpatialThing
% @param Pose  The pose term [atom Reference, atom Target, [float x,y,z], [float qx,qy,qz,qw]]
% 
current_object_pose(Obj,Pose) :- 
  object_pose_data(Obj,Pose,_),!.

current_object_pose(Obj,[ParentFrame,ObjFrame,T,Q]) :- 
  ground(ParentFrame),
  object_frame_name(Obj,ObjFrame),
  current_map_pose(ObjFrame,MapPose0),
  current_map_pose(ParentFrame,MapPose1),
  transform_between(MapPose1,MapPose0,[ParentFrame,ObjFrame,T,Q]), !.

current_object_pose(Obj,[RefFrame,ObjFrame,T,Q]) :-
  object_localization(Obj,Loc),
  kb_triple(Loc, ease_obj:hasSpaceRegion, [RefFrame,_,T,Q]),
  object_frame_name(Obj,ObjFrame),!.

current_object_pose(Obj,[ParentFrame,ObjFrame,T,Q]) :-
  % try TF lookup in case above clauses failed.
  (ground(ParentFrame) ; map_frame_name(ParentFrame)),
  object_frame_name(Obj,ObjFrame),
  tf_lookup_transform(ParentFrame,ObjFrame,pose(T,Q)),!.

%%
current_map_pose(ObjFrame,MapPose) :-
  map_frame_name(MapFrame),
  object_pose_data(_,[ParentFrame,ObjFrame,T,Q],_),
  ( MapFrame = ParentFrame ->
    MapPose=[MapFrame,ObjFrame,T,Q] ; (
    current_map_pose(ParentFrame,MapParent),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).
  
%% object_pose(+Obj:iri, ?Pose:term, +Instant:float) is semidet
%
% True if Pose is the pose of Obj at time Instant.
% Poses may be requested in particular reference frames
% by partially specifying Pose. For instance, to request the pose in camera frame:
%    Pose=['head_rgb_camera',ObjFrame,Position,Orientation]
%
% @param Obj     Instance of SpatialThing
% @param Instant The time instant (float or Timepoint:iri)
% @param Pose    The pose [atom Reference, atom Target, [float x,y,z], [float qx,qy,qz,qw]]
% 
object_pose(Obj, Pose) :-
  current_object_pose(Obj, Pose).

object_pose(Obj, Pose, Stamp0) :- 
  object_pose_data(Obj,Pose,Stamp1),
  Stamp1 < Stamp0, !.

object_pose(Obj, [RefFrame,ObjFrame,T,Q], Time) :- 
  ground(RefFrame),
  object_frame_name(Obj,ObjFrame),
  object_map_pose(ObjFrame,MapPose0,Time),
  object_map_pose(RefFrame,MapPose1,Time),
  transform_between(MapPose1,MapPose0,[RefFrame,ObjFrame,T,Q]), !.

object_pose(Obj, [RefFrame,ObjFrame,T,Q], Time) :- 
  object_frame_name(Obj,ObjFrame),
  object_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [RefFrame,_,T,Q], Time),!.

object_map_pose(ObjFrame,MapPose,Time) :-
  map_frame_name(MapFrame),
  object_frame_name(Obj,ObjFrame), % FIXME: only works with Obj Bound
  object_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [ParentFrame,_,T,Q], Time),
  ( MapFrame = ParentFrame ->
    MapPose=[MapFrame,ObjFrame,T,Q] ; (
    object_map_pose(ParentFrame,MapParent,Time),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).

object_localization(Obj,Obj) :-
  atom(Obj),
  rdfs_individual_of(Obj,ease_obj:'Localization'),!.
object_localization(Obj,Obj) :-
  atom(Obj),
  rdf_has(Obj, ease_obj:hasSpaceRegion, _),!.
object_localization(Obj,Loc) :-
  object_localization_(Obj,Loc).

mark_dirty_objects([]) :- !.
mark_dirty_objects(Objects) :-
  %
  findall(ObjState, (
    member(Obj,Objects),
    object_pose_data(Obj,_,_),
    object_state(Obj,ObjState)
  ), ObjStates),
  object_state_add_cpp(ObjStates).

%% object_state(+Obj:iri, ?State:list) is semidet
%% object_state(+Obj:iri, ?State:list, +Properties:dict) is semidet
%
object_state(Obj, State) :-
  object_state(Obj, State, _{}).

object_state(Obj, State, Properties_list) :-
  is_list(Properties_list),!,
  findall(Key-Val, (
    member(X,Properties_list),
    X=..[Key,Val]
  ), Pairs),
  dict_pairs(Dict, _, Pairs),
  object_state(Obj, State, Dict).

object_state(Obj, [
   Obj,       % object_id
   FrameName, % frame_name
   TypeName,  % object_type
   Shape,     % shape
   Mesh,      % mesh_path
   [R,G,B,A], % color
   [D,W,H],   % size
   Pose,      % pose
   StaticTransforms % static_transforms
], Properties) :-
  ( get_dict(timestamp,Properties,Time) ;
    current_time(Time)
  ),
  %
  object_frame_name(Obj,FrameName),
  kb_type_of(Obj,Type),
  rdf_split_url(_,TypeName,Type),
  % get the shape, default to BoxShape
  ( get_dict(shape,Properties,ShapeIri) ;
    object_shape_type(Obj,ShapeIri);
    rdf_equal(ShapeIri,ease_obj:'BoxShape')
  ),
  object_state_shape_(ShapeIri,Shape),
  % get the color, default to grey color
  ( get_dict(color,Properties,[R,G,B,A]) ;
    object_color(Obj,[R,G,B,A]);
    [R,G,B,A]=[0.5,0.5,0.5,1.0]
  ),
  % get mesh path or empty string
  ( get_dict(mesh,Properties,Mesh) ;
    object_mesh_path(Obj,Mesh);
    Mesh=''
  ),
  % get the object bounding box
  ( get_dict(bbox,Properties,[D,W,H]) ;
    object_dimensions(Obj,D,W,H);
    [D,W,H] = [0.05,0.05,0.05]
  ), !,
  % handle transforms
  ( object_pose(Obj, Pose, Time); (
    print_message(warning, unlocalized(Obj)),
    fail
  )),
  findall(X, (
    object_feature(Obj,Feature),
    feature_transform(Obj,Feature,X)
  ), StaticTransforms),
  !.

%object_state_shape_(Iri,0) :- rdfs_subclass_of(Iri,ease_obj:'Arrow'),!.
object_state_shape_(Iri,1) :- rdfs_subclass_of(Iri,ease_obj:'BoxShape'),!.
object_state_shape_(Iri,2) :- rdfs_subclass_of(Iri,ease_obj:'SphereShape'),!.
object_state_shape_(Iri,3) :- rdfs_subclass_of(Iri,ease_obj:'CylinderShape'),!.

%%
% Map RDF transform to Prolog list representation.
%
knowrob:kb_rdf_object(SpaceRegion,[Ref_frame,_,Pos,Rot]) :-
  ground(SpaceRegion),
  rdfs_individual_of(SpaceRegion,ease_obj:'6DPose'),!,
  transform_reference_frame(SpaceRegion,Ref_frame),
  transform_data(SpaceRegion,(Pos,Rot)).

%% object_trajectory(+Obj, +Interval, +Density, -Trajectory) is semidet
%
% Sample trajectory of Obj's origin frame within Interval.
% The trajectory either has Count (Density=num_samples(Count)) samples,
% or Delta (Density=dt(Delta)) temporal distance between samples (in seconds).
% Trajectory is a list of poses
%   [reference_frame, source_frame, position, orientation]
%
% @param Obj Instance of SpatialThing
% @param Interval Interval of the trajectory
% @param Density Trajectory density
% @param Trajectory Sample trajectory (list of pose terms)
%
% TODO(daniel): Implement more efficient way of sampling trajectories.
%               - create interface for mongo to query many transforms at once
object_trajectory(Obj, Interval, num_samples(Count), Trajectory) :-
  interval(Interval, [Begin,End]),
  Dt is (End - Begin) / Count,
  object_trajectory(Obj, Interval, dt(Dt), Trajectory).

object_trajectory(_, [Begin,End], _, []) :- Begin > End, !.
object_trajectory(Obj, [Begin,End], dt(Dt), [X|Xs]) :-
  Begin =< End, Next is Begin + Dt,
  object_pose_at_time(Obj, Begin, X),
  object_trajectory(Obj, [Next,End], dt(Dt), Xs).

%% object_frame_name(+Obj:iri, ?FrameName:atom) is det
%
% True if FrameName is the name of origin frame of the object.
% Fallback is to use the IRI suffix of Obj.
% 
% @param Obj Instance of SpatialThing
% @param FrameName The frame name
%
object_frame_name(Obj,FrameName) :-
  kb_triple(Obj,knowrob:frameName,FrameName), !.

object_frame_name(Localization,FrameName) :-
  rdf_has(Obj,ease_obj:hasLocalization,Localization),
  object_frame_name(Obj,FrameName), !.

object_frame_name(Obj,FrameName) :-
  atom(Obj), rdf_split_url(_,FrameName,Obj).

%% object_distance(+A:iri, +B:iri, ?Distance:float) is semidet
% 
% Computes euclidean distance between A and B.
%
% @param A         Instance of SpatialThing
% @param B         Instance of SpatialThing
% @param Distance  The current distance between A and B
%
object_distance(A,B,Distance):-
  map_frame_name(MapFrame),
  current_object_pose(A, [MapFrame,_,[AX,AY,AZ],_]),
  current_object_pose(B, [MapFrame,_,[BX,BY,BZ],_]),
  DX is AX - BX,
  DY is AY - BY,
  DZ is AZ - BZ,
  Distance is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)), !.

%% object_color(?Obj:iri, ?Col:list) is det
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue, alpha], on a scale of 0-1.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color(Obj, [R,G,B,A]) :-
  kb_triple(Obj,ease_obj:hasColor,Color),
  kb_triple(Color,dul:hasRegion,ColorRegion),
  kb_triple(ColorRegion, ease_obj:hasRGBValue, [R,G,B|Rest]),
  ( Rest=[A] ; A is 1.0 ),!.

object_color(Obj, [R,G,B,A]) :-
  kb_triple(Obj, ease_obj:hasRGBValue, [R,G,B|Rest]),
  ( Rest=[A] ; A is 1.0 ),!.

%% object_assert_color(+Obj:iri, +Col:list) is det
%
% Assert object main color property.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_assert_color(Obj,ColorValue) :-
  once(rdf(Obj,_,_,G)),
  object_color_(Obj,Color),
  %%
  kb_create(ease_obj:'ColorRegion',ColorRegion,_{graph: G}),
  kb_assert(ColorRegion,ease_obj:hasRGBValue,ColorValue),
  %%
  kb_retract(Color,dul:hasRegion,_),
  kb_assert(Color,dul:hasRegion,ColorRegion).

%% object_dimensions(?Obj:iri, ?Depth:float, ?Width:float, ?Height:float) is semidet
%
% True if Depth x Width x Height are (exactly) the extends of the bounding box of Obj.
% NOTE that we use ROS conventions here: Coordinate systems in ROS are
% always right-handed, with X forward, Y left, and Z up. 
%
% @param Obj    Instance of SpatialThing
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_dimensions(Obj, Depth, Width, Height) :-
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_triple(Shape,dul:hasRegion,ShapeRegion),
  shape_bbox(ShapeRegion, Depth, Width, Height),!.

object_dimensions(Obj, Depth, Width, Height) :-
  shape_bbox(Obj, Depth, Width, Height),!.

%% object_shape_type(?Obj:iri, ?Shape:iri) is semidet
%
object_shape_type(Obj,ShapeType) :-
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_type_of(Shape,ShapeType).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) :-
  kb_triple(ShapeRegion, ease_obj:hasDepth, Depth),
  kb_triple(ShapeRegion, ease_obj:hasWidth, Width),
  kb_triple(ShapeRegion, ease_obj:hasHeight, Height), !.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) :-
  kb_type_of(ShapeRegion,ease_obj:'SphereShape'),
  kb_triple(ShapeRegion, ease_obj:hasRadius, Radius), !,
  number(Radius),
  Diameter is 2 * Radius.

%% object_assert_dimensions(+Obj:iri, +Depth:float, +Width:float, +Height:float) is det
%
% Assert object dimension properties.
%
% @param Obj    Instance of a subclass of EnduringThing-Localized
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_assert_dimensions(Obj, Depth, Width, Height) :-
  once(rdf(Obj,_,_,G)),
  object_shape_(Obj, Shape),
  % create a new region
  kb_create(ease_obj:'BoxShape',ShapeRegion,_{graph: G}),
  kb_assert(ShapeRegion, ease_obj:hasDepth, Depth),
  kb_assert(ShapeRegion, ease_obj:hasWidth, Width),
  kb_assert(ShapeRegion, ease_obj:hasHeight, Height),
  % unlink previous box shapes
  forall((
    kb_triple(Shape,dul:hasRegion,X),
    kb_type_of(X,ease_obj:'BoxShape')),
    kb_retract(Shape,dul:hasRegion,X)
  ),
  kb_assert(Shape,dul:hasRegion,ShapeRegion),
  mark_dirty_objects([Obj]).

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) :-
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_triple(Shape,dul:hasRegion,ShapeRegion),
  kb_triple(ShapeRegion,ease_obj:hasFilePath,FilePath),!.

object_mesh_path(Obj, FilePath) :-
  kb_triple(Obj,ease_obj:hasFilePath,FilePath),!.
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object qualities

%%
object_aspect_(Object,Relation,AspectType,Aspect) :-
  atom(Object),
  rdfs_individual_of(Object,dul:'Object'),
  (( 
    kb_triple(Object,Relation,Aspect),
    kb_type_of(Aspect,AspectType)
  ) ; (
    once(rdf(Object,_,_,G)),
    kb_create(AspectType,Aspect,_{graph: G}),
    kb_assert(Object,Relation,Aspect),
    % FIXME: below is a bit hacked, shouldn't e.g. owl_individual_of
    %         draw restrictions from related things?
    %       - this is problematic when AspectType is something general and
    %         there are multiple aspects with that type!
    %         probably best to take max cardinality and disjointness into account.
    forall((
      property_cardinality(Object,Relation,X,Min,_),
      Min>0,
      once(owl_subclass_of(X,AspectType))
    ), (
      kb_assert(Aspect,rdf:type,X)
    ))
  )), !.

%%
object_color_(Obj,Color) :-
  object_aspect_(Obj, ease_obj:hasColor, ease_obj:'Color', Color).
%%
object_shape_(Obj,Shape) :-
  object_aspect_(Obj, ease_obj:hasShape, ease_obj:'Shape', Shape).
%%
object_localization_(Obj,Localization) :-
  object_aspect_(Obj, ease_obj:hasLocalization, ease_obj:'Localization', Localization).

object_quality(Obj, QualityType, Quality) :-
  object_aspect_(Obj, dul:hasQuality, QualityType, Quality).
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object dispositions

%% object_disposition(?Obj:iri, ?Disposition:iri) is nondet.
%
% Relates an object to one of its dispositions.
%
% @param Obj           An individual of type dul:'Object'.
% @param Disposition   An individual of type ease_obj:'Disposition'.
%
object_disposition(Obj, Disposition) :-
  atom(Disposition),!,
  rdf_has(Obj,ease_obj:hasDisposition,Disposition).
object_disposition(Obj, Disposition) :-
  kb_some(Obj,ease_obj:hasDisposition,DispositionType),
  object_aspect_(Obj,ease_obj:hasDisposition,DispositionType,Disposition).

%% object_disposition(?Obj:iri, ?Disposition:iri, +DispositionType:iri) is nondet.
%
% Relates an object to one of its dispositions that is an instance
% of given disposition type.
%
% @param Obj               An individual of type dul:'Object'.
% @param Disposition       An individual of type ease_obj:'Disposition'.
% @param DispositionType   A sub-class of ease_obj:'Disposition'.
%
object_disposition(Obj, Disposition, DispositionType) :-
  ground(DispositionType),!,
  object_aspect_(Obj,ease_obj:hasDisposition,DispositionType,Disposition).
object_disposition(Obj, Disposition, DispositionType) :-
  object_disposition(Obj, Disposition),
  % get the Disposition type
  once((
    kb_type_of(Disposition,DispositionType),
    owl_subclass_of(DispositionType,ease_obj:'Disposition')
  )).

%% disposition_trigger_type(?Disposition:iri, ?TriggerType:iri) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition   An individual of type ease_obj:'Disposition'.
% @param TriggerType   A sub-class of dul:'Object'.
%
disposition_trigger_type(Disposition,TriggerType) :-
  property_range(Disposition,ease_obj:affordsTrigger,TriggerRole),!,
  property_range(TriggerRole,dul:classifies,ClassifiedType),
  ( var(TriggerType) -> TriggerType=ClassifiedType ;
    once(owl_subclass_of(TriggerType,ClassifiedType))
  ),
  rdfs_subclass_of(TriggerType,dul:'Object'),!.
  
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object features

%%
object_feature(Obj, Feature) :-
  atom(Feature),!,
  rdf_has(Obj,ease_obj:hasFeature,Feature).
object_feature(Obj, Feature) :-
  kb_some(Obj,ease_obj:hasFeature,FeatureType),
  object_aspect_(Obj,ease_obj:hasFeature,FeatureType,Feature).

%%
object_feature(Obj, Feature, FeatureType) :-
  ground(FeatureType),!,
  object_aspect_(Obj,ease_obj:hasFeature,FeatureType,Feature).
object_feature(Obj, Feature, FeatureType) :-
  object_feature(Obj, Feature),
  % get the feature type
  once((
    kb_type_of(Feature,FeatureType),
    owl_subclass_of(FeatureType,ease_obj:'Feature')
  )).

%%
feature_transform(Obj, Feature, [ObjFrame,FeatureFrame,Pos,Rot]) :-
  %%
  object_frame_name(Obj, ObjFrame),
  current_object_pose(Feature, [ObjFrame,FeatureFrame,Pos,Rot]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Reasoning about function and storage location of objects


%% storage_place_for(St, ObjT) is nondet
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% 
storage_place_for(St, ObjT) :-
  storage_place_for_because(St, ObjT, _).

%% storage_place_for_because(St, ObjType, ObjT) is nondet
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% In addition to the storage place, this predicate further returns the superclass
% of Obj for which this information is asserted (e.g. Perishable)
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% @param ObjType  Class for which information about the storage place has been asserted
%
storage_place_for_because(Container,Object,PatientType) :-
  ground([Container,Object]) ->
  once(storage_place_for_because_(Container,Object,PatientType)) ;
  storage_place_for_because_(Container,Object,PatientType).

storage_place_for_because_(Container,Object,PatientType) :-
  atom(Object),
  rdfs_individual_of(Object,dul:'Entity'),!,
  kb_type_of(Object,ObjType),
  storage_place_for_because_(Container,ObjType,PatientType).

storage_place_for_because_(Container,ObjType,PatientType) :-
  atom(Container),
  object_disposition(Container, Disposition, ease_obj:'Containment'),
  storage_place_for_because__(Disposition,ObjType,PatientType).

storage_place_for_because__(Disposition,ObjType,PatientType) :-
  property_cardinality(Disposition,ease_obj:affordsTrigger,Concept,Min,_), Min>0,
  property_range(Concept,dul:classifies,PatientType),
  rdfs_subclass_of(PatientType,dul:'PhysicalObject'),
  rdfs_subclass_of(ObjType,PatientType).
