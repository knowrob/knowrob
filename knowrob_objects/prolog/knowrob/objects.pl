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
      object_color/2,
      object_dimensions/4,
      object_mesh_path/2,
      object_assert_dimensions/4,
      object_assert_color/2,
      object_assert_frame_name/1,
      object_affordance/2,
      object_instantiate_affordances/1,
      object_affordance_static_transform/3,
      object_perception_affordance_frame_name/2,
      object_information/8,
      storage_place_for/2,
      storage_place_for_because/3,
      object_set_lifetime_begin/2,
      object_set_lifetime_end/2
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
    object_color(r, ?),
    object_frame_name(r,?),
    object_mesh_path(r, ?),
    object_assert_dimensions(r, +, +, +),
    object_assert_color(r, +),
    object_assert_frame_name(r),
    object_affordance(r,r),
    object_instantiate_affordances(r),
    object_affordance_static_transform(r,r,?),
    storage_place_for(r,r),
    storage_place_for_because(r,r,r),
    object_set_lifetime_begin(r,+),
    object_set_lifetime_end(r,+),
    object_quality_(r,r,r,r).

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
object_lifetime(Obj,Interval) :-
  object_lifetime_(Obj,LT),
  inteval(LT,Interval).

object_lifetime_(Obj,LT) :-
  rdf_has(Obj,dul:hasTimeInterval,LT),!.

object_lifetime_(Obj,LT) :-
  once(rdf(Obj,rdf:type,_,G)),
  kb_create(dul:'TimeInterval',LT,G),
  rdf_assert(Obj,dul:hasTimeInterval,LT,G).

%%
object_set_lifetime_begin(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalBegin,Stamp,G).

%%
object_set_lifetime_end(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalEnd,Stamp,G).

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
  get_localization(Obj,Loc),
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
  get_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [RefFrame,_,T,Q], Time),!.

object_map_pose(ObjFrame,MapPose,Time) :-
  map_frame_name(MapFrame),
  object_frame_name(Obj,ObjFrame),
  get_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [ParentFrame,_,T,Q], Time),
  ( MapFrame = ParentFrame ->
    MapPose=[MapFrame,ObjFrame,T,Q] ; (
    object_map_pose(ParentFrame,MapParent,Time),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).

get_localization(Obj,Obj) :-
  atom(Obj),
  rdfs_individual_of(Obj,ease_obj:'Localization'),!.
get_localization(Obj,Obj) :-
  atom(Obj),
  rdf_has(Obj, ease_obj:hasSpaceRegion, _),!.
get_localization(Obj,Loc) :-
  object_localization_(Obj,Loc).

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

%%
object_information(Obj, TypeName, HasVisual, Color, Mesh, [D, W, H], Pose, StaticTransforms) :-
  kb_type_of(Obj,Type), rdf_split_url(_,TypeName,Type),
  (owl_has(Obj, knowrob:'hasVisual', literal(type(_,HasVisual)));HasVisual=true),
  (object_color(Obj,Color);(Color=[0.5,0.5,0.5,1.0])),
  (object_mesh_path(Obj,Mesh);Mesh=''),
  (object_dimensions(Obj,D,W,H);(D=0.05,W=0.05,H=0.05)),
  (current_object_pose(Obj, Pose);Pose=[map,null,[0,0,0],[0,0,0,1]]),
  findall(X, object_affordance_static_transform(Obj,_,X), StaticTransforms), !.
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object qualities

%%
object_quality_(Obj,HasQuality,QualityType,Quality) :-
  atom(Obj),
  rdfs_individual_of(Obj,dul:'PhysicalObject'),
  ( kb_triple(Obj,HasQuality,Quality) ; (
    once(rdf(Obj,_,_,G)),
    kb_create(QualityType,Quality,_{graph: G}),
    kb_assert(Obj,HasQuality,Quality)
  )), !.

%%
object_color_(Obj,Color) :-
  object_quality_(Obj, ease_obj:hasColor, ease_obj:'Color', Color).
%%
object_shape_(Obj,Shape) :-
  object_quality_(Obj, ease_obj:hasShape, ease_obj:'Shape', Shape).
%%
object_localization_(Obj,Localization) :-
  object_quality_(Obj, ease_obj:hasLocalization, ease_obj:'Localization', Localization).
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object affordances
%% TODO: need to revise affordances. Consider using
%%       disposition model.

%%
object_perception_affordance_frame_name(Obj, AffFrameName) :-
  object_instantiate_affordances(Obj), % HACK
  owl_has(Obj, knowrob:hasAffordance, Aff),
  rdfs_individual_of(Aff, knowrob:'PerceptionAffordance'),
  object_frame_name(Aff, AffFrameName), !.

%%
object_affordance(Obj, Aff) :-
  object_instantiate_affordances(Obj), % HACK
  owl_has(Obj, knowrob:hasAffordance, Aff).

%%
object_affordance_static_transform(Obj, Aff, [ObjFrame,AffFrame,Pos,Rot]) :-
  object_instantiate_affordances(Obj), % HACK
  object_frame_name(Obj, ObjFrame),
  owl_has(Obj, knowrob:hasAffordance, Aff),
  % TODO: StaticAffordanceTransform declares
  %  ((relativeTo o hasAffordance o pose) Self)
  % Use this to infer the relativeTo entity:
  %    relativeTo value (Self.(inverse(pose)oinverse(hasAffordance)))
  current_object_pose(Aff, [ObjFrame,AffFrame,Pos,Rot]).

%%
object_instantiate_affordances(Obj) :-
  findall(Type, (
    owl_restriction_on(Obj, knowrob:hasAffordance, R),
    owl_restriction_object_domain(R, Type)), Types),
  list_to_set(Types, Types_set),
  forall(
    owl_most_specific(Types_set, Specific), (
    owl_description(Specific, Specific_descr),
    ignore(object_instantiate_affordances(Obj, Specific_descr))
  )).

object_instantiate_affordances(Obj, class(Cls)) :-
  owl_cardinality_on_resource(Obj, knowrob:hasAffordance, Cls, cardinality(Desired,_)),
  owl_cardinality(Obj, knowrob:hasAffordance, Cls, Actual),
  Missing is Desired - Actual,
  object_instantiate_affordances(Obj,[Cls],Missing).

object_instantiate_affordances(Obj, union_of(Classes)) :-
  forall(
    member(Cls,Classes), (
    owl_description(Cls,Cls_descr),
    ignore(object_instantiate_affordances(Obj,Cls_descr))
  )).

object_instantiate_affordances(_Obj, intersection_of(_Classes)) :- fail.
object_instantiate_affordances(_Obj, one_of(_Classes))          :- fail.
object_instantiate_affordances(_Obj, complement_of(_Classes))   :- fail.

object_instantiate_affordances(_,_,Missing) :- Missing =< 0, !.
object_instantiate_affordances(Obj,[Cls|Rest],Missing) :-
  kb_create(Cls, Affordance),
  forall(member(X,Rest), rdf_assert(Affordance,rdf:type, X)),
  rdf_assert(Obj, knowrob:hasAffordance, Affordance),
  Next is Missing-1,
  object_instantiate_affordances(Obj,[Cls|Rest],Next).

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
  kb_triple(Container,ease_obj:hasDesign,Design),
  kb_type_of(Design,ease_obj:'Containment'),
  storage_place_for_because__(Design,ObjType,PatientType).

storage_place_for_because_(Container,ObjType,PatientType) :-
  atom(Container),
  property_cardinality(Container,ease_obj:hasDesign,Design,Min,_), Min>0,
  rdfs_subclass_of(Design,ease_obj:'Containment'),
  storage_place_for_because__(Design,ObjType,PatientType).

storage_place_for_because__(Design,ObjType,PatientType) :-
  property_range(Design,ease_obj:hasDesignatedPatient,PatientType),
  rdfs_subclass_of(PatientType,dul:'PhysicalObject'),
  rdfs_subclass_of(ObjType,PatientType).
