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
      object_pose/2,
      object_pose/3,
      current_object_pose/2,
      current_object_pose_stamp/2,
      object_pose_update/2,
      object_pose_update/3,
      object_distance/3,
      storage_place_for/2,
      storage_place_for_because/3,
      mark_dirty_objects/1
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
:- use_module(library('knowrob/model/Object')).

:-  rdf_meta
    current_object_pose(r,-),
    object_pose(r,+,r),
    object_pose(r,+),
    object_pose_update(r,+),
    object_pose_update(r,+,+),
    object_trajectory(r,t,+,-),
    object_distance(r,r,-),
    storage_place_for(r,r),
    storage_place_for_because(r,r,r).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

:- dynamic object_pose_data/3,
           current_object_pose_stamp/2.

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
  asserta(object_pose_data(Obj,Pose,Stamp)), !.

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

current_object_pose(Obj, [RefFrame,ObjFrame,T,Q]) :- 
  object_frame_name(Obj,ObjFrame),
  object_localization(Obj,Loc),
  kb_triple(Loc, ease_obj:hasSpaceRegion, [ParentFrame,ObjFrame,T0,Q0]),
  ( RefFrame=ParentFrame ->
  ( T=T0, Q=Q0 ) ;
  ( current_map_pose_([ParentFrame,ObjFrame,T0,Q0], MapPose0),
    current_map_pose(RefFrame,MapPose1),
    transform_between(MapPose1,MapPose0,[RefFrame,ObjFrame,T,Q])
  )),!.

current_object_pose(Obj,[ParentFrame,ObjFrame,T,Q]) :-
  % try TF lookup in case above clauses failed.
  (ground(ParentFrame) ; map_frame_name(ParentFrame)),
  object_frame_name(Obj,ObjFrame),
  tf_lookup_transform(ParentFrame,ObjFrame,pose(T,Q)),!.

%%
current_map_pose(ObjFrame,MapPose) :-
  object_frame_name(Obj,ObjFrame),
  current_object_pose(Obj,CurrentPose),
  current_map_pose_(CurrentPose,MapPose).

current_map_pose_([ParentFrame,ObjFrame,T,Q],MapPose) :-
  map_frame_name(MapFrame),
  ( MapFrame=ParentFrame ->
  ( MapPose=[MapFrame,ObjFrame,T,Q] ) ;
  ( current_map_pose(ParentFrame,MapParent),
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
  object_frame_name(Obj,ObjFrame),
  object_localization(Obj,Loc),
  holds(Loc, ease_obj:hasSpaceRegion, [ParentFrame,ObjFrame,T0,Q0], Time),
  ( RefFrame=ParentFrame ->
  ( T=T0, Q=Q0 ) ;
  ( object_map_pose_([ParentFrame,ObjFrame,T0,Q0], MapPose0, Time),
    object_map_pose(RefFrame,MapPose1,Time),
    transform_between(MapPose1,MapPose0,[RefFrame,ObjFrame,T,Q])
  )).

%%
object_map_pose(ObjFrame,MapPose,Time) :-
  object_frame_name(Obj,ObjFrame),
  object_pose(Obj,CurrentPose,Time),
  object_map_pose_(CurrentPose,MapPose,Time).

object_map_pose_([ParentFrame,ObjFrame,T,Q],MapPose,Time) :-
  map_frame_name(MapFrame),
  ( MapFrame=ParentFrame ->
  ( MapPose=[MapFrame,ObjFrame,T,Q] ) ;
  ( object_map_pose(ParentFrame,MapParent,Time),
    transform_multiply(MapParent,
      [ParentFrame,ObjFrame,T,Q], MapPose)
  )).

%%
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
  atom(SpaceRegion),
  rdfs_individual_of(SpaceRegion,ease_obj:'6DPose'),!,
  transform_reference_frame(SpaceRegion,Ref_frame),
  transform_data(SpaceRegion,(Pos,Rot)).

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

%%
feature_transform(Obj, Feature, [ObjFrame,FeatureFrame,Pos,Rot]) :-
  %%
  object_frame_name(Obj, ObjFrame),
  current_object_pose(Feature, [_,FeatureFrame,Pos,Rot]).

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
  object_disposition(Container, Disposition, ease_obj:'Insertion'),
  storage_place_for_because__(Disposition,ObjType,PatientType).

storage_place_for_because__(Disposition,ObjType,PatientType) :-
  % FIXME: bug in property_range, ease_obj:affordsTrigger range inferred as plain role,
  %        without including constraints derived from axioms of Disposition!
  property_range(Disposition,[ease_obj:affordsTrigger,dul:classifies],PatientType),
  rdfs_subclass_of(PatientType,dul:'PhysicalObject'),
  rdfs_subclass_of(ObjType,PatientType).
