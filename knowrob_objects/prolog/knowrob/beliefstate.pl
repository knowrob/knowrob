/*
  Copyright (C) 2017 Daniel Beßler
  Copyright (C) 2017 Mihai Pomarlan

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

:- module(knowrob_beliefstate,
    [
      belief_parse/1,
      belief_existing_objects/1,    % set of known objects in the belief state
      belief_existing_objects/2,
      belief_existing_object_at/4,  % query known object near some pose
      belief_new_object/2,          % assert a new object in the belief state
      belief_new_part/3,
      belief_at/2,                  % query the current pose of an object
      belief_at/3,
      belief_at_id/2,
      belief_at_id/3,
      belief_at_update/2,           % assign new pose to object
      belief_at_update/3,
      belief_at_global/2,           % query the current pose of an object in global frame
      belief_at_global/3,
      belief_at_relative_to/3,      % query the current pose of an object relative to some parent object
      belief_at_relative_to/4,
      belief_at_internal/2,         % TODO: these should not be exposed
      belief_at_internal/3,
      belief_perceived_at/4,        % convinience rule to be called by perception system to inform about perceptions
      belief_perceived_part_at/5,
      belief_perceived_part_at_axis/4,
      belief_republish_objects/1,   % causes marker messages to be generated
      belief_forget/0,
      belief_new_pose/3,
      belief_new_pose/2
    ]).
/** <module> Maintaining symbolic beliefs about perceived objects.
  
  @author Daniel Beßler
  @license BSD
*/

:- use_module(library('lists')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/rdfs')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/objects')).

:- use_foreign_library('libknowrob_objects.so').

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).

:-  rdf_meta
    belief_existing_objects(-,t),
    belief_existing_object_at(r,+,+,r),
    belief_new_object(r,r),
    belief_new_part(r,r,r),
    belief_at(r,+,r),
    belief_at(r,+),
    belief_at_id(r,+,r),
    belief_at_id(r,+),
    belief_at_update(r,+,r),
    belief_at_update(r,+),
    belief_at_internal(r,+,r),
    belief_at_internal(r,+),
    belief_at_global(r,-),
    belief_at_relative_to(r,r,-),
    belief_perceived_at(r,+,+,r),
    belief_perceived_part_at(r,+,+,r,r),
    belief_perceived_part_at_axis(r,r,+,r),
    belief_republish_objects(t).

% TODO
% - auto-instantiate object parts according to class restrictions
%        - only fixed parts share frame of reference with Obj
%        - align with knowrob_assembly ontology!
% - use octree (or so) to find the nearest object perceived before
%      instead of going through complete belief state.

%% belief_existing_objects(-ObjectIds:list) is det.
%
% Returns a list of perceived objects that are known to the KB.
%
% @param ObjectIds the object names
%
belief_existing_objects(ObjectIds) :-
  belief_existing_objects(ObjectIds, [knowrob:'EnduringThing-Localized']).

belief_existing_objects(ObjectIds, ObjectTypes) :-
  findall(J, (
      rdf(J, _, _, belief_state),
      member(T, ObjectTypes),
      rdfs_individual_of(J, T)
  ), X),
  list_to_set(X,ObjectIds).

belief_parse(File) :-
  owl_parser:owl_parse(File, belief_state),
  belief_assign_frames.

belief_assign_frames :-
  belief_existing_objects(ObjectIds),
  forall( member(Obj,ObjectIds), (
    rdf_split_url(_, ObjName, Obj),
    once(( rdf_has(Obj, knowrob:'frameName', _) ;
           rdf_assert(Obj, knowrob:'frameName', literal(ObjName), belief_state) )))).
  

%% belief_forget is det.
%
% Retracts all facts asserted to the "belief_state" RDF graph.
%
belief_forget :-
  forall( rdf(J, _, _, belief_state),
          retractall(J,_,_) ).

%% belief_new_object(+ObjType:iri, -Obj:iri) is det.
%
% Asserts a new object to the "belief_state" RDF graph.
%
% @param ObjType the type of the new object
% @param Obj the object asserted
%
belief_new_object(ObjType, Obj) :-
  rdf_instance_from_class(ObjType, belief_state, Obj),
  rdf_assert(Obj, rdf:type, owl:'NamedIndividual', belief_state),
  % set TF frame to object name
  rdf_split_url(_, ObjName, Obj),
  rdf_assert(Obj, knowrob:'frameName', literal(ObjName), belief_state),
  ignore(once((
    %% HACK get this info from somewhere else!
    rdfs_individual_of(Map, knowrob:'SemanticEnvironmentMap'),
    rdf_assert(Obj, knowrob:'describedInMap', Map, belief_state)
  ))).

%%
belief_new_part(PartType, Part, Parent) :-
  belief_new_object(PartType, Part),
  rdf_assert(Parent, knowrob:properPhysicalParts, Part, belief_state).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Beliefs about the class of things

%% belief_class_of(+Obj:iri, +ObjType:iri) is semidet.
%
% From now on, belief that Obj is of type ObjType.
% Old beliefs are withdrawn but still memorized as
% temporal part of Obj (see knowrob_temporal).
%
% @param Obj a perceived object
% @param ObjType the new type of the object
%
belief_class_of(Obj, ObjType) :-
  % nothing to do if current classification matches beliefs
  rdfs_type_of(Obj, ObjType), !.
belief_class_of(Obj, NewObjType) :-
  current_time(Now),
  ignore(once((
      % withdraw old beliefs about object type
      rdfs_type_of(Obj, CurrObjType),
      rdfs_subclass_of(CurrObjType, Parent),
      rdfs_subclass_of(NewObjType, Parent),
      assert_temporal_part_end(Obj, rdf:type, CurrObjType, Now, belief_state)
  ))),
  assert_temporal_part(Obj, rdf:type, nontemporal(NewObjType), Now, belief_state).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Beliefs about the spatial location of things

%% belief_existing_object_at(+ObjType:iri, +Transform:list, +Threshold:float, -Obj:iri) is semidet.
%
% Checks whether one of the already known objects of ObjType is located close to Transform.
% The check uses Threshold as a threshold for comparing translation distance.
%
% Transform is expected to be of the form [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]]
%
% If such an object is found, its id is returned via ObjectId.
%
% @param ObjType     the object type
% @param Transform   the transform data
% @param Threshold   a distance below which two translations are thought to be the same
% @param Obj         the object id
%
belief_existing_object_at(_, Transform, Threshold, Obj) :-
  % FIXME: super slow for large belief state
  rdf(Obj, rdf:type, owl:'NamedIndividual', belief_state),
  rdfs_individual_of(Obj, knowrob:'EnduringThing-Localized'),
  belief_object_at_location(Obj, Transform, Threshold), !.

belief_object_at_location(ObjectId, NewPose, Dmax) :-
  belief_at_id(ObjectId, OldPose),
  transform_close_to(NewPose, OldPose, Dmax).

%% belief_perceived_at(+ObjType:iri, +Transform:list, +Threshold:float, -Obj:iri) is det.
%
% Convenience predicate that first tries to find some existing object
% Obj at the pose Transform and if this fails asserts a new object
% at that pose.
%
% @param ObjType     the object type
% @param Transform   the transform data
% @param Threshold   a distance below which two translations are thought to be the same
% @param Obj         the object id
%
belief_perceived_at(ObjType, Transform, Threshold, Obj) :-
  belief_existing_object_at(ObjType, Transform, Threshold, Obj),
  belief_class_of(Obj, ObjType), !.

belief_perceived_at(ObjType, Transform, _, Obj) :-
  belief_new_object(ObjType, Obj),
  belief_at_update(Obj, Transform).

%%
belief_perceived_part_at(PartType, Transform, Threshold, Part, Parent) :-
  owl_has(Parent, knowrob:properPhysicalParts, Part),
  belief_existing_object_at(PartType, Transform, Threshold, Part),
  belief_class_of(Part, PartType), !.

belief_perceived_part_at(PartType, Transform, _, Part, Parent) :-
  belief_new_part(PartType, Part, Parent),
  % TODO enforce transform in parent frame
  belief_at_update(Part, Transform).

belief_perceived_pos([DX,DY,DZ], pos(x,P), [X,DY,DZ]) :- X is DX+P, !.
belief_perceived_pos([DX,DY,DZ], pos(z,P), [DX,Y,DZ]) :- Y is DY+P, !.
belief_perceived_pos([DX,DY,DZ], pos(y,P), [DX,DY,Z]) :- Z is DZ+P, !.

denormalize_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_), Out is V*In.
denormalize_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V), Out is V*In.
denormalize_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_), Out is V*In.

center_part_pos(Obj, x, In, Out) :-
  object_dimensions(Obj,_,V,_),
  Out is In - 0.5*V.
center_part_pos(Obj, y, In, Out) :-
  object_dimensions(Obj,_,_,V),
  Out is In - 0.5*V.
center_part_pos(Obj, z, In, Out) :-
  object_dimensions(Obj,V,_,_),
  Out is In - 0.5*V.

belief_part_offset(Parent, PartType, [DX,DY,DZ]) :-
  object_affordance(Parent,Affordance),
  rdfs_individual_of(Affordance, knowrob:'PartOffsetAffordance'),
  owl_property_range_on_subject(Affordance, knowrob:userOfAffordance, AllowedType),
  rdfs_subclass_of(PartType, AllowedType),
  belief_at_id(Affordance, [_,_,[DX,DY,DZ],_]), !.
belief_part_offset(_, _, [0,0,0]).

belief_perceived_part_at_axis(Parent, PartType, norm(Axis,Pos), Part) :- !,
  denormalize_part_pos(Parent, Axis, Pos, Denormalized),
  belief_perceived_part_at_axis(Parent, PartType, pos(Axis,Denormalized), Part).

belief_perceived_part_at_axis(Parent, PartType, pos(Axis,Pos), Part) :-
  center_part_pos(Parent, Axis, Pos, Centered),
  object_frame_name(Parent,ParentFrame),
  belief_part_offset(Parent, PartType, Offset),
  belief_perceived_pos(Offset, pos(Axis,Centered), PerceivedPos),
  belief_perceived_part_at(PartType, [ParentFrame,_,PerceivedPos,
      [0.0, 0.0, 0.0, 1.0]], 0.02, Part, Parent).

%% belief_at(+Obj:iri, ?Transform:list) is semidet.
%% belief_at(+Obj:iri, ?Transform:list, +Instant:time) is semidet.
%
% True if Transform is the active transform of Obj at time Instant (or the current time).
% The transform is specified as
% [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
% If some frame of reference is specified in Transform the necessary
% transformations are made automatically.
%
% @param Obj         the object id
% @param Transform   the transform data
% @param Instant     the time instant RDF iri or timestamp
%
belief_at(Obj, Transform) :-
  current_time(Instant),
  belief_at(Obj, Transform, Instant).

belief_at(Obj, [MapFrame, TargetFrame, Translation, Rotation], Instant) :-
  map_frame_name(MapFrame), !,
  belief_at_global(Obj, [MapFrame, TargetFrame, Translation, Rotation], Instant).

belief_at(Obj, [ReferenceFrame, TargetFrame, Translation, Rotation], Instant) :-
  ground(ReferenceFrame),
  object_frame_name(Ref, ReferenceFrame), !,
  belief_at_relative_to(Obj, Ref, [ReferenceFrame, TargetFrame, Translation, Rotation], Instant).

belief_at(Obj, Transform, Instant) :-
  belief_at_id(Obj, Transform, Instant).

belief_at_id(Obj, Transform) :-
  current_time(Instant),
  belief_at_id(Obj, Transform, Instant).

belief_at_id(Obj, [ReferenceFrame, TargetFrame, Translation, Rotation], Instant) :-
  holds( knowrob:pose(Obj,TransformId), Instant ),
  object_frame_name(Obj, TargetFrame),
  transform_reference_frame(TransformId, ReferenceFrame), 
  transform_data(TransformId, (Translation, Rotation)), !.
  
%% belief_at_relative_to(+Child:iri, +Parent:iri, -RelPose:list) is semidet.
%% belief_at_relative_to(+Child:iri, +Parent:iri, -RelPose:list, +Instant:time) is semidet.
%
% Computes the pose of Child expressed using Parent as the frame of reference.
% This is, for example, useful if it is known that Child will stay fixed in
% Parent frame for a while (e.g., when Child is attached to Parent).
% The pose is specified as
% [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
% @param Child     the child id
% @param Parent    the parent id
% @param RelPose   the pose of Child in Parent frame
% @param Instant     the time instant RDF iri or timestamp
%
belief_at_relative_to(Child, Parent, RelPose) :-
  current_time(Instant),
  belief_at_relative_to(Child, Parent, RelPose, Instant).

belief_at_relative_to(Child, Parent, [ParentFrame, TargetFrame, Translation, Rotation], Instant) :-
  object_frame_name(Parent, ParentFrame),
  belief_at_id(Child, [ParentFrame, TargetFrame, Translation, Rotation], Instant), !.

belief_at_relative_to(Child, Parent, RelPose, Instant) :-
  belief_at_global(Child,  ChildGlobal, Instant),
  belief_at_global(Parent, ParentGlobal, Instant),
  % FIXME: is there a bug in here?
  transform_between(ParentGlobal, ChildGlobal, RelPose).

%% belief_at_global(+Obj:iri, ?GlobalPose:list) is semidet.
%% belief_at_global(+Obj:iri, ?GlobalPose:list, +Instant:time) is semidet.
%
% True if GlobalPose is the pose of Obj expressed in global coordinates at time Instant (or the current time).
% The pose is specified as
% [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
% @param Obj          the object id
% @param GlobalPose   the transform data in map frame
% @param Instant     the time instant RDF iri or timestamp
%
belief_at_global(Obj, GlobalPose) :-
  current_time(Instant),
  belief_at_global(Obj, GlobalPose, Instant).

belief_at_global(Obj, GlobalPose, Instant) :-
  object_frame_name(Obj, ChildFrame),
  holds( knowrob:pose(Obj,TransformId), Instant ),
  transform_data(TransformId, (T,Q)),
  ( transform_parent_object(TransformId, Parent) -> (
    belief_at_global(Parent, GlobalTransform, Instant),
    object_frame_name(Parent, ParentFrame),
    transform_multiply(GlobalTransform, [ParentFrame,ChildFrame,T,Q], GlobalPose)
  ) ; ( map_frame_name(MapFrame), GlobalPose=[MapFrame,ChildFrame,T,Q]) ), !.

transform_parent_object(TransformId, Parent) :-
  rdf_has(TransformId, knowrob:'relativeTo', RelativeTo),
  (( rdf_has(X, knowrob:'eventOccursAt', RelativeTo),
     rdf_has(X, knowrob:'objectActedOn', Object) );
     rdf_has(Object, knowrob:'pose', RelativeTo) ;
     Object = RelativeTo ),
  resolve_temporal_part(Object,Parent), !.

resolve_temporal_part(TemporalPart, Object) :-
  rdf_has(Object, knowrob:'temporalParts', TemporalPart), !.
resolve_temporal_part(Object, Object).
  
  

%% belief_at_update(+Obj:iri, +Transform:list) is semidet.
%% belief_at_update(+Obj:iri, +Transform:list, +RelativeTo:iri) is semidet.
%
% From now on belief that Obj is located at Transform.
% The transform is specified as
% [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
% For global coordinates, the transform can also be specified as
% ([float x, y, z], [float x, y, z, w]).
%
% @param Obj         the object id
% @param Transform   the transform data in map frame
% @param RelativeTo  the reference object of the transform
%
belief_at_update(Obj, (Translation, Rotation)) :- !,
  belief_at_internal(Obj, (Translation, Rotation)),
  belief_republish_objects([Obj]).

belief_at_update(Obj, [MapFrame,_,Translation,Rotation]) :-
  map_frame_name(MapFrame), !, 
  belief_at_update(Obj, (Translation, Rotation)).

belief_at_update(Obj, [ReferenceFrame,_,Translation,Rotation]) :-
  ( object_frame_name(RelativeTo, ReferenceFrame) ; (
    write('WARN: Unable to find entity with frame "'),
    write(ReferenceFrame),
    writeln('", ignoring belief update.'),
    fail
  )), !,
  belief_at_update(Obj, (Translation, Rotation), RelativeTo).

belief_at_update(Obj, TransformData, RelativeTo) :-
  belief_at_internal(Obj, TransformData, RelativeTo),
  belief_republish_objects([Obj]).

%% belief_at_internal(+Obj, +TransformData, +RelativeTo) is det.
%
belief_at_internal(Obj, TransformData, RelativeTo) :-
  belief_at_internal_(Obj, RelativeTo, TransformData, _), !.

belief_at_internal(Obj, TransformData) :-
  belief_at_internal_(Obj, 'http://knowrob.org/kb/knowrob.owl#MapFrame', TransformData, _), !.

belief_at_internal_(Obj, RelativeTo, (Translation, Rotation), TransformId) :-
  belief_new_pose((Translation, Rotation), TransformId, RelativeTo),
  current_time(Now),
  ( rdf_has(Obj, knowrob:pose, OldPose) ->
    assert_temporal_part_end(Obj, knowrob:pose, OldPose, Now, belief_state) ;
    true
  ),
  assert_temporal_part(Obj, knowrob:pose,
    nontemporal(TransformId), Now, belief_state).

belief_new_pose(([X,Y,Z], [QW,QX,QY,QZ]), TransformId, 'http://knowrob.org/kb/knowrob.owl#MapFrame') :- !,
  belief_new_pose(([X,Y,Z], [QW,QX,QY,QZ]), TransformId).
belief_new_pose(([X,Y,Z], [QW,QX,QY,QZ]), TransformId, Frame) :-
  belief_new_pose(([X,Y,Z], [QW,QX,QY,QZ]), TransformId),
  rdf_assert(TransformId, knowrob:'relativeTo', Frame, belief_state).

belief_new_pose(([X,Y,Z], [QW,QX,QY,QZ]), TransformId) :-
  rdf_unique_id('http://knowrob.org/kb/knowrob.owl#Pose', TransformId),
  atomic_list_concat([X,Y,Z], ' ', Translation),
  atomic_list_concat([QW,QX,QY,QZ], ' ', Quaternion),
  rdf_assert(TransformId, rdf:type, knowrob:'Pose', belief_state),
  rdf_assert(TransformId, knowrob:'translation', literal(type(xsd:string,Translation)), belief_state),
  rdf_assert(TransformId, knowrob:'quaternion', literal(type(xsd:string,Quaternion)), belief_state).
  

%% belief_republish_objects(+ObjectIds) is det
%
belief_republish_objects(ObjectIds) :- mark_dirty_objects(ObjectIds).
