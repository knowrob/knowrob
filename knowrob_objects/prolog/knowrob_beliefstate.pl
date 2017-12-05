/** <module> knowrob_beliefstate: maintaining symbolic beliefs about perceived objects.

  Copyright (C) 2017 Daniel Beßler, Mihai Pomarlan

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

  @author Daniel Beßler, Mihai Pomarlan
  @license BSD
*/

:- module(knowrob_beliefstate,
    [
      belief_existing_objects/1,    % set of known objects in the belief state
      belief_new_object/2,          % assert a new object in the belief state
      belief_at_update/2,           % assign new pose to object
      belief_at_update/3,
      belief_at/2,                  % query the current pose of an object
      belief_at_global/2,           % query the current pose of an object in map frame
      belief_at_relative_to/3,      % query the current pose of an object relative to some parent object
      belief_class_at_location/4,   % query for existing object at location
      belief_at_internal/2,         % TODO: these should not be exposed
      belief_at_internal/3,
      belief_perceived_at/4,        % convinience rule to be called by perception system to inform about perceptions
      belief_dirty_object/1,       % causes marker messages to be generated
      belief_forget/0
    ]).

:- use_module(library('lists')).
:- use_module(library('util')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('owl')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_math')).
:- use_module(library('knowrob_objects')).
:- use_module(library('random')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:-  rdf_meta
    belief_new_object(r,r),
    belief_at(r,+,r),
    belief_at(r,+),
    belief_at_update(r,+,r),
    belief_at_update(r,+),
    belief_at_internal(r,+,r),
    belief_at_internal(r,+),
    belief_at_global(r,-),
    belief_at_relative_to(r,r,-),
    belief_class_at_location(r,+,+,r),
    belief_perceived_at(r,+,+,r),
    belief_dirty_object(t).


% TODO
% - don't use srdl2comp! should use more general property
% - ROS param TranThreshold for identity resolution
% - don't expect map frame

quote_id(X, O) :-
  atom_string(X, Oxx),
  string_concat('"', Oxx, Ox),
  string_concat(Ox, '"', O).

%% belief_dirty_object(-ObjectIds) is det.
%
% TODO: use jpl, or cpp ROS interface
belief_dirty_object(ObjectIds) :-
  findall(O, (member(X, ObjectIds), quote_id(X, O)), Os),
  atomic_list_concat(Os, ',', OsS),
  atom_string("'[", LB),
  string_concat(LB, OsS, PS),
  atom_string("]'", RB),
  string_concat(PS, RB, ParStr),
  atom_string("rosservice call /object_state_publisher/mark_dirty_object ", CmdKern),
  string_concat(CmdKern, ParStr, Cmd),
  thread_create(shell(Cmd), _, []).

%% belief_existing_objects(-ObjectIds) is det.
%
% Returns a list of strings representing the individuals of type MechanicalPart that are known to the KB.
%
% @param ObjectIds    [anyURI*], the object ids
%
belief_existing_objects(UniqueObjectIds) :-
  findall(J, (
      rdf(J, _, _, belief_state),
      rdfs_individual_of(J, knowrob:'SpatialThing')), ObjectIds),
  list_to_set(ObjectIds,UniqueObjectIds).

%% belief_forget is det.
%
% Retracts everything asserted to the belief_state RDF graph.
%
belief_forget :-
  forall( rdf(J, _, _, belief_state),
          retractall(J,_,_) ).

%% belief_new_object(+Cls, +Obj) is det.
%
belief_new_object(ObjectType, Obj) :-
  rdf_instance_from_class(ObjectType, belief_state, Obj),
  rdf_assert(Obj, rdf:type, owl:'NamedIndividual', belief_state),
  % set TF frame to object name
  rdf_split_url(_, ObjName, Obj),
  rdf_assert(Obj, srdl2comp:'urdfName', literal(ObjName), belief_state),
  ignore(once((
    %% HACK get this info from somewhere else!
    rdfs_individual_of(Map, knowrob:'SemanticEnvironmentMap'),
    rdf_assert(Obj, knowrob:'describedInMap', Map, belief_state)
  ))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Beliefs about the class of things

belief_class_of(Obj, ObjType) :-
  % current classification matches beliefs
  rdfs_type_of(Obj, ObjType), !.
belief_class_of(Obj, NewObjType) :-
  current_time(Now),
  ignore(once((
      rdfs_type_of(Obj, CurrObjType),
      rdfs_subclass_of(CurrObjType, Parent),
      rdfs_subclass_of(NewObjType, Parent),
      assert_temporal_part_end(Obj, rdf:type, CurrObjType, Now, belief_state)
  ))),
  assert_temporal_part(Obj, rdf:type, nontemporal(NewObjType), Now, belief_state).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Beliefs about the spatial location of things

%% belief_class_at_location(+ObjectType, +Transform, +Thresholds, -ObjectId) is det.
%
% Checks whether one of the already known objects of ObjectType is located close to Transform.
% The check uses TranThreshold as a threshold for comparing translation distance, and similarly for
% rotation and RotThreshold.
%
% Transform is expected to be of the form [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]]
%
% If such an object is found, its id is returned via ObjectId.
%
% @param ObjectType      anyURI, the object type
% @param Transform       the transform data
% @param Thresholds   a distance below which two translations are thought to be the same, and
%                     a distance below which two rotations are thought to be the same;
%                     NOTE: this threshold is interpreted as a euclidean distance threshold in a space of quaternions where +Quat and -Quat are the same.
% @param ObjectId        anyURI, the object id
%
belief_class_at_location(ObjectType, Transform, Thresholds, ObjectId) :-
  rdfs_individual_of(ObjectId, ObjectType),
  belief_object_at_location(ObjectId, Transform, Thresholds).

belief_object_at_location(ObjectId, NewPose, Dmax) :-
  belief_at(ObjectId, OldPose),
  transform_close_to(NewPose, OldPose, Dmax).

%% belief_perceived_at(+ObjectType, +TransformData, +Threshold, -Obj)
%
belief_perceived_at(ObjectType, TransformData, Threshold, Obj) :-
  belief_existing_object_at(ObjectType, TransformData, Threshold, Obj),
  belief_class_of(Obj, ObjectType), !.
belief_perceived_at(ObjectType, TransformData, _, Obj) :-
  belief_new_object(ObjectType, Obj),
  belief_at_update(Obj, TransformData).

belief_existing_object_at(ExpectedType, TransformData, Threshold, Obj) :-
  % check for typed object
  belief_class_at_location(ExpectedType, TransformData, Threshold, Obj), !.
belief_existing_object_at(ExpectedType, TransformData, Threshold, Obj) :-
  % check for any type
  % TODO: use octree to find the nearest object perceived before
  %       instead of going through complete belief state here.
  %       Then get rid of above case.
  get_known_object_ids(KnownObjects),
  member(Obj,KnownObjects),
  \+ rdfs_individual_of(Obj, ExpectedType),
  belief_object_at_location(Obj, TransformData, Threshold), !.

%% belief_at(+Obj, +TransformData) is det.
%
% Returns the currently active transform of Obj. The transform is returned as
% [string reference_frame, string target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
belief_at(Obj, ['map', TargetFrame, Translation, Rotation]) :-
  belief_at_global(Obj, ['map', TargetFrame, Translation, Rotation]), !.
belief_at(Obj, [ReferenceFrame, TargetFrame, Translation, Rotation]) :-
  ground(ReferenceFrame), !,
  rdf_has(Ref, srdl2comp:'urdfName', literal(ReferenceFrame)),
  belief_at_relative_to(Child, Ref, [ReferenceFrame, TargetFrame, Translation, Rotation]), !
belief_at(Obj, [ReferenceFrame, TargetFrame, Translation, Rotation]) :-
  holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', TransformId),
  transform_reference_frame(TransformId, ReferenceFrame), 
  transform_data(TransformId, (Translation, Rotation)).
  
%% belief_at_relative_to(+Child, +Parent, -RelPose) is det.
%
belief_at_relative_to(Child, Parent, RelPose) :-
  % FIXME: pose potentially computed twice (i.e., below clause)
  holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', RelPose),
  rdf_has(RelPose, knowrob:'relativeTo', Parent), !.
belief_at_relative_to(Child, Parent, RelPose) :-
  belief_at_global(Child,  ChildGlobal),
  belief_at_global(Parent, ParentGlobal),
  transform_compute_relative(ChildGlobal, ParentGlobal, RelPose).

%% belief_at_global(+Obj, -GlobalPose) is det.
%
belief_at_global(Obj, GlobalPose) :-
  rdf_has(Obj, srdl2comp:'urdfName', literal(ChildFrame)),
  holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', TransformId),
  transform_data(TransformId, (T,Q)),
  ( rdf_has(TransformId, knowrob:'relativeTo', Parent) -> (
    % FIXME: TransformId could be relative to Parent in the past, but not anymore in the present.
    %  e.g., object was perceived in camera frame 2min ago, but
    %  robot moved until then.
    %  in this case object_pose_at_time must be used?
    belief_at_global(Parent, GlobalTransform),
    rdf_has(Parent, srdl2comp:'urdfName', literal(ParentFrame)),
    transform_multiply(GlobalTransform, [ParentFrame,ChildFrame,T,Q], GlobalPose)
  ) ; GlobalPose=['map',ChildFrame,T,Q]).

%% belief_at_update(+Obj, +TransformData) is det.
%
belief_at_update(Obj, (Translation, Rotation)) :- !,
  belief_at_internal(Obj, (Translation, Rotation)),
  belief_dirty_object([Obj]).
belief_at_update(Obj, ['map',_,Translation,Rotation]) :- !, 
  belief_at_update(Obj, (Translation, Rotation)).
belief_at_update(Obj, [ReferenceFrame,_,Translation,Rotation]) :-
  ( rdf_has(RelativeTo, srdl2comp:'urdfName', literal(ReferenceFrame)) ; (
    write('WARN: Unable to find entity with TF frame "'), write(ReferenceFrame),
    writeln('", ignoring belief update.'),
    fail
  )),
  belief_at_update(Obj, (Translation, Rotation), RelativeTo).
belief_at_update(Obj, TransformData, RelativeTo) :-
  belief_at_internal(Obj, TransformData, RelativeTo),
  belief_dirty_object([Obj]).

%% belief_at_internal(+Obj, +TransformData, +RelativeTo) is det.
%
belief_at_internal(Obj, TransformData, RelativeTo) :-
  belief_at_internal_(Obj, TransformData, TransformId), !,
  rdf_assert(TransformId, knowrob:'relativeTo', RelativeTo).
belief_at_internal(Obj, TransformData) :-
  belief_at_internal_(Obj, TransformData, _).
belief_at_internal_(Obj, (Translation, Rotation), TransformId) :-
  forall(rdf_has(Obj, knowrob:'pose', Pose),
         rdf_retractall(Pose, _, _)),
  rdf_retractall(Obj, knowrob:'pose', _),
  create_transform(Translation, Rotation, TransformId),
  rdf_assert(Obj, knowrob:'pose', TransformId).
