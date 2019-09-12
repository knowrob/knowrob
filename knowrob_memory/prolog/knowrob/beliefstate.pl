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
      belief_new_object/2,
      belief_new_part/3,
      belief_at_update/2,
      belief_existing_objects/1,    % set of known objects in the belief state
      belief_existing_objects/2,
      belief_existing_object_at/4,  % query known object near some pose
      belief_perceived_at/4,        % convinience rule to be called by perception system to inform about perceptions
      belief_perceived_part_at/5
    ]).
/** <module> Maintaining beliefs about objects.
  
  @author Daniel Beßler
  @license BSD
*/

:- use_module(library('lists')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/objects')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).

:-  rdf_meta
    belief_new_object(r,+),
    belief_new_part(r,r,r),
    belief_at_update(r,+),
    belief_existing_objects(-,t),
    belief_existing_object_at(r,+,+,r),
    belief_perceived_at(r,+,+,r),
    belief_perceived_part_at(r,+,+,r,r).

%% belief_new_object(+ObjType:iri, -Obj:iri) is det.
%
% Asserts a new object to the "belief_state" RDF graph.
%
% @param ObjType the type of the new object
% @param Obj the object asserted
%
belief_new_object(ObjType, Obj) :-
  object_assert(ObjType, Obj, belief_state).

%%
belief_new_part(PartType, Part, Parent) :-
  object_assert(PartType, Part),
  % FIXME: dulify
  rdf_assert(Parent, knowrob:properPhysicalParts, Part, belief_state).
  
%% belief_at(+Obj:iri, ?Pose:term) is semidet
%% belief_at(+Obj:iri, ?Pose:term, +Interval) is semidet
%
% Wraps knowrob_objects:object_pose.
% 
belief_at(Obj,Pose) :-
  object_pose(Obj,Pose).

belief_at(Obj,Pose,Instant) :-
  object_pose(Obj,Pose,Instant).

%% belief_existing_objects(-ObjectIds:list) is det.
%
% Returns a list of perceived objects that are known to the KB.
%
% @param ObjectIds the object names
%
belief_existing_objects(ObjectIds) :-
  belief_existing_objects(ObjectIds, [dul:'PhysicalObject']).

belief_existing_objects(ObjectIds, ObjectTypes) :-
  findall(J, (
      rdf(J, _, _, belief_state),
      member(T, ObjectTypes),
      rdfs_individual_of(J, T)
  ), X),
  list_to_set(X,ObjectIds).

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
  kb_type_of(Obj, ObjType), !.
belief_class_of(Obj, NewObjType) :-
  current_time(Now),
  ignore(once((
      % withdraw old beliefs about object type
      kb_type_of(Obj, CurrObjType),
      rdfs_subclass_of(CurrObjType, Parent),
      rdfs_subclass_of(NewObjType, Parent),
      %%
      mem_triple_stop(Obj,rdf:type,CurrObjType,Now),
      rdf_retractall(Obj,rdf:type,CurrObjType)
  ))),
  mem_store_triple(Obj,rdf:type,NewObjType,_,Now),
  rdf_assert(Obj,rdf:type,NewObjType).

%% belief_at_update(+Obj:iri, +Transform:list) is semidet.
%
% From now on belief that Obj is located at Transform.
% The transform is specified as
% [atom reference_frame, atom target_frame, [float x, y, z], [float x, y, z, w]],
% where translations are given in meters.
%
% @param Obj         the object id
% @param Transform   the transform data in map frame
%
belief_at_update(Obj, [RefFrame,ObjFrame,T,Q]) :-
  object_frame_name(Obj,ObjFrame),
  object_pose_update(Obj,[RefFrame,ObjFrame,T,Q]).

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
belief_existing_object_at(_ObjType, [RefFrame,ObjFrame,T0,Q0], Dmax, Obj) :-
  % FIXME: super slow for large belief state
  % - maybe use octree to find the nearest neighbour?
  rdf(Obj, rdf:type, owl:'NamedIndividual', belief_state),
  rdfs_individual_of(Obj, dul:'PhysicalObject'),
  current_object_pose(Obj,[RefFrame,ObjFrame,T1,Q1]),
  transform_close_to([RefFrame,ObjFrame,T0,Q0],
                     [RefFrame,ObjFrame,T1,Q1], Dmax),!.

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
  % FIXME: dulify
  owl_has(Parent, knowrob:properPhysicalParts, Part),
  belief_existing_object_at(PartType, Transform, Threshold, Part),
  belief_class_of(Part, PartType), !.

belief_perceived_part_at(PartType, Transform, _, Part, Parent) :-
  belief_new_part(PartType, Part, Parent),
  % TODO enforce transform in parent frame
  belief_at_update(Part, Transform).

