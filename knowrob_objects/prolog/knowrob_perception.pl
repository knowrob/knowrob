/** <module> Methods for creating object instances based on perceptual information

  Copyright (C) 2011-2014 Moritz Tenorth
  Copyright (C) 2017      Daniel Beßler
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

  @author Moritz Tenorth, Daniel Beßler
  @license BSD
*/

:- module(knowrob_perception,
    [
      create_visual_perception/1,
      create_visual_perception/2,
      perception_set_object/2,
      perception_set_pose/2,
      object_detection/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Asserting visual perception events

%% create_perception_instance(PerceptionTypes, Perception) is det.
%% create_perception_instance(PerceptionTypes, ModelTypes, Perception) is det.
%
create_visual_perception(Perception) :-
  rdf_instance_from_class('http://knowrob.org/kb/knowrob.owl#VisualPerception', Perception),
  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).
create_visual_perception(ModelType, Perception) :-
  create_visual_perception(Perception),
  % set the perceivedUsingModel relation
  rdf_assert(Perception, knowrob:perceivedUsingModel, ModelType).

%% perception_set_object(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
% @param Object        Object instance
% @param Perception    Perception instance
% 
perception_set_object(Object, Perception) :-
  % add perception to linked list of object detections,
  ( rdf_has(Object, knowrob:latestDetectionOfObject, Prev) -> (
    rdf_update(Object, knowrob:latestDetectionOfObject, Prev, object(Perception)),
    rdf_assert(Perception, knowrob:previousDetectionOfObject, Prev)
  ) ; (
    rdf_assert(Object, knowrob:latestDetectionOfObject, Perception)
  )),
  rdf_assert(Perception, knowrob:objectActedOn, Object).

%% perception_set_pose(+Perception, +Pose) is det.
%
% Set the pose of an object perception to the value given as PoseList
%
% @param Perception  Perception instance
% @param Pose        Pose of the perceived object
% 
perception_set_pose(Pose, Pose) :-
  atom(Pose),
  rdfs_individual_of(Pose, knowrob:'Pose'), !.
perception_set_pose(Perception, [ReferenceFrame, _, Translation, Rotation]) :-
  rdf_has(Ref, srdl2comp:'urdfName', literal(ReferenceFrame)),
  create_transform(Translation, Rotation, TransformId),
  rdf_assert(TransformId, knowrob:'relativeTo', Ref), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Queries about object detections

%% object_detection(+Object, ?Time, -Detection) is nondet.
%
% Find all detections of the Object that are valid at time point Time
%
% @param Object     Object instance of interest
% @param Time       Time point of interest. If unbound, all detections of the object are returned.
% @param Detection  Detections of the object that are assumed to be valid at time Time
%
object_detection(Object, Time, Detection) :-
    rdf_has(Detection, knowrob:objectActedOn, Object),
    rdfs_individual_of(Detection,  knowrob:'VisualPerception'),
    interval(Detection, DetectionInterval),
    ( var(Time)
      -> Time = DetectionInterval
      ;  interval_during(Time, DetectionInterval)
    ).

%% latest_detection_of_instance(+Object, -LatestDetection) is nondet.
%
% Get the lastest detection of the object instance Object
%
% A detection is an instance of MentalEvent, i.e. can be a perception
% process as well as an inference result
%
% @param Object          An object instance
% @param LatestDetection Latest MentalEvent associated with this instance
%
latest_detection_of_instance(Object, LatestDetection) :-

  ((rdf_has(Object, knowrob:latestDetectionOfObject, LatestDetection),!);

   (% old version without linked list of detections
    findall([D_i,Object,St], (rdf_has(D_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection))).

%% latest_detection_of_type(+Type, -LatestDetection) is nondet.
%
% Get the lastest detection of an object of type Type
%
% A detection is an instance of MentalEvent, i.e. can be a perception
% process as well as an inference result
%
% @param Object          An object type
% @param LatestDetection Latest MentalEvent associated with any instance of this type
%
latest_detection_of_type(Type, LatestDetection) :-

    findall([D_i,Object,St], (rdfs_individual_of(Object, Type),
                              latest_detection_of_instance(Object, D_i),
                              detection_starttime(D_i, St)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Latest),
    nth0(0, Latest, LatestDetection).


%% latest_perception_of_type(+Type, -LatestPerception) is nondet.
%
% Get the lastest perception of an object of type Type
%
% @param Object          An object type
% @param LatestPerception Latest MentalEvent associated with any instance of this type
%
latest_perception_of_type(Type, LatestPerception) :-

    findall([P_i,Object,St], (rdfs_individual_of(Object, Type),
                              rdf_has(P_i, knowrob:objectActedOn, Object),
                              rdfs_individual_of(P_i,  knowrob:'VisualPerception'),
                              detection_starttime(P_i, St)), Perceptions),

    predsort(compare_object_detections, Perceptions, Psorted),

    % compute the homography for the newest perception
    nth0(0, Psorted, Latest),
    nth0(0, Latest, LatestPerception).


%% latest_perceptions_of_types(+Type, -LatestPerceptions) is nondet.
%
% Get the lastest perceptions of all objects of type Type
%
% @param Object          An object type
% @param LatestPerceptions Latest MentalEvents associated with instances of this type
%
latest_perceptions_of_types(Type, LatestPerceptions) :-

    findall(Obj, rdfs_individual_of(Obj, Type), Objs),

    findall(LatestDetection,
            ( member(Object, Objs),
              latest_detection_of_instance(Object, LatestDetection),
              rdfs_individual_of(LatestDetection, knowrob:'VisualPerception') ),
            LatestPerceptions).



%% latest_inferred_object_set(-Object) is nondet.
%
% Ask for the objects inferred in the last inference run
%
% @param Objects   Set of object instances inferred in the latest inference run
%
latest_inferred_object_set(Objects) :-

    findall([D_i,_,St],  (rdfs_individual_of(D_i,  knowrob:'Reasoning'),
                          rdf_has(Inf, knowrob:probability, InfProb),
                          term_to_atom(Prob, InfProb),
                          >(Prob, 0),
                          detection_starttime(D_i, St)), Inferences),

    predsort(compare_object_detections, Inferences, Psorted),

    % compute the newest perception
    nth0(0, Psorted, Latest),
    nth0(0, Latest, LatestInf),

    % find other inferences performed at the same time
    findall(OtherInf, (rdf_has(LatestInf, knowrob:'startTime', St), rdf_has(OtherInf, knowrob:'startTime', St)), OtherInfs),

    predsort(compare_inferences_by_prob, OtherInfs, SortedInfs),

    findall(Obj, (member(Inf, SortedInfs), rdf_has(Inf, knowrob:'objectActedOn', Obj)), Objects).


%% latest_inferred_object_types(-ObjectTypes) is nondet.
%
% Ask for the object types inferred in the last inference run
%
% @param ObjectTypes   Set of object types inferred in the latest inference run
%
latest_inferred_object_types(ObjectTypes) :-

    latest_inferred_object_set(Objects),
    findall(ObjT, (member(Obj, Objects), rdf_has(Obj, rdf:type, ObjT)), ObjectTypes).

%% detection_starttime(+Detection, -StartTime) is nondet.
%
% Determine the start time of an object detection as numerical value.
% Simply reads the asserted knowrob:startTime and transforms the timepoint
% into a numeric value.
%
% @param Detection  Instance of an event with asserted startTime
% @param StartTime  Numeric value describing the start time
%
detection_starttime(Detection, StartTime) :-
  number(Detection), StartTime = Detection ;
  
  % start time is asserted
  rdf_triple(knowrob:startTime, Detection, StartTtG),
  rdf_split_url(_, StartTt, StartTtG),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom),! ;

  rdf_split_url(_, StartTt, Detection),
  atom_concat('timepoint_', StartTAtom, StartTt),
  term_to_atom(StartTime, StartTAtom).


%% detection_endtime(+Detection, -EndTime) is nondet.
%
% Determine the end time of an object detection as numerical value.
% If the knowrob:endTime is asserted, it is read and and transformed
% into a numeric value. Otherwise, the predicate searches for later
% perceptions of the same object and takes the startTime of the first
% subsequent detection as the endTime of the current detection. If
% there is neither an asserted endTime nor any later detection of the
% object, it is assumed that the observation is still valid and the
% current time + 1s is returned (to avoid problems with time glitches).
%
% @param Detection  Instance of an event
% @param EndTime    Numeric value describing the ent time
%
detection_endtime(Detection, EndTime) :-
  number(Detection), EndTime = Detection ;

  % end time is asserted
  rdf_triple(knowrob:endTime, Detection, EndTtG),
  rdf_split_url(_, EndTt, EndTtG),
  atom_concat('timepoint_', EndTAtom, EndTt),
  term_to_atom(EndTime, EndTAtom),!;

  % search for later detections of the object
  ( rdf_has(LaterDetection, knowrob:previousDetectionOfObject, Detection),
    rdf_triple(knowrob:startTime, LaterDetection, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % check if the object has been destroyed in the meantime
  ( rdf_has(Detection, knowrob:objectActedOn, Object),
    rdf_has(Destruction, knowrob:inputsDestroyed, Object),
    Destruction \= Detection,
    rdfs_individual_of(Destruction,  knowrob:'PhysicalDestructionEvent'),
    rdf_triple(knowrob:startTime, Detection, StT),
    rdf_triple(knowrob:startTime, Destruction, EndTtG),
    rdf_triple(knowrob:after, StT, EndTtG),
    rdf_split_url(_, EndTt, EndTtG),
    atom_concat('timepoint_', EndTAtom, EndTt),
    term_to_atom(EndTime, EndTAtom),! );

  % otherwise take the current time (plus a second to avoid glitches)
  ( get_time(ET), EndTime is ET + 1.0).

%% compare_object_detections(-Delta, +P1, +P2) is det.
%
% Sort detections by their start time
%
% @param Delta  One of '>', '<', '='
% @param P1     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
% @param P2     List [_, _, Time] as used in latest_detection_of_instance, latest_detection_of_type, latest_inferred_object_set
%
compare_object_detections(Delta, P1, P2) :-

    nth0(2, P1, St1),
    nth0(2, P2, St2),
    compare(Delta, St2, St1).
