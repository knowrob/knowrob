/** <module> Methods for visualizing parts of the knowledge base

  Copyright (C) 2011-2015 Moritz Tenorth, Daniel Beßler, Asil Kaan Bozcuoğlu
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

  @author Moritz Tenorth, Daniel Beßler, Asil Kaan Bozcuoğlu
  @license BSD
*/

:- module(knowrob_vis,
    [
      visualisation_canvas/0,
      visualisation_server/0,
      clear_canvas/0,
      clear_trajectories/0,
      camera_pose/2,
      camera_transform/1,
      add_agent_visualization/1,
      add_agent_visualization/2,
      add_agent_visualization/3,
      add_agent_visualization/4,
      add_agent_visualization/5,
      add_stickman_visualization/1,
      add_stickman_visualization/2,
      add_stickman_visualization/3,
      add_stickman_visualization/4,
      add_stickman_visualization/5,
      remove_agent_visualization/1,
      remove_agent_visualization/2,
      add_object/1,
      add_object/2,
      add_objects/1,
      add_object_with_children/1,
      add_object_with_children/2,
      update_object/1,
      update_object/2,
      update_object_with_children/1,
      update_object_with_children/2,
      remove_object/1,
      remove_object_with_children/1,
      add_text/3,
      add_text/1,
      add_speech_bubble/2,
      add_speech_bubble/3,
      add_speech_bubble/4,
      add_agent_speech_bubble/3,
      add_agent_speech_bubble/4,
      add_agent_speech_bubble/5,
      add_mesh/2,
      add_mesh/3,
      add_mesh/4,
      add_mesh/5,
      highlight_object/1,
      highlight_object_mesh/1,
      highlight_object_mesh/2,
      highlight_object/2,
      highlight_object/5,
      highlight_object_with_children/1,
      highlight_object_with_children/2,
      highlight_trajectory/4,
      remove_highlight/1,
      remove_mesh_highlight/1,
      remove_highlight_with_children/1,
      reset_highlight/0,
      add_avg_trajectory/5,
      add_trajectory/3,
      add_trajectory/4,
      add_trajectory/5,
      remove_trajectory/1,
      add_pointer/3,
      diagram_canvas/0,
      clear_diagram_canvas/0,
      add_diagram/9,
      add_diagram/10,
      add_barchart/3,
      add_piechart/3,
      add_timeline/5,
      remove_diagram/1,
      trajectory_length/4,
      task_tree_canvas/0,
      update_task_tree/3,
      remove_task_tree/0
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta add_object(r),
            camera_pose(r,r),
            camera_transform(r),
            add_agent_visualization(r),
            add_agent_visualization(r,r),
            add_agent_visualization(r,r,r),
            add_agent_visualization(r,r,r,r),
            add_agent_visualization(r,r,r,r,r),
            add_stickman_visualization(r),
            add_stickman_visualization(r,r),
            add_stickman_visualization(r,r,r),
            add_stickman_visualization(r,r,r,r),
            add_stickman_visualization(r,r,r,r,r),
            remove_agent_visualization(r),
            remove_agent_visualization(r,r),
            add_object(r,r),
            add_object_with_children(r),
            add_object_with_children(r,r),
            update_object(r,r),
            update_object_with_children(r),
            update_object_with_children(r,r),
            remove_object(r),
            remove_object_with_children(r),
            add_text(r,r,r),
            add_text(r),
            add_speech_bubble(+,+),
            add_speech_bubble(+,+,+),
            add_speech_bubble(+,+,+,+),
            add_agent_speech_bubble(+,+,+),
            add_agent_speech_bubble(+,+,+,+),
            add_agent_speech_bubble(+,+,+,+,+),
            add_mesh(+,+),
            add_mesh(+,+,+),
            add_mesh(+,+,+,+),
            add_mesh(+,+,+,+,+),
            highlight_object(r),
            highlight_object_mesh(r),
            highlight_object_mesh(r,+),
            highlight_object(r,?),
            highlight_object(r,?,?,?,?,?),
            highlight_object_with_children(r),
            highlight_object_with_children(r,?),
            highlight_trajectory(r,r,r,?),
            remove_highlight(r),
            remove_mesh_highlight(r),
            remove_highlight_with_children(r),
            add_diagram(+,+,+,+,+,+,+,+,+),
            remove_diagram(+),
            add_avg_trajectory(r,r,r,r,r),
            add_trajectory(r,r,r),
            add_trajectory(r,r,r,+),
            add_trajectory(r,r,r,+,+),
            add_pointer(r,r,r),
            remove_trajectory(r),
            trajectory_length(r,r,r,+).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Visualization server management
%

%% visualisation_server is det.
%
% Launch the visualization server
%
visualisation_server :-
  visualisation_server(_).

visualisation_server(WebServer) :-
    (\+ current_predicate(v_server, _)),
    jpl_new('org.knowrob.vis.WebServer', [], WebServer),
    jpl_list_to_array(['org.knowrob.vis.WebServer'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [WebServer, Arr], _),
    assert(v_server(WebServer)),!.
visualisation_server(WebServer) :-
    current_predicate(v_server, _),
    v_server(WebServer).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Visualization canvas management
%

%% visualisation_canvas is det.
%
% Launch the visualization canvas
%
visualisation_canvas :-
  visualisation_canvas(_).
  
visualisation_canvas(Canvas) :-
    (\+ current_predicate(v_canvas, _)),
    jpl_call('org.knowrob.vis.MarkerVisualization', get, [], Canvas),
    jpl_list_to_array(['org.knowrob.vis.MarkerVisualization'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Canvas, Arr], _),
    assert(v_canvas(Canvas)),!.
visualisation_canvas(Canvas) :-
    current_predicate(v_canvas, _),
    v_canvas(Canvas).


%% clear_canvas is det.
%
% Completely clears the scene
%
clear_canvas :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'clear', [], _).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Setting camera pose in canvas
%
% Accepts Position and quartenion 
camera_pose(Position, Orientation) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Position, PositionArr),
    lists_to_arrays(Orientation, OrientationArr),
    jpl_call(Canvas, 'setCameraPose', [PositionArr, OrientationArr], _).

% Accepts transform matrix
camera_transform(Transform) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Transform, TransformArr),
    jpl_call(Canvas, 'setCameraTransform', [TransformArr], _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Agents
%

%% add_agent_visualization(+Individual) is det.
%% add_agent_visualization(+Individual, +Timepoint) is det.
%% add_agent_visualization(+Identifier, +Individual, +Timepoint) is det.
%% add_agent_visualization(+Identifier, +Individual, +Timepoint, +Suffix) is det.
%% add_agent_visualization(+Identifier, +Individual, +Timepoint, +Suffix, +TfPrefix) is det.
%
% Reads joint poses from logged tf data and visualizes them in the
% Web-based canvas.
%

add_agent_visualization(Individual) :-
    get_timepoint(Timepoint),
    add_agent_visualization('_', Individual, Timepoint, '').

add_agent_visualization(Individual, Timepoint) :-
    add_agent_visualization('_', Individual, Timepoint, '').

add_agent_visualization(Identifier, Individual, Timepoint) :-
    add_agent_visualization(Identifier, Individual, Timepoint, '').
    
add_agent_visualization(Identifier, Individual, Timepoint, Suffix) :-    
    robot_tf_prefix(Individual,TfPrefix),
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'visualizeAgent', [Identifier,Individual,Timepoint,Suffix,TfPrefix,0], _).
    
add_agent_visualization(Identifier, Individual, Timepoint, Suffix, TfPrefix) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'visualizeAgent', [Identifier,Individual,Timepoint,Suffix,TfPrefix,0], _).

%% add_stickman_visualization(+Individual) is det.
%% add_stickman_visualization(+Individual, +Timepoint) is det.
%% add_stickman_visualization(+Identifier, +Individual, +Timepoint) is det.
%% add_stickman_visualization(+Identifier, +Individual, +Timepoint, +Suffix) is det.
%% add_stickman_visualization(+Identifier, +Individual, +Timepoint, +Suffix, +Prefix) is det.
%
% Reads joint poses from logged tf data and visualizes them in the
% Web-based canvas.
%

add_stickman_visualization(Individual) :-
    get_timepoint(Timepoint),
    add_stickman_visualization('_', Individual, Timepoint, '').

add_stickman_visualization(Individual, Timepoint) :-
    add_stickman_visualization('_', Individual, Timepoint, '').

add_stickman_visualization(Identifier, Individual, Timepoint) :-
    add_stickman_visualization(Identifier, Individual, Timepoint, '').
    
add_stickman_visualization(Identifier, Individual, Timepoint, Suffix) :-
    robot_tf_prefix(Individual,TfPrefix),
    add_stickman_visualization(Identifier, Individual, Timepoint, Suffix,TfPrefix).

    
add_stickman_visualization(Identifier, Individual, Timepoint, Suffix, TfPrefix) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'visualizeAgent', [Identifier,Individual,Timepoint,Suffix,TfPrefix,1], _).

    
%% remove_agent_visualization(+Individual) is det.
%% remove_agent_visualization(+Identifier, +Individual) is det.
%
% Removes agent pose visualizations from the visualization canvas.
%
remove_agent_visualization(Individual) :-
    remove_agent_visualization('_', Individual).
remove_agent_visualization(Identifier, Individual) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeAgent', [Identifier,Individual], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Add / update / remove objects and trajectories
%

%% add_object(+Identifier) is nondet.
%
% Add object to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object(Identifier) :-
    get_timepoint(Time),
    add_object(Identifier, Time).


%% add_object(+Identifier, +Time) is nondet.
%
% Add object to the scene with its position at time 'Time'
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object(Identifier, Time) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addObject', [Identifier, Time], _).


%% add_objects(+Identifiers) is nondet.
%
% Add all elements of the list 'objects' to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_objects(Identifiers) :-

    get_timepoint(Time),
    jpl_list_to_array(Identifiers, IdArray),

    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addObjects', [IdArray, Time], _).



%% add_object_with_children(+Identifier)
%
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object_with_children(Identifier) :-
    get_timepoint(Time),
    add_object_with_children(Identifier, Time).


%% add_object_with_children(+Identifier, +Time)
%
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap, with their positions at time 'Time'
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object_with_children(Identifier, Time) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addObjectWithChildren', [Identifier, Time], _).



%% update_object(+Identifier) is nondet.
%
% Update object in the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
update_object(Identifier) :-
    get_timepoint(Time),
    update_object(Identifier, Time).


%% update_object(+Identifier, +Time) is nondet.
%
% Update object in the scene with its position at time 'Time'
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
update_object(Identifier, Time) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'updateObject', [Identifier, Time], _).



%% update_object_with_children(+Identifier)
%
% Updates object in the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
update_object_with_children(Identifier) :-
    get_timepoint(Time),
    update_object_with_children(Identifier, Time).


%% update_object_with_children(+Identifier, +Time)
%
% Updates object in the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap, with their positions at time 'Time'
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
update_object_with_children(Identifier, Time) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'updateObjectWithChildren', [Identifier, Time], _).
    

%% remove_object(+Identifier) is det.
%
% Remove object from the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
remove_object(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeObject', [Identifier], _).


%% remove_object_with_children(+Identifier) is det.
%
% Removes objects from the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
remove_object_with_children(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeObjectWithChildren', [Identifier], _).

%% add_text(+Identifier, +Text, +Position) is nondet.
%
% Add view aligned text object to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Text The text that should be displayed
% @param Position The position of the text object.
%
add_text(Identifier, Text, Position) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Position, PositionArr),
    jpl_call(Canvas, 'addText', [Identifier, Text, PositionArr], _).

%% add_text(+Text) is nondet.
%
% Add HUD text ontop of canvas
%
% @param Text The text that should be displayed
%
add_text(Text) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addHUDText', [_Identifier, Text], _).
 
%% add_agent_speech_bubble(+Link, +Text, +TimePoint) is nondet.
%
% Add speech bubble sprite to the scene using the link as identifier
%
% @param Link A TF link that corresponds to the speech
% @param Text The text that should be displayed
% @param TimePoint The timepoint of the speech.
%
add_agent_speech_bubble(Link, Text, TimePoint) :-
    add_agent_speech_bubble(Link, Link, Text, TimePoint).

%% add_agent_speech_bubble(+Identifier, +Link, +Text, +TimePoint) is nondet.
%
% Add speech bubble sprite to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Link A TF link that corresponds to the speech
% @param Text The text that should be displayed
% @param TimePoint The timepoint of the speech.
%
add_agent_speech_bubble(Identifier, Link, Text, TimePoint) :-
    add_agent_speech_bubble(Identifier, Link, Text, TimePoint, -1).

%% add_agent_speech_bubble(+Identifier, +Link, +Text, +TimePoint, +Duration) is nondet.
%
% Add speech bubble sprite to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Link A TF link that corresponds to the speech
% @param Text The text that should be displayed
% @param TimePoint The timepoint of the speech.
% @param Duration The duration that defines how long the bubble should be displayed.
%
add_agent_speech_bubble(Identifier, Link, Text, TimePoint, Duration) :-
    ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
    % FIXME: don't assume /map as source frame
    mng_lookup_position(TfLink, '/map', TimePoint, Position),
    % TODO: probably need an offset here so that the bubble does not intersect with robot,
    % or disable depth test and render bubble after agent
    add_speech_bubble(Identifier, Text, Position, Duration).

%% add_speech_bubble(+Text, +Position) is nondet.
%
% Add speech bubble sprite to the scene using a default identifier
%
% @param Text The text that should be displayed
% @param Position The position of the text object.
%
add_speech_bubble(Text, Position) :-
    add_speech_bubble('SPEECH', Text, Position).

%% add_speech_bubble(+Identifier, +Text, +Position) is nondet.
%
% Add speech bubble sprite to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Text The text that should be displayed
% @param Position The position of the text object.
%
add_speech_bubble(Identifier, Text, Position) :-
    add_speech_bubble(Identifier, Text, Position, -1).

%% add_speech_bubble(+Identifier, +Text, +Position, +Duration) is nondet.
%
% Add speech bubble sprite to the scene
%
% @param Identifier Object identifier, eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Text The text that should be displayed
% @param Position The position of the text object.
% @param Duration The duration that defines how long the bubble should be displayed.
%
add_speech_bubble(Identifier, Text, Position, Duration) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Position, PositionArr),
    jpl_call(Canvas, 'addSpeechBubble', [Identifier, Text, PositionArr, Duration], _).

%% add_mesh(+MarkerId, +MeshPath) is nondet.
%
% Add mesh to visualization canvas
%
% @param MeshPath Mesh resource file
%
add_mesh(MarkerId, MeshPath) :-
    add_mesh(MarkerId, MeshPath, [0.0,0.0,0.0]).

%% add_mesh(+MarkerId, +MeshPath, +Position) is nondet.
%
% Add mesh to visualization canvas
%
% @param MeshPath Mesh resource file
% @param Position Position of mesh in scene
%
add_mesh(MarkerId, MeshPath, Position) :-
    add_mesh(MarkerId, MeshPath, Position, [0.0,0.0,0.0,1.0]).

%% add_mesh(+MarkerId, +MeshPath, +Translation, +Rotation) is nondet.
%
% Add mesh to visualization canvas
%
% @param MeshPath Mesh resource file
% @param Position Position of mesh in scene
% @param Rotation Rotation of mesh
%
add_mesh(MarkerId, MeshPath, Position, Rotation) :-
    add_mesh(MarkerId, MeshPath, Position, Rotation, [1.0,1.0,1.0]).

%% add_mesh(+MarkerId, +MeshPath, +Position, +Rotation, +Scale) is nondet.
%
% Add mesh to visualization canvas
%
% @param MeshPath Mesh resource file
% @param Position Position of mesh in scene
% @param Rotation Rotation of mesh
% @param Scale Scale of mesh
%
add_mesh(MarkerId, MeshPath, Position, Rotation, Scale) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Position, PositionArr),
    lists_to_arrays(Rotation, RotationArr),
    lists_to_arrays(Scale, ScaleArr),
    jpl_call(Canvas, 'addMeshMarker', [MarkerId, MeshPath, PositionArr, RotationArr, ScaleArr], _).
    
%%
%   Reads all trajectories described by start- and endtimes from logged tf data 
%   and visualizes the average of those trajectories in the Web-based canvas.
%   Note that start and endtimes should be lists of the same length
add_avg_trajectory(Link, Starttimes, Endtimes, IntervalParts, Markertype) :-
  visualisation_canvas(Canvas),
  
  ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
      (TfLink = Link)),!,

  jpl_list_to_array(Starttimes, ArrStart),
  jpl_list_to_array(Endtimes, ArrEnd),
  jpl_call(Canvas, 'showAverageTrajectory', [TfLink, ArrStart, ArrEnd, IntervalParts, Markertype], _).

%% add_trajectory(+Link, +Starttime, +Endtime) is det.
%% add_trajectory(+Link, +Starttime, +Endtime, +Interval) is det.
%% add_trajectory(+Link, +Starttime, +Endtime, +Interval, +Markertype) is det.
%
% Reads a trajectory from logged tf data and visualizes it in the
% Web-based canvas.
%
% @param Link     OWL individual or tf identifier of the link for which the trajectory is to be shown
% @param Starttime  Time stamp identifier for the beginning of the trajectory
% @param Endtime    Time stamp identifier for the end of the trajectory
% @param Interval   Sampling interval, describing the 
% @param Markertype The shape of the markers (see ROS Marker messages)
%
add_trajectory(Link, Starttime, Endtime) :-
  add_trajectory(Link, Starttime, Endtime, 1.0).

add_trajectory(Link, Starttime, Endtime, Interval) :-
  add_trajectory(Link, Starttime, Endtime, Interval, 0).

add_trajectory(Link, Starttime, Endtime, Interval, Markertype) :-
    visualisation_canvas(Canvas),

    ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
    
    jpl_call(Canvas, 'showTrajectory', [TfLink, Starttime, Endtime, Interval, Markertype], _).

%% remove_trajectory(+Link) is det.
%
% Removes all trajectories for the link 'TfLink' from the visualization. 
%
% @param Link     Tf identifier of the link for which the trajectory is to be removed
% 
remove_trajectory(Link) :-
    visualisation_canvas(Canvas),
    
    ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
     
    jpl_call(Canvas, 'removeTrajectory', [TfLink], _).

trajectory_length(Link, Starttime, Endtime, Length) :-
    visualisation_canvas(Canvas),
    
    ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
     
    jpl_call(Canvas, 'getTrajectoryLength', [TfLink, Starttime, Endtime, 5.0], Length).

%% add_pointer(+SLink, +ELink, +Timepoint) is det
%
% Read a pointer from logged tf data and visualizes it in the
% Web-based canvas
%
% @param SLink OWL individual or tf identifier of the link for which the start of pointer is to be shown
% @param ELink OWL individual or tf identifier of the link for which the end of pointer is to be shown
% @param Timepoint Timepoint identifier for the beginning of the pointer
%
add_pointer(SLink, ELink, Timepoint) :-
   visualisation_canvas(Canvas),  
  ((rdf_has(SLink, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
    atomic_list_concat(['/', Tf], TfSLink));
(TfSLink = SLink)),
    ((rdf_has(ELink, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfELink)) ;
     (TfELink = ELink)),!,
    jpl_call(Canvas, 'addPointer', [TfSLink, TfELink, Timepoint], _).

clear_trajectories :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'clearTrajectories', [], _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Highlighting objects
%

%% highlight_object(+Identifier) is det.
%% highlight_object(Identifier) is det.
%% highlight_object(Identifier, Color) is det.
%% highlight_object(Identifier, R, B, G, Alpha) is det.
%% highlight_trajectory(+Link, +Starttime, +Endtime, +Color) is det.
%% remove_highlight(+Identifier) is det.
%
% Different methods for highlighting objects. By default, objects are drawn in bright red
% if they are highlighted, but different colors can be specified using either one integer
% value (e.g. #00FF00) or separate values for red, green, and blue.
%
% If the object detection was uncertain, its probability can be visualized using the Prob
% parameter. This is done e.g. using the alpha channel or the hue value in HSV space
% (ignoring, in this case, the parameters R, B, G).
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Color      Color value as integer, e.g. #AARRBBGG
% @param R          Red color value
% @param B          Blue color value
% @param G          Green color value
% @param Prob       Object existence probability
%
highlight_object(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier], _).

highlight_object(Identifier, Color) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, Color], _).

highlight_object(Identifier, R, B, G, Alpha) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, R, B, G, Alpha], _).

highlight_trajectory(Link, Starttime, Endtime, Color) :-
    visualisation_canvas(Canvas),

    ((rdf_has(Link, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
  
    jpl_call(Canvas, 'getTrajectoryMarker', [TfLink, Starttime, Endtime], MarkersJ),
     
    jpl_array_to_list(MarkersJ, Markers),
    
    foreach(member(Marker, Markers), highlight_object(Marker, Color)).

    
%%%%Tmporray merge into rest...
highlight_object_mesh(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlightMesh', [Identifier], _).

highlight_object_mesh(Identifier, Color) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlightMesh', [Identifier, Color], _).
    
remove_mesh_highlight(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeMeshHighlight', [Identifier], _).
    
remove_highlight(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeHighlight', [Identifier], _).


%% remove_highlight_with_children(+Identifier) is det.
%% highlight_object_with_children(+Identifier) is det.
%% highlight_object_with_children(+Identifier, +Color) is det.
%
% Highlights an object and everything that is reachable from it via knowrob:properPhysicalPartTypes
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
%
% @param Identifier eg. "http://knowrob.org/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Highlight  @(true) = highlight; @(false)=remove highlighting
% @param Color      Color value as integer, e.g. #AARRBBGG
%

highlight_object_with_children(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier], _).

highlight_object_with_children(Identifier, Color) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, Color], _).

remove_highlight_with_children(Identifier) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeHighlightWithChildren', [Identifier], _).



%% reset_highlight is det.
%
% Reset all highlighted objects in the canvas.
%
reset_highlight :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'clearHighlight', [], _).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Diagram canvas management
%


%% diagram_canvas is det.
%
% Launch the diagram data publisher
%

:- assert(d_canvas(fail)).
%diagram_canvas :-
%    d_canvas(fail),
%    jpl_new('org.knowrob.vis.DiagramVisualization', [], Canvas),
%    retract(d_canvas(fail)),
%    assert(d_canvas(Canvas)),!.
%diagram_canvas(Canvas) :-
%    d_canvas(Canvas).

diagram_canvas :-
    d_canvas(fail),
    jpl_new('org.knowrob.vis.DiagramVisualization', [], Canvas),
    jpl_list_to_array(['org.knowrob.vis.DiagramVisualization'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Canvas, Arr], _),
    retract(d_canvas(fail)),
    assert(d_canvas(Canvas)),!.
diagram_canvas(Canvas) :-
    d_canvas(Canvas).

:- diagram_canvas.


%% add_diagram(+Id, +Title, +Type, +Xlabel, +Ylabel, +Width, +Height, +Fontsize, +ValueList) is nondet.
%
% Add object to the scene with its position at time 'Time'
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param Type       Type of the diagram (piechart, barchart or treechart)
% @param Xlabel     Label for the X axis
% @param Ylabel     Label for the Y axis
% @param Width      Width of the diagram in px
% @param Height     Height of the diagram in px
% @param Fontsize   Fontsize for the labels
% @param RowLabels  List of labels for the data rows (optional)
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, ValueList) :-
  add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, @(null), ValueList).
  
add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, RowLabels, ValueList) :-
  once((
    d_canvas(Canvas),
    lists_to_arrays(ValueList, ValueArr),
    once((lists_to_arrays(RowLabels, RowLabelArr) ; RowLabelArr = @(null))),
    jpl_call(Canvas, 'addDiagram', [Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, RowLabelArr, ValueArr], _)
  )).


%% help_timeline(+SVal, +EVal, -Res)
%
% Helpfunction for add_timeline that concatenates Start and End time to one atom so it can be passed to add_diagram in the same fashion as the other data types
help_timeline(SVal, EVal, Res) :-
  atom_concat(SVal, '_', Tmp),
  atom_concat(Tmp, EVal, Res).

%% add_timeline(+Id, +Title, +EventsList, +StartList, +EndList) is nondet.
%
% Simplified predicate for adding a timeline with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_timeline(Id, Title, EventsList, StartList, EndList) :-
  %Concatenate Start and endtimes so they can be given to add_diagram as one value
  maplist(help_timeline,StartList,EndList, TimeList),
  %Call add_diagram with the appropriate settings
  add_diagram(Id, Title, 'timeline', 'Time', 'Events', 250, 250, '12px', [[EventsList,TimeList]]).

%% add_piechart(+Id, +Title, +ValueList) is nondet.
%
% Simplified predicate for adding a piechart with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_piechart(Id, Title, ValueList) :-
    add_diagram(Id, Title, 'piechart', '', '', 250, 250, '9px', ValueList).


%% add_barchart(+Id, +Title, +ValueList) is nondet.
%
% Simplified predicate for adding a barchart with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_barchart(Id, Title, ValueList) :-
    add_diagram(Id, Title, 'barchart', '', '', 250, 250, '9px', ValueList).

    
%% remove_diagram(+Id) is det.
%
% Remove the diagram specified by 'id'
%
% @param Id Unique string identifier for the chart to be removed
% 
remove_diagram(Id) :-
    d_canvas(Canvas),
    jpl_call(Canvas, 'removeDiagram', [Id], _),!.

%% clear_diagram_canvas is det.
%
% Remove all diagrams from the canvas.
% 
clear_diagram_canvas :-
    d_canvas(Canvas),
    jpl_call(Canvas, 'clear', [], _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Task tree canvas management
%


%% task_tree_canvas is det.
%
% Launch the task tree data publisher
%

:- assert(t_t_canvas(fail)).
task_tree_canvas :-
    t_t_canvas(fail),
    jpl_new('org.knowrob.vis.TaskTreeVisualization', [], Canvas),
    jpl_list_to_array(['org.knowrob.vis.TaskTreeVisualization'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Canvas, Arr], _),
    retract(t_t_canvas(fail)),
    assert(t_t_canvas(Canvas)),!.
task_tree_canvas(Canvas) :-
    t_t_canvas(Canvas).

%% update_task_tree(+ListOfTasks, +ListOfHighlightedTask, +ListOfTypesShowed) is nondet.
%
% Simplified predicate for adding a barchart with default values
%
% @param ListOfTasks All of the tasks in a certain plan log 
% @param ListOfHighlightedTask  The tasks that should be highlighted in the task tree
% @param ListOfTypesShowed  Which task types should be appear in the visualized tree (i.e. in order to simplyfy the shown task tree)
%
update_task_tree(ListOfTasks, ListOfHighlightedTask, ListOfTypesShowed) :-
    t_t_canvas(Canvas),
    jpl_call(Canvas, 'addTaskTree', [ListOfTasks, ListOfHighlightedTask, ListOfTypesShowed], _R).

%% clear_diagram_canvas is det.
%
% Remove existing task tree from the canvas.
% 
remove_task_tree :-
    t_t_canvas(Canvas),
    jpl_call(Canvas, 'removeTaskTree', [], _R).
