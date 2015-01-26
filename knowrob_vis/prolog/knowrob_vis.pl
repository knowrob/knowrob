%   Description:
%     Module providing visualisation capabilities
% 
%   Copyright (C) 2013 by Moritz Tenorth
% 
%   This program is free software; you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation; either version 3 of the License, or
%   (at your option) any later version.
% 
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
% 
%   You should have received a copy of the GNU General Public License
%   along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% @author Moritz Tenorth
% @author Daniel Beßler
% @license GPL
% 
:- module(knowrob_vis,
    [
      visualisation_canvas/0,
      clear_canvas/0,
      camera_pose/2,
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
      highlight_object/1,
      highlight_object/2,
      highlight_object/5,
      highlight_object_with_children/1,
      highlight_object_with_children/2,
      highlight_trajectory/4,
      remove_highlight/1,
      remove_highlight_with_children/1,
      reset_highlight/0,
      add_avg_trajectory/5,
      add_trajectory/3,
      add_trajectory/4,
      add_trajectory/5,
      remove_trajectory/1,
      add_human_pose/2,
      add_human_pose/3,
      add_human_pose/4,
      remove_human_pose/0,
      remove_human_pose/1,
      diagram_canvas/0,
      clear_diagram_canvas/0,
      add_diagram/9,
      add_barchart/3,
      add_piechart/3,
      remove_diagram/1,
      trajectory_length/4
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta add_object(r),
            camera_pose(r,r),
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
            highlight_object(r),
            highlight_object(r,?),
            highlight_object(r,?,?,?,?,?),
            highlight_object_with_children(r),
            highlight_object_with_children(r,?),
            highlight_trajectory(r,r,r,?),
            remove_highlight(r),
            remove_highlight_with_children(r),
            add_diagram(+,+,+,+,+,+,+,+,+),
            remove_diagram(+),
            add_human_pose(r),
            add_human_pose(r,r),
            add_human_pose(r,r,r),
            add_human_pose(r,r,r,r),
            remove_human_pose(r),
            remove_human_pose(r,r),
            add_avg_trajectory(r,r,r,r,r),
            add_trajectory(r,r,r),
            add_trajectory(r,r,r,+),
            add_trajectory(r,r,r,+,+),
            remove_trajectory(r),
            trajectory_length(r,r,r,+).


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
    jpl_new('org.knowrob.vis.MarkerVisualization', [], Canvas),
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
% Camera
%

camera_pose(Position, Orientation) :-
    visualisation_canvas(Canvas),
    lists_to_arrays(Position, PositionArr),
    lists_to_arrays(Orientation, OrientationArr),
    jpl_call(Canvas, 'setCameraPose', [PositionArr, OrientationArr], _).

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
    jpl_call(Canvas, 'addHUDText', [Identifier, Text], _).
    
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

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Human pose
%

%% add_human_pose(+Human, +Timepoint) is det.
%% add_human_pose(+Human, +Id, +Timepoint) is det.
%% add_human_pose(+Human, +Id, +Timepoint, +Prefix) is det.
%
% Reads joint poses of a human from logged tf data and visualizes them in the
% Web-based canvas using a stick-man model.
%
% @param Human  The human individual that defines the skeletal structure
% @param Timepoint  Time stamp identifier of the pose
%
add_human_pose(Human, Timepoint) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addHumanPose', [Human,Timepoint,0,''], _).
add_human_pose(Human, Id, Timepoint) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addHumanPose', [Human,Timepoint,Id,''], _).
add_human_pose(Human, Id, Timepoint, Prefix) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'addHumanPose', [Human,Timepoint,Id,Prefix], _).

%% remove_human_pose is det.
%% remove_human_pose(+Id) is det.
%
% Removes human pose visualizations from the visualization canvas.
%
remove_human_pose :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeHumanPose', [0], _).
remove_human_pose(Id) :-
    visualisation_canvas(Canvas),
    jpl_call(Canvas, 'removeHumanPose', [Id], _).

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
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_diagram(Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, ValueList) :-
    d_canvas(Canvas),
    lists_to_arrays(ValueList, ValueArr),
    jpl_call(Canvas, 'addDiagram', [Id, Title, Type, Xlabel, Ylabel, Width, Height, Fontsize, ValueArr], _),!.

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

