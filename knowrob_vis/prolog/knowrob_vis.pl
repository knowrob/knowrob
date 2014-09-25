/** <module> knowrob_vis

  Description:
    Module providing visualisation capabilities

  Copyright (C) 2013 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@author Daniel Beßler
@license GPL
*/

:- module(knowrob_vis,
    [
      visualisation_canvas/0,
      clear_canvas/0,
      add_object/1,
      add_object/2,
      add_object_with_children/1,
      add_object_with_children/2,
      remove_object/1,
      remove_object_with_children/1,
      highlight_object/1,
      highlight_object/2,
      highlight_object/3,
      highlight_object/6,
      highlight_object_with_children/1,
      highlight_object_with_children/2,
      reset_highlight/0,
      diagram_canvas/0,
      add_diagram/9,
      add_barchart/3,
      add_piechart/3,
      remove_diagram/1,
      clear_diagram_canvas/0,
      add_trajectory/3,
      add_trajectory/4,
      remove_trajectory/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta add_object(r),
            add_object(r,r),
            add_object_with_children(r),
            add_object_with_children(r,r),
            remove_object(r),
            remove_object_with_children(r),
            highlight_object(r),
            highlight_object(r,?),
            highlight_object(r,?,?),
            highlight_object(r,?,?,?,?,?),
            highlight_object_with_children(r),
            highlight_object_with_children(r,?),
            add_diagram(+,+,+,+,+,+,+,+,+),
            remove_diagram(+),
            add_trajectory(r,r,r),
            add_trajectory(r,r,r,+).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Visualization canvas management
%

%% visualisation_canvas is det.
%
% Launch the visualization canvas
%
:- assert(v_canvas(fail)).
visualisation_canvas :-
    v_canvas(fail),
    jpl_new('org.knowrob.vis.MarkerVisualization', [], Canvas),
    jpl_list_to_array(['org.knowrob.vis.MarkerVisualization'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Canvas, Arr], _),
    retract(v_canvas(fail)),
    assert(v_canvas(Canvas)),!.
visualisation_canvas(Canvas) :-
    v_canvas(Canvas).


%% clear_canvas is det.
%
% Completely clears the scene
%
clear_canvas :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'clear', [], _).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Add / remove objects and trajectories
%

%% add_object(+Identifier) is nondet.
%
% Add object to the scene
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object(Identifier) :-
    get_timepoint(Time),
    add_object(Identifier, Time).


%% add_object(+Identifier, +Time) is nondet.
%
% Add object to the scene with its position at time 'Time'
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object(Identifier, Time) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'addObject', [Identifier, Time], _).



%% add_object_with_children(+Identifier)
%
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object_with_children(Identifier) :-
    get_timepoint(Time),
    add_object_with_children(Identifier, Time).


%% add_object_with_children(+Identifier, +Time)
%
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap, with their positions at time 'Time'
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
add_object_with_children(Identifier, Time) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'addObjectWithChildren', [Identifier, Time], _).


%% remove_object(+Identifier) is det.
%
% Remove object from the scene
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
remove_object(Identifier) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'removeObject', [Identifier], _).


%% remove_object_with_children(+Identifier) is det.
%
% Removes objects from the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
remove_object_with_children(Identifier) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'removeObjectWithChildren', [Identifier], _).


%% add_trajectory(+Link, +Starttime, +Endtime) is det.
%% add_trajectory(+Link, +Starttime, +Endtime, +Interval) is det.
%
% Reads a trajectory from logged tf data and visualizes it in the
% Web-based canvas.
%
% @param Link     OWL individual or tf identifier of the link for which the trajectory is to be shown
% @param Starttime  Time stamp identifier for the beginning of the trajectory
% @param Endtime    Time stamp identifier for the end of the trajectory
% @param Interval   Sampling interval, describing the 
%
add_trajectory(Link, Starttime, Endtime) :-
  add_trajectory(Link, Starttime, Endtime, 1.0).

add_trajectory(Link, Starttime, Endtime, Interval) :-
    v_canvas(Canvas),

    ((rdf_has(Link, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
    
    jpl_call(Canvas, 'showTrajectory', [TfLink, Starttime, Endtime, Interval], _).


%% remove_trajectory(+Link) is det.
%
% Removes all trajectories for the link 'TfLink' from the visualization. 
%
% @param Link     Tf identifier of the link for which the trajectory is to be removed
% 
remove_trajectory(Link) :-
    v_canvas(Canvas),
    
    ((rdf_has(Link, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#urdfName', literal(Tf)),
      atomic_list_concat(['/', Tf], TfLink)) ;
     (TfLink = Link)),!,
     
    jpl_call(Canvas, 'removeTrajectory', [TfLink], _).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Human pose
%

%% add_human_pose(+Timepoint) is det.
%
% Reads joint poses of a human from logged tf data and visualizes them in the
% Web-based canvas using a stick-man model.
%
% @param Timepoint  Time stamp identifier of the pose
%
add_human_pose(Timepoint) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'addHumanPose', [Timepoint], _).

%% remove_human_pose() is det.
%
% Removes all human pose visualizations from the visualization canvas.
%
%remove_human_pose() :-
%    v_canvas(Canvas),
%    jpl_call(Canvas, 'removeHumanPose', [], _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Highlighting objects
%

%% highlight_object(+Identifier) is det.
%% highlight_object(Identifier, Highlight) is det.
%% highlight_object(Identifier, Highlight, Color) is det.
%% highlight_object(Identifier, Highlight, R, B, G, Alpha) is det.
%
% Different methods for highlighting objects. By default, objects are drawn in bright red
% if they are highlighted, but different colors can be specified using either one integer
% value (e.g. #00FF00) or separate values for red, green, and blue.
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
%
% If the object detection was uncertain, its probability can be visualized using the Prob
% parameter. This is done e.g. using the alpha channel or the hue value in HSV space
% (ignoring, in this case, the parameters R, B, G).
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Highlight  @(true) = highlight; @(false)=remove highlighting
% @param Color      Color value as integer, e.g. #AARRBBGG
% @param R          Red color value
% @param B          Blue color value
% @param G          Green color value
% @param Prob       Object existence probability
%
highlight_object(Identifier) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, @(true)], _).

highlight_object(Identifier, Highlight) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight], _).

highlight_object(Identifier, Highlight, Color) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, Color], _).

highlight_object(Identifier, Highlight, R, B, G, Prob) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, R, B, G, Prob], _).



%% highlight_object_with_children(+Identifier) is det.
%% highlight_object_with_children(+Identifier, +Highlight) is det.
%
% Highlights an object and everything that is reachable from it via knowrob:properPhysicalPartTypes
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
%
highlight_object_with_children(Identifier) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, @(true)], _).

highlight_object_with_children(Identifier, Highlight) :-
    v_canvas(Canvas),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, Highlight], _).



%% reset_highlighting is det.
%
% Reset all highlighted objects in the canvas.
%
reset_highlight :-
    v_canvas(Canvas),
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
diagram_canvas :-
    d_canvas(fail),
    jpl_new('org.knowrob.vis.DiagramVisualization', [], Canvas),
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
    add_diagram(Id, Title, 'piechart', '', '', 300, 300, '12px', ValueList).


%% add_barchart(+Id, +Title, +ValueList) is nondet.
%
% Simplified predicate for adding a barchart with default values
%
% @param Identifier Unique string identifier for this diagram
% @param Title      Title of this chart
% @param ValueList  List of data ranges, each of the form [[a,b],['1','2']]
%
add_barchart(Id, Title, ValueList) :-
    add_diagram(Id, Title, 'barchart', '', '', 300, 300, '12px', ValueList).

    
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

