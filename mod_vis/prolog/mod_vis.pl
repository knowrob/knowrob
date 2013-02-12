/** <module> mod_vis

  Description:
    Module providing visualisation capabilities


  Copyright (C) 2010 by Moritz Tenorth

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
@license GPL
*/

:- module(mod_vis,
    [
      visualisation_canvas/1,
      clear_canvas/1,
      draw_background/1,
      set_view_parameters/6,
      display_information_for/2,
      display_action/2,
      display_action_fixed/2,
      display_eye_trajectory/2,
      display_human_trajectory/3,
      show_actionseq/4,
      add_object/2,
      add_object_perception/2,
      add_object_with_children/2,
      remove_object/2,
      remove_object_with_children/2,
      highlight_object/2,
      highlight_object/3,
      highlight_object/4,
      highlight_object/6,
      highlight_object/7,
      add_and_highlight_object/2,
      highlight_object_with_children/2,
      highlight_object_with_children/3,
      reset_highlighting/1,
      show_images/2,
      planvis_create/1,
      planvis_create_fsm/1,
      planvis_load/2,
      planvis_highlight/2,
      planvis_highlight/3,
      planvis_clear_highlight/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).


:- rdf_meta planvis_load(r,?),
            planvis_highlight(r,?),
            planvis_highlight(r,?,?),
            display_action_fixed(r,?),
            display_action(r,?),
            display_eye_trajectory(r,?),
            display_human_trajectory(r,r,?),
            add_object(r,?),
            add_object_with_children(r,?),
            add_object_perception(r,?),
            add_trajectory(r,?),
            remove_object(r,?),
            remove_object_with_children(r,?),
            highlight_object(r,?),
            highlight_object(r,?,?),
            highlight_object(r,?,?,?),
            highlight_object(r,?,?,?,?,?),
            highlight_object_with_children(r,?),
            highlight_object_with_children(r,?,?),
            add_and_highlight_object(r,?),
            planvis_load(r,?),
            planvis_highlight(r,?),
            planvis_highlight(r,?,?).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Visualization canvas management
%

%% visualisation_canvas(-Canvas) is det.
%
% Launch the visualization canvas
%
:- assert(v_canvas(fail)).
visualisation_canvas(Canvas) :-
    v_canvas(fail),
    jpl_new('javax.swing.JFrame', [], Frame),
    jpl_call(Frame, 'resize', [800, 630], _),
    jpl_new('edu.tum.cs.ias.knowrob.vis.applets.PrologVisualizationCanvas', [], Canvas),
    jpl_call(Canvas, 'init', [], _),
    jpl_call(Canvas, 'drawBackground', [], _),
    jpl_call(Frame, 'add', [Canvas], _),
    jpl_call(Frame, 'setVisible', [@(true)], _),
    retract(v_canvas(fail)),
    assert(v_canvas(Canvas)),!.
visualisation_canvas(Canvas) :-
    v_canvas(Canvas).


%% clear_canvas(+Canvas) is det.
%
% Completely clears the scene
%
% @param Canvas Visualization canvas
%
clear_canvas(Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'clear', [], _).


%% draw_background(+Identifier, +Canvas) is det.
%
% Reset the scene and draw the background (i.e. usually the kitchen environment)
%
% @param Canvas     Visualization canvas
%
draw_background(Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'drawBackground', [], _).


%% set_view_parameters(+XShift, +YShift, +XRot, +YRot, +Zoom, +Canvas) is det.
%
% Change the display parameters of the visualization applet.
%
% @param XShift   Shift in x-direction
% @param YShift   Shift in y-direction
% @param XRot     Rotation in x-direction
% @param YRot     Rotation in x-direction
% @param Zoom     Zoom factor
% @param Canvas   Visualization canvas
%
set_view_parameters(XShift, YShift, XRot, YRot, Zoom, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'setViewParameters', [XShift, YShift, XRot, YRot, Zoom], _).


%% display_information_for(+Identifier, +Canvas) is det.
%
% Show any information that can be retrieved with rdf_has(Identifier, P, O) in the
% external control window of the visualization canvas.
%
% @param Identifier Identifier of any kind of instance, e.g. an object or an action
% @param Canvas     Visualization canvas
%
display_information_for(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayInfoFor', [Identifier], _).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Action display predicates
%

%% display_action(+Identifier, +Canvas) is det.
%
% Displays an action in the Kitchen environment via its dynamic identifier (created e.g. by CRF segmentation)
% for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'"
%
% @param Identifier Action instance, e.g. 'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching50'
% @param Canvas     Visualization canvas
%
display_action(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayAction', [Identifier], _).


%% display_action_fixed(+Identifier, +Canvas) is det.
%
% Displays an action in the Kitchen environment via its fixed identifier (including EPISODE_NR and INSTANCE_NR, usually read from the DB)
% for example "'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'"
%
% @param Identifier Action instance, e.g. 'http://ias.cs.tum.edu/kb/knowrob.owl#Reaching_0_2'
% @param Canvas     Visualization canvas
%
display_action_fixed(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayFixed', [Identifier], _).

%% display_eye_trajectory(+Identifier, +Canvas) is det.
%
% Displays an eye trajectory in the Kitchen environment via its fixed identifier
% for example "'http://ias.cs.tum.edu/kb/knowrob.owl#PickingUpAnObject'"
%
% @param Identifier, e.g. 'http://ias.cs.tum.edu/kb/knowrob.owl#PickingUpAnObject'
% @param Canvas     Visualization canvas
%
display_eye_trajectory(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'displayEyeTrajectory', [Identifier], _).



display_human_trajectory(Identifier, HandUsed, Canvas):-
   ((var(Canvas)) -> (v_canvas(Canvas));(true)),
   jpl_call(Canvas, 'displayHumanTrajectory', [Identifier, HandUsed], _).



%% show_actionseq(+SeqInfos, +Canvas, +Hand, +Level) is det.
%
% Visualize action sequences in the right part of the visualization canvas
%
% @param SeqInfos   Action information of the form [ [ [label],[startTime],[endTime],[[actionType1],...],[objtype]], [...] ], converted to Java array
% @param Hand       Hand, one of 'left' or 'right', corresponding to two stacks of action sequences
% @param Level      Abstraction level (displayed as vertical axis)
% @param Canvas     Visualization canvas
%
show_actionseq(SeqInfos, Canvas, Hand, Level) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'setActionInformation', [SeqInfos, Hand, Level], _).


%
% Adding/removing objects
%

%% add_object(+Identifier, +Canvas) is nondet.
%
% Add object to the scene
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
add_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObject', [Identifier], _).


%% add_object_with_children(+Identifier, +Canvas)
%
% Adds objects to the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
add_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObjectWithChildren', [Identifier], _).

%% add_trajectory(+Identifiers, +Canvas) is nondet.
%
% Add a trajectory as a list of point identifiers to the scene
%
% @param Identifiers List of point identifiers, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#point3d1"
% @param Canvas      Visualization canvas
%
add_trajectory(Identifiers, Canvas) :-
    jpl_list_to_array(Identifiers, TrajArray),
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addTrajectory', [TrajArray], _).




%% remove_object(+Identifier, +Canvas) is det.
%
% Remove object from the scene
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
remove_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'removeObject', [Identifier], _).


%% remove_object_with_children(+Identifier, +Canvas) is det.
%
% Removes objects from the scene, including all items that are reachable via knowrob:properPhysicalPartTypes
% or via knowrob:describedInMap
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
remove_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'removeObjectWithChildren', [Identifier], _).


%% add_object_perception(+Identifier, +Canvas) is nondet.
%
% Add an object to the scene and color it based on the kind of process that generated it:
% * Perceived objects are light grey
% * Inferred objects are colored based on their probability
%
% @param Identifier Object identifier, eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
add_object_perception(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),

    % add the object based on its latest detection
    add_object(Identifier, Canvas),

    % find all perceptions of the object and sort by their start time
    findall([D_i,Identifier,St], (rdf_has(D_i, knowrob:objectActedOn, Identifier),
                                  rdfs_individual_of(D_i,  knowrob:'MentalEvent'),
                                  rdf_has(D_i, knowrob:startTime, StTg),
                                  rdf_split_url(_, StTl, StTg),
                                  atom_concat('timepoint_', StTa, StTl),
                                  term_to_atom(St, StTa)), Detections),

    predsort(compare_object_detections, Detections, Dsorted),

    % compute the homography for the newest perception
    nth0(0, Dsorted, Newest),
    nth0(0, Newest, NewestDetection),

    % highlight based on the source of information
    ((
        % do not highlight pieces of furniture
        rdfs_individual_of(Identifier, knowrob:'FurniturePiece'),!
    ) ; (
        % display perceived objects in light grey
        rdfs_individual_of(NewestDetection, knowrob:'Perceiving'),
        highlight_object(Identifier, @(true), 160, 160, 160, Canvas),!
    ) ; (
        % display inferred objects by their probability
        rdfs_individual_of(NewestDetection, knowrob:'ProbCogReasoning'),
        rdf_has(NewestDetection, knowrob:probability, Prob),
        highlight_object(Identifier, @(true), 230, 230, 230, Prob, Canvas),!
    ) ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Highlighting objects
%

%% highlight_object(+Identifier, +Canvas) is det.
%% highlight_object(Identifier, Highlight, Canvas) is det.
%% highlight_object(Identifier, Highlight, Color, Canvas) is det.
%% highlight_object(Identifier, Highlight, R, B, G, Canvas) is det.
%% highlight_object(Identifier, Highlight, R, B, G, Prob, Canvas) is det.
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
% @param Canvas     Visualization canvas
%
highlight_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, @(true)], _).

highlight_object(Identifier, Highlight, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight], _).

highlight_object(Identifier, Highlight, Color, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, Color], _).

highlight_object(Identifier, Highlight, R, B, G, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, R, B, G], _).

highlight_object(Identifier, Highlight, R, B, G, Prob, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlight', [Identifier, Highlight, R, B, G, Prob], _).



%% add_and_highlight_object(+Identifier, +Canvas) is det.
%
% Shortcut to add an object and highlight it at the same time.
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
add_and_highlight_object(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'addObject', [Identifier], _),
    jpl_call(Canvas, 'highlight', [Identifier, @(true)], _).



%% highlight_object_with_children(+Identifier, +Canvas) is det.
%% highlight_object_with_children(+Identifier, +Highlight, +Canvas) is det.
%
% Highlights an object and everything that is reachable from it via knowrob:properPhysicalPartTypes
%
% The parameter Highlight specifies if the highlighting shall be activated or reset; if
% it is missing, a value of @(true) is assumed.
%
% @param Identifier eg. "http://ias.cs.tum.edu/kb/ias_semantic_map.owl#F360-Containers-revised-walls"
% @param Canvas     Visualization canvas
%
highlight_object_with_children(Identifier, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, @(true)], _).

highlight_object_with_children(Identifier, Highlight, Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'highlightWithChildren', [Identifier, Highlight], _).



%% reset_highlighting(+Canvas) is det.
%
% Reset all highlighted objects in the canvas.
%
% @param Canvas     Visualization canvas
%
reset_highlighting(Canvas) :-
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'clearHighlight', [], _).

%% show_images(+ImagePaths, +Canvas) is det.
%
% Visualize the images specified by ImagePaths in a new JFrame.
%
% @param ImagePaths  List of: global path+filename for each image
% @param Canvas     Visualization canvas
%
show_images(ImagePaths, Canvas) :-
    jpl_list_to_array(ImagePaths, ImagePathsArray),
    ((var(Canvas)) -> (v_canvas(Canvas));(true)),
    jpl_call(Canvas, 'showImagesInNewWindow', [ImagePathsArray], _).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% PlanVis management
%

%% planvis_create(-PlanVis) is det.
%
% Launch the planvis applet
%
:- assert(planvis(fail)).
planvis_create(PlanVis) :-
    planvis(fail),
    jpl_new('javax.swing.JFrame', [], Frame),
    jpl_call(Frame, 'resize', [1600, 400], _),
    jpl_new('edu.tum.cs.ias.knowrob.vis.applets.PlanVisApplet', [], PlanVis),
    jpl_call(PlanVis, 'init', [], _),
    jpl_call(Frame, 'add', [PlanVis], _),
    jpl_call(Frame, 'setVisible', [@(true)], _),
    retract(planvis(fail)),
    assert(planvis(PlanVis)),!.
planvis_create(PlanVis) :-
    planvis(PlanVis).


%% planvis_create_fsm(-PlanVis) is det.
%
% Launch the planvis applet for state-machine recipes
%

planvis_create_fsm(PlanVis) :-
    planvis(fail),
    jpl_new('javax.swing.JFrame', [], Frame),
    jpl_call(Frame, 'resize', [1600, 400], _),
    jpl_new('edu.tum.cs.ias.knowrob.vis.applets.PlanVisAppletFsm', [], PlanVis),
    jpl_call(PlanVis, 'init', [], _),
    jpl_call(Frame, 'add', [PlanVis], _),
    jpl_call(Frame, 'setVisible', [@(true)], _),
    retract(planvis(fail)),
    assert(planvis(PlanVis)),!.
planvis_create_fsm(PlanVis) :-
    planvis(PlanVis).


%% planvis_load(+Identifier,+Planvis) is det.
%
% Load the action/plan with given by Identifier and all its children recursively
% to draw it on the PlanVis applet
%
% @param Identifier  eg. 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
% @param PlanVis     PlanVis applet
%
planvis_load(Identifier,Planvis) :-
    ((var(Planvis)) -> (planvis(Planvis));(true)),
    jpl_call(Planvis, 'loadPrologPlan', [Identifier], _).

%% planvis_highlight(+Identifier,+Planvis) is det.
%% planvis_highlight(Identifier,Expand, Planvis) is det.
%
% Hightlights the action identified by identifier.
%
% @param Identifier  eg. 'http://www.roboearth.org/kb/serve_drink.owl#ServeADrink'
% @param PlanVis     PlanVis applet
% @param Expand	     Expand all sequences to show hightlighted action. Default: true
%
planvis_highlight(Identifier,Planvis) :-
    ((var(Planvis)) -> (planvis(Planvis));(true)),
    jpl_call(Planvis, 'highlightAction', [Identifier, @(true)], _).
planvis_highlight(Identifier,Expand,Planvis) :-
    ((var(Planvis)) -> (planvis(Planvis));(true)),
    jpl_call(Planvis, 'highlightAction', [Identifier, Expand], _).

%% planvis_clear_highlight(+Planvis) is det.
%
% Removes the hightlight from current highlighted action
%
% @param PlanVis     PlanVis applet
%
planvis_clear_highlight(Planvis) :-
    ((var(Planvis)) -> (planvis(Planvis));(true)),
    jpl_call(Planvis, 'clearHighlight', [], _).
