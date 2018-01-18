/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2015 Daniel Beßler
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

:- module(knowrob_vis,
    [
      show/0,
      show/1,
      show/2,
      show/3,
      show_next/0,
      highlight/1,
      highlight/2,
      camera_pose/2,
      visualisation_server/0
    ]).
/** <module> Methods for visualizing parts of the knowledge base

  @author Moritz Tenorth
  @author Daniel Beßler
  @license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('jpl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/marker_vis')).
:- use_module(library('knowrob/data_vis')).

:- rdf_meta 
      show(t),
      show(t,r),
      show(t,r,t),
      camera_pose(r,r),
      camera_transform(r).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % Convinience predicate for different types of visualizations

%% show is det.
%% show(+VisualThing) is det.
%% show(+VisualThing, +Instant) is det.
%% show(+VisualThing, +Instant, +Properties) is det.
%
% VisualThing is a thing with a visual interpretation
% and some way to generate a ROS visualization message for it.
% This includes marker_visualization messages and data_vis messages.
% VisualThing may be a RDF IRI of some OWL individual,
% a marker term (e.g., object(Iri)), or a data vis object (e.g., timeline(Identifier)).
% All existing markers are updated for the current timepoint if
% VisualThing is left unspecified.
% Properties is a list of properties passed to
% the respective submodules (i.e., marker or data visualization).
% If VisualThing is a list then each element is expected to be a term
% describing one visualization artifact.
%
show :- marker_update.

show(VisualThing) :-
  is_list(VisualThing), !,
  show_next,
  forall( member(MarkerDescr, VisualThing), (
    T =.. [show|MarkerDescr], call(T)
  )), !.

show(VisualThing) :-
  current_time(Instant),
  show(VisualThing,Instant,[]), !.

show(VisualThing, Properties) :-
  is_list(Properties),
  current_time(Instant),
  show(VisualThing,Instant,Properties), !.

show(VisualThing, Instant) :-
  show(VisualThing, Instant, []), !.

show(VisualThing, Instant, Properties) :-
  marker_term(VisualThing, MarkerTerm), !,
  marker(MarkerTerm, MarkerObj),
  marker_update(MarkerObj,Instant),
  marker_properties(MarkerObj, Properties).

show(DataVisTerm, _, Properties) :-
  data_vis(DataVisTerm, Properties), !.

%% show_next is det
%
% Unhighlights objects and removes displayed trajectories.
show_next :-
  marker_highlight_remove(all),
  marker_remove(trajectories).

%% highlight(+VisualThing) is det.
%% highlight(+VisualThing,+Color) is det.
%
% Visually highlights VisualThing in the respective canvas.
highlight(VisualThing) :-
  marker_term(VisualThing, MarkerTerm),
  marker_highlight(MarkerTerm).
highlight(VisualThing,Color) :-
  marker_term(VisualThing, MarkerTerm),
  marker_highlight(MarkerTerm,Color).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % Canvas camera manipulation

%% camera_pose(+Position:list, +Orientation:list) is det
%
% Sends a pose via the ROS topic _|/camera/pose|_.
% Visualization clients may choose to manipulate some 3D camera accordingly.
%
% @param Position [float x,y,z]
% @param Orientation [float qx,qy,qz,qw]
%
camera_pose(Position, Orientation) :-
    camera_interface(Camera),
    lists_to_arrays(Position, PositionArr),
    lists_to_arrays(Orientation, OrientationArr),
    jpl_call(Camera, 'setCameraPose', [PositionArr, OrientationArr], _).

camera_interface(Camera) :-
    (\+ current_predicate(v_camera_interface, _)),
    jpl_new('org.knowrob.vis.Camera', [], Camera),
    jpl_list_to_array(['org.knowrob.vis.Camera'], Arr),
    jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [Camera, Arr], _),
    assert(v_camera_interface(Camera)),!.
camera_interface(Camera) :-
    current_predicate(v_camera_interface, _),
    v_camera_interface(Camera).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % Visualization server management

%% visualisation_server is det.
%
% Launches a web server that runs a minimal visualization client.
% This implementation is able to handle most marker visualization messages,
% data visualization messages (among other features) are unsupported.
% A proper client is implemented in [[openEASE][http://www.open-ease.org/]].
% You can access the minimal client via http://localhost:1111
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

