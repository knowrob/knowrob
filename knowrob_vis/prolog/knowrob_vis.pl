/** <module> Methods for visualizing parts of the knowledge base

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

  @author Moritz Tenorth
  @license BSD
*/

:- module(knowrob_vis,
    [
      show/0,
      show/1,
      show/2,
      camera_pose/2,
      visualisation_server/0
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).

:- rdf_meta 
      show(t),
      show(t,r),
      camera_pose(r,r),
      camera_transform(r).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % Convinience predicate for different types of visualizations

show :- marker_update.

show(X) :-
  is_list(X), !,
  show_next,
  forall( member(MarkerDescr, X), (
    T =.. [show|MarkerDescr], call(T)
  )), !.

show(X) :-
  rdfs_individual_of(X, knowrob:'Designator'),
  designator_publish(X),
  (( rdf_has(Act, knowrob:objectActedOn, X),
     rdf_has(Act, knowrob:capturedImage, _) )
  -> designator_publish_image(Act)
  ;  true ), !.

show(X) :-
  get_timepoint(Instant),
  show(X,Instant,[]), !.

show(X, Properties) :-
  is_list(Properties),
  get_timepoint(Instant),
  show(X,Instant,Properties), !.

show(X, Instant) :-
  show(X, Instant, []), !.

show(X, Instant, Properties) :-
  is_list(Properties),
  
  marker_term(X, MarkerTerm),
  marker(MarkerTerm, MarkerObj),
  marker_update(MarkerObj,Instant),
  
  % TODO: X could also be a term agent(?Identifier) or object(?Identifier)
  (( atom(X), rdfs_individual_of(X, knowrob:'EmbodiedAgent') )
  -> ignore(show_speech(X,Instant)) ; true ),
  
  marker_properties(MarkerObj, Properties).
  

show_speech(Agent,Instant) :-
  rdf_has(Ev, knowrob:'sender', Agent),
  rdfs_individual_of(Ev, knowrob:'SpeechAct'),
  occurs(Ev, Instant),
  rdf_has(Ev, knowrob:'content', literal(type(_,Text))),
  rdf_has(Ev, knowrob:'sender', Agent),
  % find head
  sub_component(pr2:'PR2Robot1', Head),
  rdfs_individual_of(Head, knowrob:'Head-Vertebrate'),
  rdf_has(Head, srdl2comp:urdfName, URDFVal),
  strip_literal_type(URDFVal,URDF),
  % FIXME: /map bad assumption
  mng_lookup_transform('/map', URDF, Instant, Transform),
  matrix_translation(Transform, [X,Y,Z]),
  Z_Offset is Z + 0.2,
  marker(sprite_text('PR2_SPEECH'), MarkerObj),
  marker_color(sprite_text('PR2_SPEECH'), [1.0,1.0,1.0]),
  marker_translation(MarkerObj, [X,Y,Z_Offset]),
  % Create styled html text
  format(atom(TextHtml), '<div style="font-size: 18px; font-style: italic; font-family: Oswald,Arial,Helvetica,sans-serif; text-align: center;">~w</div>', [Text]),
  marker_text(MarkerObj, TextHtml),
  marker_scale(MarkerObj, [1.0,1.0,1.0]).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % Canvas camera manipulation

%% camera_pose(+Position:list, +Orientation:list) is det
%
% Sends a pose via the ROS topic "/camera/pose".
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

