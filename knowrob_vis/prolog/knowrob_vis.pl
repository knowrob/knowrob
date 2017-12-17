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
      visualisation_server/0,
      camera_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).

:- rdf_meta camera_pose(r,r),
            camera_transform(r).

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

