/*
  Copyright (C) 2017 Daniel Beßler
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

@author Daniel Beßler
@license BSD
*/

:- module('knowrob/comp/tf',
    [
      map_frame_name/1,             % +Name
      transform_reference_frame/2,  % +Transform, ?ReferenceFrame
      transform_data/2,             % +Transform, ?(Translation, Rotation)
      transform_multiply/3,         % +Transform1, +Transform2, -Product
      transform_between/3,          % +Transform1, +Transform2, -Relative
      transform_interpolate/4,      % +Transform1, +Transform2, +Factor, -Interpolated
      transform_close_to/3,         % +Transform1, +Transform2, +Delta
      transform_invert/2,           % +Transform, -Inverted
      matrix/3,                     % ?Matrix, ?Translation, ?Quaternion
      matrix_translate/3,           % +In, +Delta, -Out
      quaternion_multiply/3,        % +Quaternion1, +Quaternion2, -Multiplied
      quaternion_inverse/2,         % +Quaternion, -Inverse
      quaternion_transform/3        % +Quaternion, +Vector, -Transformed
    ]).
/** <module> Utilities for handling transforms in KnowRob.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob/lang/ask'), [
    kb_triple/3
]).

:-  rdf_meta
    transform_reference_frame(r,?),
    transform_data(r,?).

:- use_foreign_library('libkb_algebra.so').

%% map_frame_name(?Name:atom) is det.
%
% True is Name is the name of the global coordinate system.
%
% FIXME: this needs different handling
map_frame_name(FrameName) :-
  rdf_has(knowrob:'MapFrame', knowrob:'frameName', literal(type(_,FrameName))), !.
%map_frame_name(FrameName) :-
  %ros_param_get_string('knowrob/map_frame', FrameName)
map_frame_name('map').

%% transform_multiply(+Transform1:term, +Transform2:term, ?Product:term) is semidet.
%
% True if Product is Transform1 x Transform2.
% This is only defined if the target frame of Transform1 is equal
% to the reference frame of Transform2.
%
% @param Transform1 A Prolog term [Ref,A,Pos1,Rot1]
% @param Transform2 A Prolog term [A,Src,Pos2,Rot2]
% @param Product A Prolog term [Ref,Src,Pos3,Rot3]
%
transform_multiply([RefFrame,       F, [Lx,Ly,Lz], [LQx, LQy, LQz, LQw]],
                   [       F, TgFrame, [Rx,Ry,Rz], [RQx, RQy, RQz, RQw]],
                   [RefFrame, TgFrame, [Nx,Ny,Nz], [NQx, NQy, NQz, NQw]]) :-
  NQw is LQw*RQw - LQx*RQx - LQy*RQy - LQz*RQz,
  NQx is LQw*RQx + LQx*RQw + LQy*RQz - LQz*RQy,
  NQy is LQw*RQy - LQx*RQz + LQy*RQw + LQz*RQx,
  NQz is LQw*RQz + LQx*RQy - LQy*RQx + LQz*RQw,
  RRx is 2*(Rx*(0.5 - LQy*LQy - LQz*LQz) + Ry*(LQx*LQy - LQw*LQz) + Rz*(LQw*LQy + LQx*LQz)),
  RRy is 2*(Rx*(LQw*LQz + LQx*LQy) + Ry*(0.5 - LQx*LQx - LQz*LQz) + Rz*(LQy*LQz - LQw*LQx)),
  RRz is 2*(Rx*(LQx*LQz - LQw*LQy) + Ry*(LQw*LQx + LQy*LQz) + Rz*(0.5 - LQx*LQx - LQy*LQy)),
  Nx is Lx + RRx,
  Ny is Ly + RRy,
  Nz is Lz + RRz.

%% transform_between(+Transform1:term, +Transform2:term, ?Relative:term) is semidet.
%
% True if Relative is the relative transform between the target frame of
% Transform1 and Transform2.
% Only defined if Transform1 and Transform2 share the same reference frame.
%
% @param Transform1 A Prolog term [F,Src,Pos1,Rot1]
% @param Transform2 A Prolog term [F,Ref,Pos2,Rot2]
% @param Transform1 A Prolog term [Ref,Src,Pos3,Rot3]
%
transform_between([F,TgFrame, [T1x,T1y,T1z],Q1],
                  [F,RefFrame,[T2x,T2y,T2z],Q2],
                  [RefFrame,TgFrame,TN,QN]) :-
  quaternion_inverse(Q2, Q2_inv),
  quaternion_multiply(Q1, Q2_inv, QN),
  Diff_x is T2x - T1x,
  Diff_y is T2y - T1y,
  Diff_z is T2z - T1z,
  quaternion_transform(Q1, [Diff_x,Diff_y,Diff_z], TN).

%%
transform_interpolate(
    [Ref,Tg,T1,Q1],
    [Ref,Tg,T2,Q2],
    Factor,
    [Ref,Tg,T,Q]) :-
  % linear interpolation
  lerp_list(T1,T2,Factor,T),
  % spherical linear interpolation
  quaternion_slerp(Q1,Q2,Factor,Q).

%%
lerp(X0,X1,Factor,X) :-
  X is Factor*X0 + (1.0 - Factor)*X1.

lerp_list([],[],_,[]) :- !.
lerp_list([X0|Xs0],[X1|Xs1],Factor,[X|Xs]) :-
  lerp(X0,X1,Factor,X),
  lerp_list(Xs0,Xs1,Factor,Xs).


%% transform_data(+Transform:term, ?Data:tuple) is semidet.
%
% True if Pos, and Rot are the translation and orientation of
% Transform and Data=(Pos,Rot).
%
% @param Transform A transform RDF id or Prolog term [Ref,Src,Pos,Rot]
% @param Data The tuple (Pos,Rot)
%
transform_data([_,_,Pos,Rot], (Pos,Rot)) :- !.

transform_data(Transform, ([X,Y,Z],[QX,QY,QZ,QW])) :-
  % FIXME: why does term expansion not work here? e.g. action_execution unit test
  kb_triple(Transform, 'http://www.ease-crc.org/ont/EASE-OBJ.owl#hasPositionVector', [X,Y,Z]),
  kb_triple(Transform, 'http://www.ease-crc.org/ont/EASE-OBJ.owl#hasOrientationVector', [QX,QY,QZ,QW]),!.

%% transform_invert(+Transform:term, ?Inverted:term) is det.
%
% True if Inverted is the inverted transform of Transform
% (i.e., with inverted reference and source frame).
%
% @param Transform A transform term [A,B,Pos,Rot]
% @param Inverted A transform term [B,A,Pos',Rot']
%
transform_invert([A,B,[TX,TY,TZ],Q],
                 [B,A,T_inv,Q_inv]) :-
  quaternion_inverse(Q,Q_inv),
  X is -TX, Y is -TY, Z is -TZ,
  quaternion_transform(Q_inv,[X,Y,Z],T_inv).

%% transform_reference_frame(+Transform, ?Ref) is det.
%
% True if Ref is the reference frame of Transform.
% Ref is unified with the global coordinate frame if
% Transform does not specify the reference frame.
%
% @param Transform The transform RDF id or term
% @param Ref The name of the reference frame
%
transform_reference_frame([Ref, _, _, _], Ref) :- !.
transform_reference_frame(Transform, Ref) :-
  kb_triple(Transform, 'http://www.ease-crc.org/ont/EASE-OBJ.owl#hasReferenceFrame', Ref), !.
transform_reference_frame(_TransformId, MapFrame) :-
  map_frame_name(MapFrame).

%% transform_close_to(+Transform1:term, +Transform2:term, +Delta:number) is semidet.
%
% True if the squared distance between Transform1 and Transform2
% is less then or equal to Delta.
%
% @param Transform1 Transform term
% @param Transform2 Transform term
% @param Delta Squared distance delta
%
transform_close_to(
  [ReferenceFrame, _, [X1,Y1,Z1], _],
  [ReferenceFrame, _, [X2,Y2,Z2], _], Dmax) :-
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz,
  DmaxSq is Dmax*Dmax,
  Dsq =< DmaxSq.

%% matrix(?Matrix:list, ?Translation:list, ?Quaternion:list) is semidet.
%
% True if Matrix is the transformation matrix build from
% Translation and Quaternion.
%
% @param Matrix A 4x4 matrix
% @param Translation A translation [number x,y,z]
% @param Quaternion A quaternion [number qx,qy,qz,qw]
%
matrix(Matrix, Translation, Quaternion) :-
  ground(Matrix), !,
  matrix_translation(Matrix,Translation),
  matrix_quaternion(Matrix,Quaternion).

matrix(Matrix, Translation, Quaternion) :-
  ground([Translation,Quaternion]), !,
  matrix_create(Quaternion, Translation, Matrix).

%% matrix_translate(+In:list, +Offset:list, ?Out:list) is semidet.
%
% True if Out unifies with In translated by Offset.
%
% @param In A 4x4 matrix
% @param Offset The offset [number x,y,z]
% @param Out A 4x4 matrix
%
matrix_translate([M00, M01, M02, MX,
                  M10, M11, M12, MY,
                  M20, M21, M22, MZ,
                  M30, M31, M32, M33],
                 [OX,OY,OZ],
                 [M00, M01, M02, MX_,
                  M10, M11, M12, MY_,
                  M20, M21, M22, MZ_,
                  M30, M31, M32, M33]) :-
  MX_ is MX + OX,
  MY_ is MY + OY,
  MZ_ is MZ + OZ.

%% quaternion_multiply(+Quaternion1:list, +Quaternion2:list, ?Multiplied:list) is semidet.
%
% True if Multiplied is the result of multiplying Quaternion1
% with Quaternion2.
%
% @param Quaternion1 A quaternion [number qx,qy,qt,qw]
% @param Quaternion2 A quaternion [number qx,qy,qt,qw]
% @param Multiplied A quaternion [number qx,qy,qt,qw]
%

%%% quaternion_inverse(+Quaternion:list, ?Inverse:list) is semidet.
%%
%% True if Inverse is the inverse of Quaternion.
%%
%% @param Quaternion A quaternion [number qx,qy,qt,qw]
%% @param Inverse A quaternion [number qx,qy,qt,qw]
%%

%%% quaternion_transform(+Quaternion:list, +Vector:list, ?Transformed:list) is semidet.
%%
%% True if Transformed unifies with Vector rotated by Quaternion.
%%
%% @param Quaternion A quaternion [number qx,qy,qt,qw]
%% @param Vector A position vector [number x,y,z]
%% @param Transformed A position vector [number x,y,z]
%%
