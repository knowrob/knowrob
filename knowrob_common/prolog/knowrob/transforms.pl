/** <module> Utilities for handling OWL information in KnowRob.

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

:- module(knowrob_transforms,
    [
      map_frame/1,
      map_frame_name/1,
      matrix_rotation/2,
      matrix_translation/2,
      matrix_translate/3,
      matrix/3,
      transform_invert_topology/2,   % invert child-parent frame relation
      transform_reference_frame/2,   % the reference frame name
      quaternion_multiply/3,
      quaternion_inverse/2,
      quaternion_transform/3,
      transform_multiply/3,
      transform_compute_relative/3,
      transform_data/2,
      transform_close_to/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/rdfs')).

:-  rdf_meta
    transform_reference_frame(r,?),
    transform_invert_topology(r,r).

% TODO unified pose representation
% [Ref, Target, Pos, Rot] vs. pose(Pos, Rot) vs. mat(Mat)

:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

%% map_frame
map_frame('http://knowrob.org/kb/knowrob.owl#MapFrame').
%% map_frame_name
map_frame_name(FrameName) :-
  rdf_has(knowrob:'MapFrame', knowrob:'frameName', literal(type(_,FrameName))), !.
%map_frame_name(FrameName) :-
  %ros_param_get_string('knowrob/map_frame', FrameName)
map_frame_name('map').

matrix_rotation(Matrix, [QW,QX,QY,QZ]) :-
  jpl_list_to_array(Matrix, MatrixArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrixToQuaternion', [MatrixArr], QuaternionArr),
  jpl_array_to_list(QuaternionArr, [QW,QX,QY,QZ]).

matrix_translation(Matrix, [X,Y,Z]) :-
  nth0( 3, Matrix, X),
  nth0( 7, Matrix, Y),
  nth0(11, Matrix, Z).

matrix(Translation, Orientation, Matrix) :-
  jpl_list_to_array(Translation, TranslationArr),
  jpl_list_to_array(Orientation, OrientationArr),
  jpl_call('org.knowrob.utils.MathUtil', 'matrix', [TranslationArr,OrientationArr], MatrixArr),
  jpl_array_to_list(MatrixArr, Matrix).

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

quaternion_multiply(Q0,Q1,Multiplied) :-
  jpl_list_to_array(Q0, Q0_array),
  jpl_list_to_array(Q1, Q1_array),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionMultiply', [Q0_array, Q1_array], Out_array),
  jpl_array_to_list(Out_array, Multiplied).

quaternion_inverse(Q,Q_inv) :-
  jpl_list_to_array(Q, Q_array),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionInverse', [Q_array], Out_array),
  jpl_array_to_list(Out_array, Q_inv).

quaternion_transform(Q,T,T_transformed) :-
  jpl_list_to_array(Q, Q_array),
  jpl_list_to_array(T, T_array),
  jpl_call('org.knowrob.utils.MathUtil', 'quaternionTransform', [Q_array,T_array], Out_array),
  jpl_array_to_list(Out_array, T_transformed).

transform_multiply([RefFrame,       _, [Lx,Ly,Lz], [LQx, LQy, LQz, LQw]],
                   [       _, TgFrame, [Rx,Ry,Rz], [RQx, RQy, RQz, RQw]],
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
  
transform_compute_relative([_,TgFrame, [T1x,T1y,T1z],Q1],
                           [_,RefFrame,[T2x,T2y,T2z],Q2],
                           [RefFrame,TgFrame,TN,QN]) :-
  quaternion_inverse(Q2, Q2_inv),
  quaternion_multiply(Q1, Q2_inv, QN),
  Diff_x is T2x - T1x,
  Diff_y is T2y - T1y,
  Diff_z is T2z - T1z,
  quaternion_transform(Q1, [Diff_x,Diff_y,Diff_z], TN).

transform_data(TransformId, (Translation, Rotation)) :-
  rdf_has_prolog(TransformId, knowrob:translation, Translation),
  rdf_has_prolog(TransformId, knowrob:quaternion, Rotation).

%% transform_invert_topology(+Child, +Parent)
%
% Make Parent relative to Child by inverting the 
% current pose of child which is relative to parent.
%
transform_invert_topology(Child, Parent) :-
  belief_at_relative_to(Child, Parent, RelativePose),
  transform_data(RelativePose, ([TX,TY,TZ],Q)),
  quaternion_inverse(Q,Q_inv),
  X is -TX, Y is -TY, Z is -TZ,
  quaternion_transform(Q_inv,[X,Y,Z],T_inv),
  belief_at_internal(Parent, (T_inv,Q_inv), Child).

%% transform_reference_frame(+TransformId, ?Ref)
%
transform_reference_frame([Ref, _, _, _], Ref) :- !.
transform_reference_frame(TransformId, Ref) :-
  rdf_has(TransformId, knowrob:'relativeTo', RefObjId),
  rdf_has(RefObjId, knowrob:'frameName', literal(Ref)), !.
transform_reference_frame(_TransformId, 'map').

%% transform_close_to(+Transform1, +Transform2, +Dmax)
%
% TODO: support to compare transforms with different reference frame!
transform_close_to(
  [ReferenceFrame, _, [X1,Y1,Z1], _],
  [ReferenceFrame, _, [X2,Y2,Z2], _], Dmax) :-
  Dx is X1 - X2,
  Dy is Y1 - Y2,
  Dz is Z1 - Z2,
  Dsq is Dx*Dx + Dy*Dy + Dz*Dz,
  DmaxSq is Dmax*Dmax,
  Dsq =< DmaxSq.
