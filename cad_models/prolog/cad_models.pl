/** <module> cad_models

  Description:
    Module providing predicates for cad model examination


  Copyright (C) 2011 by Stefan Profanter

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

@author Stefan Profanter
@license GPL
*/

:- module(cad_models,
    [
      cad_model_xdim/2,
      cad_model_ydim/2,
      cad_model_zdim/2
    ]).

:- register_ros_package(knowrob_common).
use_module(library('cad_models/knowrob_owl')).
% :- use_module(library('knowrob_common/knowrob_owl')).

:- owl_parser:owl_parse('/home/stefan/ros/knowrob/cad_models/owl/cad_models.owl', false, false, true).

:- rdf_meta cad_model_xdim(r,t),
            cad_model_ydim(r,t),
            cad_model_zdim(r,t).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Model Properties
%

%% cad_model_xdim(+Prop, -Dim) is det.
%
% Get x_dimension (width) of model identified by Prop.
% Prop may be a Class name or an instance id
%
cad_model_xdim(Prop, Dim) :-
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
    jpl_new('edu.tum.cs.vis.model.Properties', [], Properties),
    jpl_call(Properties, 'xDimOfObject', [SubProp], Dim).


%% cad_model_ydim(+Prop, -Dim) is det.
%
% Get y_dimension (depth) of model identified by Prop.
% Prop may be a Class name or an instance id
%
cad_model_ydim(Prop, Dim) :-
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
    jpl_new('edu.tum.cs.vis.model.Properties', [], Properties),
    jpl_call(Properties, 'yDimOfObject', [SubProp], Dim).


%% cad_model_zdim(+Prop, -Dim) is det.
%
% Get z_dimension (height) of model identified by Prop.
% Prop may be a Class name or an instance id
%
cad_model_zdim(Prop, Dim) :-
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
    jpl_new('edu.tum.cs.vis.model.Properties', [], Properties),
    jpl_call(Properties, 'zDimOfObject', [SubProp], Dim).
