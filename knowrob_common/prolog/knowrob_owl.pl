%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides utilities for handling OWL information
%% in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


:- module(knowrob_owl,
    [
      class_properties/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_meta(class_properties(r,r,r)).


%% class_properties(?Class, ?Prop, ?Values) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class   Class whose restrictions are being considered
% @param Prop    Property whose restrictions in Class are being considered
% @param Values  List of all classes that appear in a restriction of a superclass of Class along Property

class_properties(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1(Class, Prop, Val).

class_properties(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1(Super, Prop, Val).

% read restrictions defined for Class for Prop or a sub-property of Prop

% TODO: Do we need this alternative? results in double results, and super-classes
% should already be handled by the second alternative of class_properties
%
% class_properties_1(Class, Prop, Val) :-
%   owl_direct_subclass_of(Class, Sup),
%   owl_direct_subclass_of(Sup, Sup2),
%   ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
%   owl_restriction(Sup2,restriction(SubProp, some_values_from(Val))).

class_properties_1(Class, Prop, Val) :-
  owl_direct_subclass_of(Class, Sup),
  ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
  owl_restriction(Sup,restriction(SubProp, some_values_from(Val))).
