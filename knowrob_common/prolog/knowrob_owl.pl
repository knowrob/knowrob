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
      class_properties/3%,
%       rdf_instance_from_class/2%,
%       get_timepoint/1,
%       get_timepoint/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_meta class_properties(r,r,r),
            rdf_instance_from_class(r,r),
            get_timepoint(r),
            get_timepoint(+,r).





%% rdf_instance_from_class(+Class, -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers
%
% @param Class   Class describing the type of the instance
% @param Inst    Identifier of the generated instance of Class
rdf_instance_from_class(Class, Instance) :-
    rdf_instance_from_class(Class, _, Instance).



%% rdf_instance_from_class(+Class, +SourceRef -Inst) is nondet.
%
% Utility predicate to generate unique instance identifiers using
% the source reference SourceRef for rdf_assert
%
% @param Class     Class describing the type of the instance
% @param SourceRef Atom as source reference for rdf_assert
% @param Inst      Identifier of the generated instance of Class

:- assert(instance_nr(0)).
rdf_instance_from_class(Class, SourceRef, Instance) :-

  % retrieve global index
  instance_nr(Index),

  % create instance from type
  ((concat_atom(List, '#', Class),length(List,Length),Length>1) -> (
    % Class is already a URI
    T=Class
  );(
    atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', Class, T)
  )),
  atom_concat(T, Index, Instance),

  ( ( nonvar(SourceRef), rdf_assert(Instance, rdf:type, T, SourceRef));
    ( rdf_assert(Instance, rdf:type, T)) ),

  % update index
  retract(instance_nr(_)),
  Index1 is Index+1,
  assert(instance_nr(Index1)).




%% get_timepoint(-T) is det.
%
% Create a timepoint-identifier for the current time
%
% @param T TimePoint instance identifying the current time stamp
%
get_timepoint(T) :-
  set_prolog_flag(float_format, '%.12g'),
  get_time(Ts),
  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', Ts, T),
  rdf_assert(T, rdf:type, knowrob:'TimePoint').

%% get_timepoint(+Diff, -T) is det.
%
% Create a timepoint-identifier for the current time +/- Diff
%
% @param Diff Time difference to the current time
% @param T    TimePoint instance identifying the current time stamp
%
get_timepoint(Diff, Time) :-

  get_time(Ts),

  ((atom_concat('+', Dunit, Diff), atom_concat(DiffSeconds, 's', Dunit),term_to_atom(A, DiffSeconds)) -> (T is Ts + A) ;
   (atom_concat('+', Dunit, Diff), atom_concat(DiffMinutes, 'm', Dunit),term_to_atom(A, DiffMinutes)) -> (T is Ts + 60.0 * A) ;
   (atom_concat('+', Dunit, Diff), atom_concat(DiffHours,   'h', Dunit),term_to_atom(A, DiffHours))   -> (T is Ts + 3600.0 * A) ;

   (atom_concat('-', Dunit, Diff), atom_concat(DiffSeconds, 's', Dunit),term_to_atom(A, DiffSeconds)) -> (T is Ts - A) ;
   (atom_concat('-', Dunit, Diff), atom_concat(DiffMinutes, 'm', Dunit),term_to_atom(A, DiffMinutes)) -> (T is Ts - 60.0 * A) ;
   (atom_concat('-', Dunit, Diff), atom_concat(DiffHours,   'h', Dunit),term_to_atom(A, DiffHours))   -> (T is Ts - 3600.0 * A) ),


  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#timepoint_', T, Time),
  rdf_assert(Time, rdf:type, knowrob:'TimePoint').





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
%
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

  ( owl_restriction(Sup,restriction(SubProp, some_values_from(Val))) ;
    owl_restriction(Sup,restriction(SubProp, has_value(Val))) ).





