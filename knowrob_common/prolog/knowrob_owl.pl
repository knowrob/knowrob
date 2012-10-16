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
      class_properties/3,
      class_properties_some/3,
      class_properties_all/3,
      class_properties_value/3,
      create_restr/6,
      rdf_instance_from_class/2,
      rdf_instance_from_class/3,
      get_timepoint/1,
      get_timepoint/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).

:- rdf_meta class_properties(r,r,t),
            class_properties_some(r,r,t),
            class_properties_all(r,r,t),
            class_properties_value(r,r,t),
            rdf_instance_from_class(r,r),
            rdf_instance_from_class(r,r,r),
            get_timepoint(r),
            get_timepoint(+,r),
            create_restr(r, r, r, r, +, r).

:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs,   'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).




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

  ( ( nonvar(SourceRef), rdf_assert(Instance, rdf:type, T, SourceRef),!);
    ( rdf_assert(Instance, rdf:type, T)) ),

  % update index
  retract(instance_nr(_)),
  Index1 is Index+1,
  assert(instance_nr(Index1)).



%% create_restr(+Class, +Prop, +Value, +RestrType, +SourceRef, -Restr) is det.
%
% Create a restriction for property Prop and value Value on class Class
% with the sourceRef SourceRef
%
% @param Class     Class that is to be annotated with the restriction
% @param Prop      Property to be used for the restriction
% @param Value     Value to be used for the restriction
% @param RestrType Restriction type as OWL identifier, e.g. owl:someValuesFrom or owl:hasValue
% @param SourceRef Atom as source reference for rdf_assert
%
create_restr(Class, Prop, Value, RestrType, SourceRef, Restr) :-

  rdf_node(Restr),
%   rdf_assert(Restr, rdf:'type', owl:'Restriction', SourceRef),

  rdf_assert(Class, rdfs:'subClassOf', Restr, SourceRef),

%   assert(blanknode(Prop, someValuesFrom(Value), used)),

  rdf_assert(Restr, owl:'onProperty', Prop, SourceRef),
  rdf_assert(Restr, RestrType, Value, SourceRef).



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





%% class_properties(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom- and hasValue-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties(Class, Prop, Val) :-
  (class_properties_some(Class, Prop, Val);
   class_properties_value(Class, Prop, Val)).




%% class_properties_some(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_some(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_some(Class, Prop, Val).

class_properties_some(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_some(Super, Prop, Val).


class_properties_1_some(Class, Prop, Val) :-       % read all values for some_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, some_values_from(Val))).




%% class_properties_some(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_all(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_all(Class, Prop, Val).

class_properties_all(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_all(Super, Prop, Val).


class_properties_1_all(Class, Prop, Val) :-       % read all values for all_values_from restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, all_values_from(Val))) .




%% class_properties_some(?Class, ?Prop, ?Val) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class Class whose restrictions are being considered
% @param Prop  Property whose restrictions in Class are being considered
% @param Val   Values that appear in a restriction of a superclass of Class on Property
%
class_properties_value(Class, Prop, Val) :-         % read directly asserted properties
  class_properties_1_value(Class, Prop, Val).

class_properties_value(Class, Prop, Val) :-         % also consider properties of superclasses
  owl_subclass_of(Class, Super), Class\=Super,
  class_properties_1_value(Super, Prop, Val).


class_properties_1_value(Class, Prop, Val) :-       % read all values for has_value restrictions

  ( (nonvar(Class)) -> (owl_direct_subclass_of(Class, Sup)) ; (Sup     = Class)),
  ( (nonvar(Prop))  -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),

  owl_restriction(Sup,restriction(SubProp, has_value(Val))) .


