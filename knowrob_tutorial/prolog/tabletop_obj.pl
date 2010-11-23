/** <module> tabletop_obj

  This module provides routines to interface the the tabletop_object_detector
  system, i.e. to read data and interpret the results.

  Copyright (C) 2010 by Moritz Tenorth

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a ttoy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

@author Moritz Tenorth
@license GPL
*/

:- module(tabletop_obj,
    [
      tabletop_object/2,
      comp_tabletop_object/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).


:- rdf_db:rdf_register_ns(knowrob,  'http://ias.cs.tum.edu/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(tabletop_obj, 'http://ias.cs.tum.edu/kb/tabletop_obj.owl#', [keep(true)]).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% As part of the KnowRob tutorial, you are asked to implement the
% following two predicates. You can use the predicates below for
% creating the object and perception instances.
%


%% tabletop_object(-Obj)
%
% Queries the tabletop_object_detector service and creates the
% internal object representations in the knowledge base.
%
tabletop_object(Obj, Type) :-

    % create ROS client object


    % call the method for retrieving objects from the tabletop_object_detector


    % convert the result into a prolog list over which we can iterate


    % create the object representations in the knowledge base
    member(Match, ObjList),
    create_tabletop_object(Match, Obj, Type).



%% create_tabletop_object(+Match, -Obj)
%
% Create the internal representation of the detection of a tabletop object
%
% This involves to
% - create the object instance of the respective type
% - create a perception instance for the current point in time
% - set the pose where the object was perceived
% - link the object and the perception instance
%

create_tabletop_object(Match, Obj, Type) :-

    % retrieve the model ID


    % retrieve the pose as array of double values

    % create object instance

    % create perception instance

    % set pose

    % link object and perception
    fail.




%% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
%% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %% %%
%% %%
%% %% PROVIDED LIBRARY METHODS FOR CREATING OBJECT AND PERCEPTION INSTANCES
%% %%

comp_tabletop_object(Obj, 'http://ias.cs.tum.edu/kb/knowrob.owl#HumanScaleObject') :-
   tabletop_object(Obj, _).


%% create_perception_instance(-Perception) is det.
%
% Create perception instance for the current point in time
%
create_perception_instance(Perception) :-

  rdf_instance_from_class('http://ias.cs.tum.edu/kb/tabletop_obj.owl#TabletopPerception', Perception),

  % create detection time point
  get_timepoint(TimePoint),
  rdf_assert(Perception, knowrob:startTime, TimePoint).



%% create_object_instance(+ObjTypes, +TabletopID, -Obj) is det.
%
% Create object instance having all the types in ObjTypes
%
create_object_instance(ObjTypes, TabletopID, Obj) :-

  member(ObjType, ObjTypes),
  string_to_atom(ObjType, TypeAtom),
  atom_concat('_', TabletopID, ObjID),
  atom_concat(TypeAtom, ObjID, Obj),

  (rdf_has(Obj, rdf:type, TypeAtom),!;
  rdf_assert(Obj, rdf:type, TypeAtom)),

  string_to_atom(TabletopID, TabletopIDAtom),
  term_to_atom(TabletopIDTerm, TabletopIDAtom),
  rdf_assert(Obj, knowrob:ttoID, literal(type(xsd:int, TabletopIDTerm))).



%% set_object_perception(?A, ?B) is det.
%
% Link the object instance to the perception instance
%
set_object_perception(Object, Perception) :-
  rdf_assert(Perception, knowrob:objectActedOn, Object).



%% set_perception_pose(+Perception, +PoseList) is det.
%
% Set the pose of an object perception
%
set_perception_pose(Perception, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]) :-

  % set the pose
  atomic_list_concat(['rotMat3D_',M00,'_',M01,'_',M02,'_',M03,'_',M10,'_',M11,'_',M12,'_',M13,'_',M20,'_',M21,'_',M22,'_',M23,'_',M30,'_',M31,'_',M32,'_',M33], LocIdentifier),

  atom_concat('http://ias.cs.tum.edu/kb/knowrob.owl#', LocIdentifier, Loc),
  rdf_assert(Loc, rdf:type, knowrob:'RotationMatrix3D'),
  rdf_assert(Perception, knowrob:eventOccursAt, Loc).





%% id_to_type(?TabletopID, ?KnowrobIdentifier) is det.
%
% This predicate defines a simple mapping to translate from any kind of identifier in the tabletop_object_detector
% (class names, model names, etc) into the corresponding identifiers in KnowRob.
%
% @param TabletopID        Object ID in the tabletop_object_detector
% @param KnowrobIdentifier Corresponding atom identifying something in KnowRob
%

id_to_type(18800,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle').%'contact lens cleaner';'{bottle}'
id_to_type(18798,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingCan').%'tennis_ball_can';'{can}'
id_to_type(18665,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingGlass').%'000.580.67';'{glass}'
id_to_type(18685,'http://ias.cs.tum.edu/kb/knowrob.owl#Cup').%'501.245.12'.'{cup}'
id_to_type(18802,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle').%'hydrogen_peroxide_bottle';'{bottle}'
id_to_type(18746,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingCan').%'mach3_gel'.'{can}'
id_to_type(18766,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle').%'suave-kids-3in1';'{bottle}'
id_to_type(18783,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingCan').%'izze_can'.'{can}'
id_to_type(18765,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle').%'coffee-mate';'{bottle}'
id_to_type(18791,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingBottle').%'v8_bottle'.'{bottle}'
id_to_type(18799,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingGlass').%'Clearasil_jar'.'{jar}'
id_to_type(18693,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingGlass').%'801.327.80';'{glass}'
id_to_type(18744,'http://ias.cs.tum.edu/kb/knowrob.owl#DrinkingCan').%'Coke_classic';'{can}'
id_to_type(18807,'http://ias.cs.tum.edu/kb/knowrob.owl#Soup').%'campbell_soup_can';'{can}'
id_to_type(18808,'http://ias.cs.tum.edu/kb/knowrob.owl#Soup').%'campbell_soup_handheld';'{can}'
id_to_type(18699,'http://ias.cs.tum.edu/kb/knowrob.owl#Bowl-Eating').%'901.334.73';'{bowl}'
id_to_type(18691,'http://ias.cs.tum.edu/kb/knowrob.owl#Bowl-Eating').%'800.572.57';'{bowl}'

id_to_type('cereal','http://ias.cs.tum.edu/kb/knowrob.owl#BreakfastCereal').
id_to_type('bowl','http://ias.cs.tum.edu/kb/knowrob.owl#Bowl-Eating').
id_to_type('milk','http://ias.cs.tum.edu/kb/knowrob.owl#CowsMilk-Product').
