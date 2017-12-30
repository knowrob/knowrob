/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2016 Daniel Beßler
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

:- module(owl_export,
    [
      export_tbox/1,
      export_abox/1,
      export_abox/2,
      export_object/2,
      export_object_class/2,
      export_map/2,
      export_action/2
    ]).
/** <module> Methods for exporting triples into OWL files

This module contains methods for exporting triples into OWL files,
for instance object definitions, environment maps, or task specifications.

@author Moritz Tenorth, Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
% TODO 'knowrob/*' modules should not be used here
:- use_module(library('knowrob/utility/filesystem')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/actions')).

:- rdf_db:rdf_register_ns(owl,    'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs,   'http://www.w3.org/2000/01/rdf-schema#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob,'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).


:- rdf_meta export_object(r,r),
      export_object_class(r,r),
      export_map(r,r),
      export_action(r,r),
      rdf_unique_class_id(r, +, r).


:- assert(instance_nr(0)).
rdf_unique_class_id(BaseClass, SourceRef, ID) :-

  instance_nr(Index),
  atom_concat(BaseClass, Index, ID),

  ( ( nonvar(SourceRef), rdf_assert(ID, rdf:type, owl:'Class', SourceRef),!);
    ( rdf_assert(ID, rdf:type, owl:'Class')) ),

  % update index
  retract(instance_nr(_)),
  Index1 is Index+1,
  assert(instance_nr(Index1)),!.


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %


%% export_object(+Obj, -File)
%
% Export the perception of an object to an OWL file.
% Not-existing directories will be created.
%
% @param Obj  Object instance to be exported (including the perception instances)
% @param File Filename for the exported OWL file
%
export_object(Obj, File) :-
  read_object_info(Obj, ObjInfos),
  export_to_owl(ObjInfos, File).


%% export_object_class(+Obj, -File)
%
% Export the definition of an object class to an OWL file.
% Not-existing directories will be created.
%
% @param Obj  Object class to be exported
% @param File Filename for the exported OWL file
%
export_object_class(Obj, File) :-
  read_objclass_info(Obj, ObjInfos),
  export_to_owl(ObjInfos, File).

%% export_map(+Map, -File)
%
% Export the map as the set of all perceptions of objects to an OWL file.
% Not-existing directories will be created.
%
% @param Map  Map instance to be exported (including the perception instances for all objects)
% @param File Filename for the exported OWL file
%
export_map(Map, File) :-
  read_map_info(Map, MapInfos),
  export_to_owl(MapInfos, File).


%% export_action(+Action, -File)
%
% Export an action specification (TBOX) to an OWL file.
% Not-existing directories will be created.
%
% @param Action Action specification to be exported
% @param File   Filename for the exported OWL file
%
export_action(Action, File) :-
  read_action_info(Action, ActionInfos),
  export_to_owl(ActionInfos, File).

%% export_tbox(+File)
%% export_tbox(+Dir)
%
% Export the TBOX to the OWL file @File
% or create OWL files based on individual names
% and save multiple OWL files in the directory @Dir.
% Not-existing directories will be created.
%
% @param File The target file with ".owl" extension
% @param Dir The target directory.
%
export_tbox(File) :-
  file_name_extension(_, 'owl', File),
  findall( Concept, rdfs_subclass_of(Concept, owl:'Thing'), Concepts ),
  sort([Concepts], Sorted),
  export_to_owl(Sorted, File), !.

export_tbox(Dir) :-
  % query TBOX
  findall( (Concept,PathAbsolute), (
    rdfs_subclass_of(Concept, owl:'Thing'),
    rdf_split_url(Prefix, _, Concept),
    atomic_list_concat([URL|_], #, Prefix),
    atomic_list_concat([_,_|PathList], /, URL),
    path_split(PathRelative, PathList),
    path_concat(Dir, PathRelative, PathAbsolute)
  ), AboxInfo ),
  % find set of file paths to be exported
  findall( PathAbsolute, member((_,PathAbsolute), AboxInfo), Paths ),
  sort(Paths, PathsSorted),
  % export owl file for each of the paths
  forall( member(Path, PathsSorted), (
    findall( Concept, member((Concept,Path),AboxInfo), Concepts ),
    sort(Concepts, Sorted),
    export_to_owl(Sorted, Path)
  )).

%% export_abox(+File)
%% export_abox(+Dir)
%% export_abox(+Individual,+File)
%% export_abox(+Individual,+Dir)
%
% Export the ABOX to the OWL file @File
% or create OWL files based on individual names
% and save multiple OWL files in the directory @Dir.
% Not-existing directories will be created.
% If @Individual is specified, then only individuals
% related to @Individual will be exported.
%
% @param File The target file with ".owl" extension
% @param Dir The target directory.
% @param Individual The OWL individual that should be exported recursively.
%
export_abox(File) :-
  file_name_extension(_, 'owl', File),
  findall( Instance, rdf_has(Instance, rdf:type, owl:'NamedIndividual'), Individuals ),
  sort([Individuals], Sorted),
  export_to_owl(Sorted, File), !.

export_abox(Dir) :- export_abox_separated(Dir, all).

export_abox(Individual, File) :-
  file_name_extension(_, 'owl', File),
  findall( Related, owl_has(Individual, _, Related), RelatedInstances ),
  sort([Individual|RelatedInstances], Sorted),
  export_to_owl(Sorted, File), !.

export_abox(Dir, Instance) :- export_abox_separated(Dir, Instance).

export_abox_separated(Dir, Individual) :-
  % query ABOX
  findall( (Instance,PathAbsolute), (
    (  Individual=all
    -> rdf_has(Instance, rdf:type, owl:'NamedIndividual')
    ;  owl_has(Individual, _, Instance)
    ),
    rdf_split_url(Prefix, _, Instance),
    atomic_list_concat([URL|_], #, Prefix),
    atomic_list_concat([_,_|PathList], /, URL),
    path_split(PathRelative, PathList),
    path_concat(Dir, PathRelative, PathAbsolute)
  ), AboxInfo ),
  % find set of file paths to be exported
  findall( PathAbsolute, member((_,PathAbsolute), AboxInfo), Paths ),
  sort(Paths, PathsSorted),
  % export owl file for each of the paths
  forall( member(Path, PathsSorted), (
    findall( Instance, member((Instance,Path),AboxInfo), Instances ),
    sort(Instances, InstancesSorted),
    export_to_owl(InstancesSorted, Path)
  )).

%% export_to_owl(+Atoms, -File)
%
% Write all information about Atoms to a file using rdf_save_subject
%
% Atoms is a list that is to be generated by read_object_info, read_map_info, or read_action_info
%
% @param Atoms  List of OWL identifiers that are to be exported using rdf_save_subject
% @param File   Filename for the exported OWL file
%
export_to_owl(Atoms, File) :-

  file_directory_name(File, Dir), mkdir(Dir),

  open(File, write, Stream, [encoding('ascii')]),
  rdf_save_header(Stream, [write_xml_base(true)]),

  findall(_, (
    member(Atom, Atoms),
    ((atom(Atom),
      rdf_save_subject(Stream, Atom, _))
    ; true)
  ), _),

  rdf_save_footer(Stream),
  close(Stream).



%% read_object_info(+Inst, -ObjInfosSorted)
%
% Collect information about object perceptions and assemble it into the ObjInfosSorted list.
%
% @param Inst            Object instance that is to be exported
% @param ObjInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_object_info(Inst, ObjInfosSorted) :-

  % read all direct properties
  findall(Prop, (
              owl_has(Inst,P,Prop),
              rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')
          ), Properties),


  % read all direct properties
  findall(PartInfo, (
              rdf_reachable(Inst, 'http://knowrob.org/kb/knowrob.owl#properPhysicalParts', Part),
              Part \= Inst,
              read_object_info(Part, PartInfo)
          ), PartInfos),

  % read all perception instances
  findall(Perc, (
              owl_has(Perc, 'http://knowrob.org/kb/knowrob.owl#objectActedOn', Inst),
              owl_individual_of(Perc, 'http://knowrob.org/kb/knowrob.owl#MentalEvent')
        ), Perceptions),

  % read properties of all perception instances
  findall(PercProp, (
              member(Perc, Perceptions),
              owl_has(Perc,P,PercProp),
              rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')
          ), PercProperties),

  append([Properties,PartInfos,Perceptions,PercProperties], ObjInfos),
  flatten(ObjInfos, ObjInfosFlat),
  sort(ObjInfosFlat, ObjInfosSorted).



%% read_map_info(+Map, -MapInfosSorted)
%
% Collect information about the map and all object perceptions in it and assemble it into the MapInfosSorted list.
%
% @param Map             Map instance that is to be exported
% @param MapInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_map_info(Map, MapInfosSorted) :-

  % read all objects in the map (knowrob:describedInMap) ...
  findall(Obj,   (owl_has(Obj, 'http://knowrob.org/kb/knowrob.owl#describedInMap', Map)), RootObjs),

  % ... and their parts
  findall(Part, (member(Obj, RootObjs),
                  rdf_reachable(Obj, 'http://knowrob.org/kb/knowrob.owl#parts', Part)), Parts),

  % ... as well as the things they are part of (e.g. street, building)
  findall(Super, (member(Obj, RootObjs),
                  rdf_reachable(Super, 'http://knowrob.org/kb/knowrob.owl#parts', Obj)), SuperRootObjs),

  append([SuperRootObjs, RootObjs, Parts], Objs),

  % combine information for each of these objects and object parts
  findall(MapInfo, (member(Obj, Objs),read_object_info(Obj, MapInfo)), MapInfos),
  flatten(MapInfos, MapInfosFlat),
  sort(MapInfosFlat, MapInfosSorted).



%% read_action_info(+Action, -ActionInfosSorted)
%
% Collect information about the action Action and assemble it into the ActionInfosSorted list.
%
% @param Action             Action class that is to be exported
% @param ActionInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_action_info(Action, ActionInfosSorted) :-

  % recursively read all sub-actions of the action
  findall(SubAction, plan_subevents_recursive(Action, SubAction), SubActions),
  append([Action], SubActions, Actions),

  % read all properties for each of them
  findall(Value, (member(Act, Actions), owl_class_properties(Act, _, Value)), ActionProperties),

  % read everything related to these things by an ObjectProperty
  findall(PropVal, (member(ActProp, ActionProperties),
                    owl_has(ActProp, P, PropVal),
                    rdfs_individual_of(P, 'http://www.w3.org/2002/07/owl#ObjectProperty')), ActionPropProperties),

  append([Actions, ActionProperties, ActionPropProperties], ActionInfos),
  flatten(ActionInfos, ActionInfosFlat),
  sort(ActionInfosFlat, ActionInfosSorted).



%% read_objclass_info(+ObjClass, -ObjClassInfosSorted)
%
% Collect information about the object class ObjClass and assemble it into the ObjClassInfosSorted list.
%
% @param ObjClass             Object class that is to be exported
% @param ObjClassInfosSorted  List of all related OWL identifiers (to be used in rdf_save_subject)
%
read_objclass_info(ObjClass, ObjClassInfosSorted) :-

%   findall(ObjSuperClass, (owl_direct_subclass_of(ObjClass, ObjSuperClass), not(is_bnode(ObjSuperClass))), ObjSuperClasses),

  % read all parts of the object class to be exported
  findall(ObjPart, (owl_class_properties_transitive_nosup(ObjClass, knowrob:parts, ObjPart), not(is_bnode(ObjPart))), ObjParts),

%   append([[ObjClass], ObjParts, ObjSuperClasses], ObjClassDefs),
  append([[ObjClass], ObjParts], ObjClassDefs),
  sort(ObjClassDefs, ObjClassDefsSorted),

  % read all properties for each of them
  findall(ObjPr, (member(ObjCl,ObjClassDefsSorted),
                  owl_class_properties_nosup(ObjCl, P, ObjPr),
                  not(rdfs_subproperty_of(P, owl:subClassOf))), ObjProperties),

  % read everything related to these things by an ObjectProperty
  findall(PropVal, (member(ObjProp, ObjProperties),
                    rdf_has(ObjProp, P, PropVal), not(P='http://www.w3.org/2000/01/rdf-schema#subClassOf'),
                    not(is_bnode(PropVal))), ObjClassPropProperties),

  append([ObjClassDefsSorted, ObjProperties, ObjClassPropProperties], ObjClassInfos),

  flatten(ObjClassInfos, ObjClassInfosFlat),
  sort(ObjClassInfosFlat, ObjClassInfosSorted).


is_bnode(Node) :-
  atom(Node),
  sub_string(Node,0,2,_,'__').
