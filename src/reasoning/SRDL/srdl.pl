/*
  Copyright (C) 2019 Daniel Beßler
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

:- module(srdl,
  [
        robot_create/2,
        robot_set_urdf/2,
        robot_set_tf_prefix/2,
        robot_tf_prefix/2,
        has_direct_component/3,
        has_component/3,
        component_type/2
  ]).
/** <module> Representation of individual robots in the RDF triple store.

  @author Daniel Beßler
  @license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/model/URDF')).

:- rdf_meta
        robot_create(r,r),
        robot_set_urdf(r,+),
        robot_set_tf_prefix(r,+),
        robot_tf_prefix(r,+),
        has_direct_component(r,r,r),
        has_component(r,r,r),
        component_type(r,r),
        urdf_name_of_range(r,r,?).

%% robot_create(RobotType,Robot) is det.
%
% Create a new robot instance and also auto-instantiate
% its components recursively.
%
% @param RobotType The RDF type of the robot.
% @param Robot The new RDF individual.
%
robot_create(RobotType,Robot) :-
  kb_create(RobotType,Robot),
  subcomponents_create(Robot,RobotType).

% TODO: handle tf prefixes
robot_set_tf_prefix(Robot,TF_prefix) :-
  fail.
robot_tf_prefix(Robot,TF_prefix) :-
  fail.

%%
subcomponents_create(Comp, CompType) :-
  findall(Range-Count,
    ( property_cardinality(CompType,
         ease:hasPhysicalComponent, Range, Count, _),
      Count > 0
    ),
    Pairs
  ),
  % FIXME: also take into account cardinality
  dict_pairs(Dict,_,Pairs),
  pairs_keys(Pairs, Ranges),
  forall(
    owl_most_specific(Ranges, X), (
    get_dict(X, Dict, C),
    component_create(Comp, X, C)
  )).

%%
component_create(PartType,Part) :-
  kb_create(PartType,Part),
  subcomponents_create(Part,PartType).

%%
component_create(_,PartType,_) :-
  \+ rdfs_subclass_of(PartType,dul:'PhysicalObject'), !.

component_create(_,PartType,_) :-
  ( rdfs_subclass_of(PartType,urdf:'Link') ;
    rdf_equal(PartType,dul:'PhysicalObject') ), !.

component_create(_,_,0) :- !.

component_create(Parent,PartType,N) :-
  component_create(PartType,X),
  ( rdfs_individual_of(Parent,dul:'Agent') ->
    kb_assert(Parent,srdlcomp:hasBodyPart,X);
    kb_assert(Parent,ease:hasPhysicalComponent,X) ),
  M is N - 1,
  component_create(Parent,PartType,M).

%% robot_set_urdf(+Robot,+URDF) is det.
%
% Load URDF file into memory and map it into
% RDF triple store while associating the links
% to components of the robot.
% The component-link association is done based on
% URDF names.
%
% @param Robot The RDF name of a robot.
% @param URDF URI to a URDF file.
%
robot_set_urdf(Robot,URDF) :-
  rdf_urdf_load(Robot,URDF),
  forall( has_component(Robot,Comp,CompType), (
    ( kinematic_chain(CompType,BL,ELs) ->
      robot_set_kinematic_(Robot,Comp,BL,ELs) ; true ),
    ( owl_property_range_on_class(CompType,urdf:hasURDFName,literal(type(_,Name))) ->
      robot_link_classify_(Robot,Name,Comp) ; true )
  )),
  forall( rdf_urdf_robot_link(Robot,L), once(
    is_link_of_component(L,_) ;
    ( rdf_urdf_name(L, LName),
      print_message(warning, orphan_link(Robot,LName)) )
  )),
  forall( has_direct_component(Robot,BodyPart,_), once((
    is_composition(BodyPart) ;
    is_classified_link(BodyPart) ;
    print_message(warning, orphan_component(Robot,BodyPart))
  ))).

is_composition(Comp) :-
  rdf_has(Comp,srdlcomp:hasBaseLink,_),
  rdf_has(Comp,srdlcomp:hasEndLink,_),!.

is_classified_link(Comp) :-
  rdfs_individual_of(Comp,dul:'PhysicalObject'),
  \+ rdfs_individual_of(Comp,urdf:'Joint'),
  component_type(Comp,_),!.

%% assert hasBaseLink / hasEndLink
robot_set_kinematic_(Robot,BodyPart,BL,ELs) :-
  get_robot_link(Robot,BL,BaseLink),
  kb_assert(BodyPart,srdlcomp:hasBaseLink,BaseLink),
  %%
  forall( member(N,ELs), (
    get_robot_link(Robot,N,EndLink),
    kb_assert(BodyPart,srdlcomp:hasEndLink,EndLink)
  )).

%% classify a link, i.e. replacing the link in the RDF store
%% with the robot component
robot_link_classify_(Robot,LinkName,Comp) :-
  get_robot_link(Robot,LinkName,Link),
  %%
  forall( rdf(Link,P0,O), kb_assert(Comp,P0,O) ),
  forall( rdf(S,P1,Link), kb_assert(S,P1,Comp) ),
  rdf_retractall(Link,_,_),
  rdf_retractall(_,_,Link).

%%
get_robot_link(Robot,URDF_name,Link) :-
  rdf_has(Robot,urdf:hasLink,Link),
  rdf_urdf_name(Link,URDF_name),!.

get_robot_link(Robot,URDF_name,_) :-
  print_message(warning, undefined_link(Robot,URDF_name)),
  fail.

%%
kinematic_chain(BodyPart,BL,ELs) :-
  urdf_name_of_range(BodyPart,srdlcomp:hasBaseLink,BL),!,
  findall(EL,
    urdf_name_of_range(BodyPart,srdlcomp:hasEndLink,EL),
    ELs
  ).

%% infer urdf name of a property range class
urdf_name_of_range(Class,P,Name) :-
  property_cardinality(Class,P,Restr,C,_), C > 0,
  owl_property_range_on_class(Restr,urdf:hasURDFName,literal(type(_,Name))),!.

%% has_direct_component(?Obj,?Comp,?CompType) is nondet.
%
% Non-transitive hasPhysicalComponent relation excluding joints and links
% that are not classified.
%
% @param Obj The RDF name of a robot.
% @param Comp The RDF name of a body part.
% @param CompType The RDF name of a body part type.
%
has_direct_component(Obj,Comp,CompType) :-
  rdf_has(Obj,ease:hasPhysicalComponent,Comp),
  component_type(Comp,CompType).

%% has_component(?Obj,?Comp,?CompType) is nondet.
%
% Transitive hasPhysicalComponent relation excluding joints and links
% that are not classified.
%
% @param Obj The RDF name of a robot.
% @param Comp The RDF name of a body part.
% @param CompType The RDF name of a body part type.
%
has_component(Obj,Obj,ObjType) :-
  component_type(Obj,ObjType).

has_component(Obj,Comp,CompType) :-
  rdf_has(Obj,ease:hasPhysicalComponent,X),
  has_component(X,Comp,CompType).

%% component_type(?Comp,?CompType) is nondet.
%
% Relation between individual component and its
% component type which is a subclass of dul:'PhysicalObject'
% but not of urdf:'Link' or urdf:'Joint'.
%
% @param Comp The RDF name of a body part.
% @param CompType The RDF name of a body part type.
%
component_type(Comp,CompType) :-
  kb_type_of(Comp,CompType),
  rdfs_subclass_of(CompType,dul:'PhysicalObject'),
  \+ rdfs_subclass_of(CompType,urdf:'Joint'),
  \+ rdf_equal(CompType,dul:'PhysicalObject').

%%
is_link_of_component(Link,Comp) :-
  rdf_has(Comp,srdlcomp:hasBaseLink,Link).

is_link_of_component(Link,Comp) :-
  rdf_has(Joint,urdf:hasChildLink,Link),
  rdf_has(Joint,urdf:hasParentLink,ParentLink),
  is_link_of_component(ParentLink,Comp).
