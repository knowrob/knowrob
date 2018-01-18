/*
  Copyright (C) 2015 Daniel Beßler
  Copyright (C) 2015 Asil Kaan Bozcuoğlu
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

:- module(marker_vis,
    [
      marker_publish/0,
      marker_republish/0,
      marker/2,
      marker/3,
      marker_update/0,
      marker_update/1,
      marker_update/2,
      marker_remove/1,
      marker_child/2,
      marker_object/4,
      
      marker_properties/2,
      marker_type/2,
      marker_scale/2,
      marker_mesh_resource/2,
      marker_pose/2,
      marker_points/2,
      marker_tf_prefix/2,
      marker_translation/2,
      marker_text/2,
      marker_color/2,
      marker_colors/2,
      marker_has_visual/2,
      
      marker_hide/1,
      marker_show/1,
      
      marker_highlight/1,
      marker_highlight/2,
      marker_highlight_remove/1,
      marker_highlight_toggle/1,
      
      marker_primitive/5,
      marker_queries/2
    ]).
/** <module> Integration RViz marker visualization messages

  @author Daniel Beßler
  @license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('jpl')).
:- use_module(library('lists')). % for sum_list
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/transforms')).

:- rdf_meta marker(t,?),
            marker(t,?,?),
            marker_hide(t),
            marker_show(t),
            marker_update(t),
            marker_update(t,r),
            marker_remove(t),
            marker_children(t,?),
            marker_type(t,?),
            marker_scale(t,?),
            marker_color(t,?),
            marker_has_visual(t,?),
            marker_highlight(t,?),
            marker_highlight_remove(t),
            marker_highlight_toggle(t),
            marker_mesh_resource(t,?),
            marker_pose(t,?),
            marker_translation(t,?),
            marker_tf_prefix(t,?),
            marker_text(t,?),
            marker_properties(t,?),
            marker_queries(+,-).

% define marker_new and marker_update as meta-predicate and allow the definitions
% to be in different source files
%:- meta_predicate marker_new(0, ?, ?, ?, ?),
%                  marker_update(0, ?, ?, ?, ?).
:- multifile marker_new/4,
             marker_update/3.

:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

marker_has_visual(Identifier) :-
  not(owl_individual_of(Identifier, srdl2comp:'UrdfJoint')),
  not(owl_individual_of(Identifier, knowrob:'RoomInAConstruction')),
  not(owl_individual_of(Identifier, knowrob:'SemanticEnvironmentMap')),
  not(rdf_has(Identifier, knowrob:'hasVisual', literal(type(_,false)))).

marker_children(Parent, Children) :-
  findall(Child, (
    rdf_reachable(Part, knowrob:describedInMap, Parent),
    (
      rdf_reachable(Part, knowrob:'parts', Child);
      rdf_reachable(Part, srdl2comp:'subComponent', Child);
      rdf_reachable(Part, srdl2comp:'successorInKinematicChain', Child)
    ),
    not(Child = Parent)
  ), ChildrenList),
  list_to_set(ChildrenList, Children).

marker_links(Identifier, Links) :-
  findall(Link, (
    rdf_reachable(Identifier, srdl2comp:'subComponent', Component),
    owl_has(Component, srdl2comp:'baseLinkOfComposition', BaseLink),
    rdf_reachable(BaseLink, srdl2comp:'successorInKinematicChain', Link),
    owl_individual_of(Link, srdl2comp:'UrdfLink')
  ), LinksList),
  list_to_set(LinksList, Links).

marker_succeeding_links(Link0, Links) :-
  findall(Link1, (
    rdf_has(Link0, srdl2comp:'successorInKinematicChain', Joint),
    rdf_has(Joint, srdl2comp:'successorInKinematicChain', Link1)
  ), Links).

marker_srdl_tf_frame(Identifier, UrdfName) :-
  rdf_has(Identifier, knowrob:'frameName', literal(UrdfName)), !.
marker_srdl_tf_frame(Identifier, UrdfName) :-
  not( atom_prefix(Identifier, 'http') ),
  UrdfName = Identifier.

marker_tf_frame(MarkerObject, Identifier, TfFrame) :-
  marker_srdl_tf_frame(Identifier, UrdfName),
  atom_ensure_prefix(UrdfName, '/', UrdfNameResolved),
  marker_tf_prefix(MarkerObject, Prefix),
  (  Prefix = '/'
  -> TfFrame = UrdfNameResolved
  ;  (
   atom_ensure_prefix(Prefix, '/', PrefixResolved),
   atom_concat(PrefixResolved, UrdfNameResolved, TfFrame)
  )).
  
marker_lookup_transform(MarkerObject, Identifier, T, (Translation,Orientation)) :-
  rdfs_individual_of(Identifier, knowrob:'Designator'),
  rdf_split_url(Prefix, ObjName, Identifier),
  atomic_list_concat([Prefix,'Object_',ObjName], Object),
  rdfs_individual_of(Object, knowrob:'SpatialThing-Localized'),
  marker_lookup_transform(MarkerObject, Object, T, (Translation,Orientation)), !.

marker_lookup_transform(_, Identifier, T, (Translation,Orientation)) :-
  map_frame_name(MapFrame),
  object_pose_at_time(Identifier, T, [MapFrame, _, Translation, Orientation]), !.

marker_transform_at_time(MarkerObject, Identifier, T, Pose) :-
  marker_lookup_transform(MarkerObject, Identifier, T, Pose)
  -> marker_show(MarkerObject)
  ;  marker_hide(MarkerObject).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker constants
%

marker_prop_type(arrow,0).
marker_prop_type(cube,1).
marker_prop_type(sphere,2).
marker_prop_type(cylinder,3).
marker_prop_type(line_strip,4).
marker_prop_type(line_list,5).
marker_prop_type(cube_list,6).
marker_prop_type(sphere_list,7).
marker_prop_type(points,8).
marker_prop_type(text_view_facing,9).
marker_prop_type(mesh_resource,10).
marker_prop_type(triangle_list,11).
marker_prop_type(sprite_scaled,999994).
marker_prop_type(background_image,999995).
marker_prop_type(hud_image,999996).
marker_prop_type(hud_text,999997).
marker_prop_type(sprite,999998).
marker_prop_type(sprite_text,999999).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% ROS node for marker visualization
%

marker_visualisation :-
  marker_visualisation(_).
  
marker_visualisation(MarkerVis) :-
  (\+ current_predicate(v_marker_vis, _)),
  jpl_call('org.knowrob.vis.MarkerPublisher', get, [], MarkerVis),
  jpl_list_to_array(['org.knowrob.vis.MarkerPublisher'], Arr),
  jpl_call('org.knowrob.utils.ros.RosUtilities', runRosjavaNode, [MarkerVis, Arr], _),
  assert(v_marker_vis(MarkerVis)),!.

marker_visualisation(MarkerVis) :-
  current_predicate(v_marker_vis, _),
  v_marker_vis(MarkerVis).

%% marker_publish is det.
%
% Publish all queued messages on the ROS /marker_visualisation topic
% and remove them from queue.
%
marker_publish :-
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'publishMarker', [], _).

%% marker_republish is det.
%
% (Re)-publish all known marker messages on the ROS /marker_visualisation topic.
%
marker_republish :-
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'republishMarker', [], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker factory
%

%% marker_create(+MarkerName, +MarkerTerm, -MarkerObject, +Parent) is det.
%
% Creates a new MarkerObject instance that is identified by MarkerTerm.
%
% @param MarkerName A atom that identifies the new marker (e.g., 'PR2')
% @param MarkerTerm A term that identifies the new marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject object created
% @param Parent The parent Java object (i.e., MarkerPublisher or MarkerObject instance)
%
marker_create(MarkerName, MarkerTerm, MarkerObject, Parent) :-
  compound(MarkerTerm), atom(MarkerName), ground(MarkerTerm),
  jpl_call(Parent, 'createMarker', [MarkerName], MarkerObject),
  assert( v_marker_object(MarkerName, MarkerTerm, MarkerObject, Parent) ).

marker_create(MarkerTerm, MarkerObject, Parent) :-
  compound(MarkerTerm),
  term_to_atom(MarkerTerm, MarkerAtom),
  marker_create(MarkerAtom, MarkerTerm, MarkerObject, Parent).

%% marker_object(?Name,?Object,?Parent,?Term) is nondet.
%% marker_object(?Name,?Object,?Parent) is nondet.
%% marker_object(?Name,?Object) is nondet.
%
% Maps marker identification term to MarkerObject instance.
%
% @param Name The marker identification term
% @param Object The MarkerObject instance
% @param Parent The MarkerObject parent instance
% @param Term The marker identification term
%
marker_object(Name,Object,Parent,Term) :-
  current_predicate(v_marker_object,_),
  v_marker_object(Name,Term,Object,Parent).

marker_object(Name,Object,Parent) :-
  current_predicate(v_marker_object,_),
  v_marker_object(Name,_,Object,Parent).

marker_object(Name,Object) :-
  marker_object(Name,_,Object,_).

%% marker_initialize_object(+Identifier,+MarkerObject) is det.
%
% Read some default visualization properties from OWL individual.
%
% @param Identifier The name of the OWL individual
% @param MarkerObject The corresponding MarkerObject instance
%
marker_initialize_object(Identifier,MarkerObject) :-
  (  marker_has_visual(Identifier)
  -> marker_has_visual(MarkerObject,true)
  ;  marker_has_visual(MarkerObject,false)
  ),
  ignore((
    rdfs_individual_of(Identifier, knowrob:'Designator'),
    % TODO: use existing instance if available
    mng_designator_timestamp(Identifier, T),
    mng_designator_location(Identifier, LocList),
    owl_instance_from_class(knowrob:'TimePoint', [instant=T], Timepoint),
    owl_instance_from_class(knowrob:'Pose', [mat=LocList], Loc),
    add_object_as_semantic_instance(Identifier, Loc, Timepoint, Instance),
    marker_initialize_object(Instance,MarkerObject)
  )),
  ignore((
    %not( marker_type(MarkerObject, mesh_resource) ),
    object_dimensions(Identifier, X, Y, Z),
    marker_scale(MarkerObject, [X, Y, Z])
  )),
  ignore((
    get_model_path(Identifier, Path),
    marker_type(MarkerObject, mesh_resource),
    marker_mesh_resource(MarkerObject, Path),
    marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
    marker_scale(MarkerObject, [1.0,1.0,1.0]),
    
    ( rdf_has(Identifier, srdl2comp:'mesh_scale', literal(type(_,Scale)))
    -> (
      rdf_vector_prolog(Scale, ScaleVec),
      marker_scale(MarkerObject, ScaleVec)
    ) ; true )
  )),
  ignore((
    object_color(Identifier, Color),
    marker_color(MarkerObject, Color)
  )).

%% marker_primitive(+Type, +MarkerName, +MarkerTerm, -MarkerObject, +Parent) is det.
%
% Creates MarkerObject instance with primitive geometry shape (e.g., cube, sphere, ...)
% and some default properties.
%
% @param Type The shape primitive type (defined by marker_prop_type/2)
% @param MarkerName A atom that identifies the new marker (e.g., 'PR2')
% @param MarkerTerm A term that identifies the new marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject object created
% @param Parent The parent Java object (i.e., MarkerPublisher or MarkerObject instance)
%
marker_primitive(Type, MarkerName, MarkerTerm, MarkerObject, Parent) :-
  marker_create(MarkerName, MarkerTerm, MarkerObject, Parent),
  marker_type(MarkerObject, Type),
  marker_color(MarkerObject, [0.6,0.6,0.6,1.0]),
  marker_scale(MarkerObject, [0.05,0.05,0.05]).

%% marker_remove(+MarkerTerm) is det.
%
% Removes MarkerObject instances that match given term.
% marker_remove(all) removes all known markers.
% marker_remove(trajectories) removes all known trajectory markers.
% marker_remove(MarkerTerm) removes all markers that can be unified with
% the compound term MarkerTerm.
% 
% @param MarkerTerm Marker identification term.
%
marker_remove(all) :-
  ignore( forall(
    v_marker_object(_, _, MarkerObject, _),
    ignore(marker_remove(MarkerObject))
  )), !.

marker_remove(trajectories) :-
  ignore( forall(
    v_marker_object(_, trajectory(_), MarkerObject, _),
    ignore(marker_remove(MarkerObject))
  )), !.

marker_remove(MarkerObject) :-
  jpl_is_object(MarkerObject),
  marker_visualisation(MarkerVis),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  jpl_call(MarkerVis, 'eraseMarker', [MarkerObject], _),
  ignore((
    retract( v_marker_object(_, _, MarkerObject, _) )
  )),
  forall( member(ChildObject,Children), marker_remove(ChildObject) ).

marker_remove(MarkerTerm) :-
  compound(MarkerTerm), ground(MarkerTerm),
  not( jpl_is_object(MarkerTerm) ),
  term_to_atom(MarkerTerm, MarkerName),
  forall(
    v_marker_object(MarkerName, _, MarkerObject,_),
    marker_remove(MarkerObject)
  ).

marker_remove(MarkerName) :-
  atom(MarkerName),
  forall(
    v_marker_object(MarkerName, _, MarkerObject,_),
    marker_remove(MarkerObject)
  ).

%% marker_child(+MarkerTerm,?MarkerChildTerm) is nondet.
%
% Query the children markers of the marker identified by MarkerTerm.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerChildTerm A term that identifies the marker child
marker_child(MarkerTerm, MarkerChildTerm) :-
  marker(MarkerTerm, MarkerObject),
  marker_child(MarkerObject, MarkerChildTerm).

marker_child(MarkerObject, MarkerChildTerm) :-
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  member(ChildObject,Children),
  v_marker_object(MarkerChildTerm, _, ChildObject, _).

%% marker(?MarkerTerm,?MarkerObject) is det.
%% marker(?MarkerTerm,?MarkerObject,?Markername) is det.
%
% Selects MarkerObject instance that corresponds to MarkerTerm.
% Possibly creates a new MarkerObject if MarkerTerm is unknown.
% The MarkerPublisher instance is automatically selected as parent
% if the marker object is newly created with this call.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
%
marker(MarkerTerm, MarkerObject) :-
  compound(MarkerTerm),
  not(ground(MarkerTerm)), % matching many terms
  marker_visualisation(MarkerVis),
  marker(_, MarkerTerm, MarkerObject, MarkerVis).

marker(MarkerTerm, MarkerObject) :-
  compound(MarkerTerm),
  ground(MarkerTerm), % matching one term
  term_to_atom(MarkerTerm, MarkerAtom),
  marker(MarkerTerm, MarkerObject, MarkerAtom).

marker(MarkerName, MarkerObject) :-
  ( atom(MarkerName) ; var(MarkerName) ),
  marker_visualisation(MarkerVis),
  marker(MarkerName, _, MarkerObject, MarkerVis).
  
marker(MarkerTerm, MarkerObject, MarkerName) :-
  marker_visualisation(MarkerVis),
  marker(MarkerName, MarkerTerm, MarkerObject, MarkerVis).

%% marker(?MarkerName,?MarkerTerm,?MarkerObject,+Parent) is det.
%
% Selects MarkerObject instance that corresponds to MarkerTerm.
% Possibly creates a new MarkerObject if MarkerTerm is unknown.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
% @param Parent The parent of MarkerObject instance
%
marker(MarkerName, MarkerTerm, MarkerObject, Parent) :-
  findall(X, marker_object(MarkerName, X, _, MarkerTerm), MarkerObjects),
  (  length(MarkerObjects,0)
  -> (
       ground(MarkerName),
       ground(MarkerTerm),
       marker_new(MarkerName, MarkerTerm, MarkerObject, Parent)
     )
  ;  (
       marker_object(MarkerName, MarkerObject, _, MarkerTerm)
     )
  ).

marker_new(MarkerName, cube(Name), MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName, cube(Name), MarkerObject, Parent).

marker_new(MarkerName, sphere(Name), MarkerObject, Parent) :-
  marker_primitive(sphere, MarkerName, sphere(Name), MarkerObject, Parent).

marker_new(MarkerName, arrow(Name), MarkerObject, Parent) :-
  marker_primitive(arrow, MarkerName, arrow(Name), MarkerObject, Parent).

marker_new(MarkerName, cylinder(Name), MarkerObject, Parent) :-
  marker_primitive(cylinder, MarkerName, cylinder(Name), MarkerObject, Parent).

marker_new(MarkerName, cylinder_tf(From,To), MarkerObject, Parent) :-
  marker_primitive(cylinder, MarkerName, cylinder_tf(From,To), MarkerObject, Parent).

marker_new(MarkerName, link(Link), MarkerObject, Parent) :-
  marker_primitive(arrow, MarkerName, link(Link), MarkerObject, Parent).

marker_new(MarkerName, trajectory(Link), MarkerObject, Parent) :-
  marker_primitive(points, MarkerName, trajectory(Link), MarkerObject, Parent),
  marker_color(MarkerObject, [1.0,1.0,0.0,1.0]).

marker_new(MarkerName, pointer(From,To), MarkerObject, Parent) :-
  marker_primitive(arrow, MarkerName, pointer(From,To), MarkerObject, Parent).

marker_new(MarkerName, mesh(Name), MarkerObject, Parent) :-
  marker_create(MarkerName, mesh(Name), MarkerObject, Parent),
  marker_type(MarkerObject, mesh_resource),
  marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker_new(MarkerName, mesh(Name,MeshFile), MarkerObject, Parent) :-
  marker_new(MarkerName, mesh(Name), MarkerObject, Parent),
  marker_mesh_resource(MarkerObject, MeshFile).

marker_new(MarkerName, object_without_children(Identifier), MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName, object_without_children(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier, MarkerObject).

% special case particles (e.g., a pile of stuff)
marker_new(MarkerName, object(Identifier), MarkerObject, Parent) :-
  rdf_has(Identifier, srdl2comp:'urdfName', UrdfVal),
  rdf_has(Identifier, knowrob:'numParticles', literal(type(_,ParticleCount))),
  strip_literal_type(UrdfVal,UrdfFormat),
  atom_number(ParticleCount, N),
  
  marker_primitive(sphere, MarkerName, object(Identifier), MarkerObject, Parent),
  marker_color(MarkerObject, [1.0,1.0,0.0,1.0]),
  marker_has_visual(MarkerObject, false),
  marker_new_particles(MarkerObject, Identifier, UrdfFormat, N, 1), !.

marker_new(MarkerName, object(Identifier), MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName, object(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier, MarkerObject),
  marker_children(Identifier,Children),
  forall(
    member( Child,Children ), ignore((
      marker_term(Child, ChildTerm),
      marker_child_name(object(Identifier), MarkerName, ChildTerm, ChildName),
      marker(ChildName, ChildTerm, _, MarkerObject)
    ))
  ).

marker_new(MarkerName, experiment(Identifier), MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName, object(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier, MarkerObject),
  % Semantic map
  rdf_has(Identifier, knowrob:'performedInMap', Map),
  marker_child_name(experiment(Identifier), MarkerName,
                    object(Map), MapName),
  marker_new(MapName, object(Map), _, MarkerObject),
  % TODO: knowrob:robotActor, knowrob:humanActor don't exist
  forall( % robots
    owl_has(Map, knowrob:robotActor, Agent), ignore((
      marker_child_name(experiment(Identifier), MarkerName,
                        agent(Agent), AgentName),
      marker(AgentName, agent(Agent), _, MarkerObject)
    ))
  ),
  forall( % humans
    owl_has(Map, knowrob:humanActor, Human), ignore((
      marker_child_name(experiment(Identifier), MarkerName,
                        stickman(Human), HumanName),
      marker(HumanName, stickman(Human), _, MarkerObject)
    ))
  ).
  
marker_new(MarkerName, attached(MarkerTerm,AttachedTo), MarkerObject, Parent) :-
  marker_primitive(cube, MarkerName, attached(MarkerTerm,AttachedTo), MarkerObject, Parent),
  marker_has_visual(MarkerObject, false),
  term_to_atom(MarkerTerm, MarkerAtom),
  marker(MarkerAtom, MarkerTerm, _, MarkerObject).

marker_new(MarkerName, agent(Identifier), MarkerObject, Parent) :-
  marker_new(MarkerName, kinematic_chain(Identifier,agent(Identifier)), MarkerObject, Parent).

marker_new(MarkerName, kinematic_chain(Identifier), MarkerObject, Parent) :-
  marker_new(MarkerName, kinematic_chain(Identifier,kinematic_chain(Identifier)), MarkerObject, Parent).

marker_new(MarkerName, kinematic_chain(Identifier,Term), MarkerObject, Parent) :-
  marker_primitive(arrow, MarkerName, Term, MarkerObject, Parent),
  marker_initialize_object(Identifier,MarkerObject),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker_has_visual( Link ),
    marker_child_name(Term, MarkerName, object_without_children(Link), ChildName),
    marker( ChildName, object_without_children(Link), _, MarkerObject )
  ))).

marker_new(MarkerName, stickman(Identifier), MarkerObject, Parent) :-
  StickManColor = [1.0,1.0,0.0,1.0],
  marker_primitive(sphere, MarkerName, stickman(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier,MarkerObject),
  marker_color(MarkerObject, StickManColor),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker_child_name(stickman(Identifier), MarkerName, object_without_children(Link), ChildName0),
    marker( ChildName0, object_without_children(Link), LinkMarker, MarkerObject ),
    marker_type(LinkMarker, sphere),
    marker_color(LinkMarker, StickManColor),
    
    marker_succeeding_links(Link, SucceedingLinks),
    forall(
      member( SucceedingLink,SucceedingLinks ), (
      marker_child_name(stickman(Identifier), MarkerName, cylinder_tf(Link,SucceedingLink), ChildName1),
      once( marker( ChildName1, cylinder_tf(Link,SucceedingLink), CylinderMarker, MarkerObject) ),
      marker_color(CylinderMarker, StickManColor)
    ))
  ))).

marker_new(MarkerName, black(Primitive,Term), MarkerObject, Parent) :-
  marker_primitive(Primitive, MarkerName, Term, MarkerObject, Parent),
  marker_color(MarkerObject, [0.0,0.0,0.0,1.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker_new(MarkerName, text(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(text_view_facing,text(Id)), MarkerObject, Parent).

marker_new(MarkerName, hud_text(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(hud_text,hud_text(Id)), MarkerObject, Parent),
  set_marker_translation(MarkerObject, [2.0,34.0,0]).

marker_new(MarkerName, hud_image(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(hud_image,hud_image(Id)), MarkerObject, Parent).

marker_new(MarkerName, sprite(Name), MarkerObject, Parent) :-
  marker_new(MarkerName, black(sprite,sprite(Name)), MarkerObject, Parent).

marker_new(MarkerName, sprite_text(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(sprite_text,sprite_text(Id)), MarkerObject, Parent),
  marker_scale(MarkerObject, [200.0,150.0,1.0]).

marker_new(MarkerName, sprite_scaled(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(sprite_scaled,sprite_scaled(Id)), MarkerObject, Parent),
  marker_scale(MarkerObject, [200.0,150.0,1.0]).

marker_new(MarkerName, background_image(Id), MarkerObject, Parent) :-
  marker_new(MarkerName, black(background_image,background_image(Id)), MarkerObject, Parent).

% TODO: this should be rather handled by one marker message with point array
marker_new_particles(_, _, _, Count, Count) :- !.
marker_new_particles(Parent, Obj, UrdfFormat, Count, Index) :-
  format(atom(Urdf), UrdfFormat, [Index]),
  % Create marker
  marker(Urdf, link(Urdf), MarkerObject, Parent),
  ( owl_has(Obj, knowrob:'pathToCadModel', literal(type(_,MeshPath))) -> (
    marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
    marker_scale(MarkerObject, [1.0,1.0,1.0]),
    marker_type(MarkerObject, mesh_resource),
    marker_mesh_resource(MarkerObject, MeshPath)
  )),
  % create next marker
  NextIndex is Index + 1,
  marker_new_particles(Parent, Obj, UrdfFormat, Count, NextIndex).

marker_child_name(ParentTerm, ParentName, ChildTerm, ChildName) :-
  term_to_atom(ParentTerm, N), N = ParentName,
  term_to_atom(ChildTerm, ChildName), !.
marker_child_name(_, ParentName, ChildTerm, ChildName) :-
  term_to_atom(ChildTerm, ChildAtom),
  atom_concat(ParentName, '_', Buf),
  atom_concat(Buf, ChildAtom, ChildName).
  
%% marker_hide(+Marker) is det.
%
% Hides marker if it was shown before.
%
% @param Marker A marker object / name / update term.
%
marker_hide(MarkerObject) :-
  jpl_is_object(MarkerObject),
  jpl_call(MarkerObject, 'hide', [], _).

marker_hide(Marker) :-
  marker(Marker, MarkerObject),
  marker_hide(MarkerObject).
  
%% marker_show(+Marker) is det.
%
% Shows marker again if it was hidden before.
%
% @param Marker A marker object / name / update term.
%
marker_show(MarkerObject) :-
  jpl_is_object(MarkerObject),
  jpl_call(MarkerObject, 'show', [], _).

marker_show(Marker) :-
  marker(Marker, MarkerObject),
  marker_show(MarkerObject).

%% marker_term
marker_term(X, experiment(X)) :-
  atom(X),
  rdfs_individual_of(X, knowrob:'Experiment'), !.
marker_term(X, MarkerTerm) :-
  atom(X),
  rdfs_individual_of(X, _), !,
  (owl_has(X, knowrob:markerType, literal(type(_,Type)))
  -> MarkerTerm =.. [Type,X]
  ;  MarkerTerm = object(X)).
marker_term(X, X).

%% marker_update is det.
%% marker_update(+T) is det.
%
% Updates all markers for the current local time or the time instant T.
%
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604).
%
marker_update :-
  current_time(T),
  marker_update(T).

marker_update(T) :-
  ( atom(T) ; number(T) ),
  time_term(T, T_value),
  forall(
    v_marker_object(Name, _, _, _),
    ignore(marker_update(Name, T_value))
  ), !.

%% marker_update(+MarkerTerm) is det.
%% marker_update(+MarkerTerm, +T) is det.
%% marker_update(+MarkerTerm, +MarkerObject, +T) is det.
%
% Updates the marker identified by MarkerTerm
% for the time instant T (or the current time).
%
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604) or an interval (e.g., interval(1396512604, 1396512608[, 0.5])).
%
marker_update(MarkerTerm) :-
  current_time(T),
  marker_update(MarkerTerm,T).

marker_update(MarkerObject, T) :-
  jpl_is_object(MarkerObject),
  marker(MarkerTerm, MarkerObject),
  marker_update(MarkerTerm, T).

marker_update(MarkerTerm, T) :-
  atom(T), time_term(T, T_Term),
  marker_update(MarkerTerm, T_Term).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  atom(T0), not(is_list(T0)), time_term(T0, T0_Term),
  atom(T1), not(is_list(T1)), time_term(T1, T1_Term),
  marker_update(MarkerTerm, interval(T0_Term,T1_Term,Interval)).

marker_update(MarkerTerm, T) :-
  number(T),
  marker_update(MarkerTerm, time(T,T)).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  number(T0), number(T1),
  marker_update(MarkerTerm, time(T0,interval(T0,T1,Interval))).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  is_list(T0), is_list(T1),
  findall(T_Term, (member(T,T0), time_term(T, T_Term)), T0_Terms),
  findall(T_Term, (member(T,T1), time_term(T, T_Term)), T1_Terms),
  % Use no caching if list of start/end times provided
  marker(MarkerTerm, MarkerObject),
  v_marker_object(_, Term, MarkerObject, _),
  marker_update(Term, MarkerObject, interval(T0_Terms,T1_Terms,Interval)).

marker_update(MarkerTerm, time(T,Arg)) :-
  number(T),
  T_float is float(T),
  marker(MarkerTerm, MarkerObject),
  marker_timestamp(MarkerObject, Last_T),
  % Only update once for given timestamp
  ( Last_T is T_float ; once((
    marker_timestamp(MarkerObject, T_float),
    v_marker_object(_, Term, MarkerObject, _),
    marker_update(Term, MarkerObject, Arg)
  ))), !.

marker_update(object_without_children(Identifier), MarkerObject, T) :-
  ignore(once((
    marker_transform_at_time(MarkerObject,Identifier,T,(Translation,Orientation)),
    marker_pose(MarkerObject,Translation,Orientation)
  ))).

marker_update(object(Identifier), MarkerObject, T) :-
  ignore(once((
    marker_transform_at_time(MarkerObject,Identifier,T,(Translation,Orientation)),
    marker_pose(MarkerObject,Translation,Orientation)
  ))),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), once((
    v_marker_object(_, ChildTerm, ChildObject, _),
    once(( marker_update(ChildTerm, ChildObject, T) ; 
           marker_hide(ChildObject) )),
    marker_timestamp(MarkerObject, T) % FIXME: shouldn't it be ChildObject here?
  ))).

marker_update(attached(Marker,AttachedTo), _, T) :-
  marker_update(AttachedTo,T),
  marker_pose(AttachedTo,Translation,Orientation),
  marker_pose(Marker,Translation,Orientation).

marker_update(link(Link), MarkerObject, T) :-
  marker_transform_at_time(MarkerObject,Link,T,(Translation,Orientation)),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(pointer(From,To), MarkerObject, T) :-
  marker_tf_frame(MarkerObject, From, FromResolved),
  marker_tf_frame(MarkerObject, To, ToResolved),
  mng_lookup_transform(FromResolved, ToResolved, T, Pose),
  matrix(Pose, Translation, Orientation),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(cylinder_tf(From,To), MarkerObject, T) :-
  marker_tf_frame(MarkerObject, From, FromResolved),
  marker_tf_frame(MarkerObject, To, ToResolved),
  map_frame_name(MapFrame),
  mng_lookup_transform(MapFrame, FromResolved, T, Pose0),
  mng_lookup_transform(MapFrame, ToResolved, T, Pose1),
  matrix(Pose0, [X0,Y0,Z0], _),
  matrix(Pose1, [X1,Y1,Z1], _),
  DX is X1-X0, DY is Y1-Y0, DZ is Z1-Z0,
  Distance is sqrt(DX*DX + DY*DY + DZ*DZ),
  QX is -DY, QY is DX, QZ is 0.0, QW is Distance + DZ,
  X is 0.5*(X0+X1), Y is 0.5*(Y0+Y1), Z is 0.5*(Z0+Z1),
  marker_scale(MarkerObject, [0.1,0.1,Distance]),
  marker_pose(MarkerObject, [X,Y,Z], [QX,QY,QZ,QW]),
  marker_show(MarkerObject), !.

marker_update(cylinder_tf(_,_), MarkerObject, _) :-
  marker_hide(MarkerObject).

marker_update(trajectory(Link), MarkerObject, interval(T0,T1)) :-
  marker_update(trajectory(Link), MarkerObject, interval(T0,T1,dt(0.5))).

marker_update(trajectory(Link), MarkerObject, interval(T0,T1,Dt)) :-
  object_trajectory(Link, [T0,T1], TrajectoryData, Dt),
  findall(P, member([P,_],TrajectoryData), TrajectoryPositions),
  marker_points(MarkerObject, TrajectoryPositions).

%marker_update(speech(Id), MarkerObject, T) :-
%  marker_object(_, MarkerObject, Parent),
%  marker_visualisation(MarkerVis),
%  not( Parent = MarkerVis ),
%  marker_translation(Parent, [X,Y,Z]),
%  marker_translation(MarkerObject, [X,Y,Z]).

marker_update(stickman(_), MarkerObject, T) :-
  marker_update_children(MarkerObject, T).
marker_update(agent(_), MarkerObject, T) :-
  marker_update_children(MarkerObject, T).
marker_update(experiment(_), MarkerObject, T) :-
  marker_update_children(MarkerObject, T).

% primitive types don't have update hooks
marker_update(Term, _, _) :-
  Term =.. [Type|_],
  marker_prop_type(Type,_).

marker_update_children(MarkerObject, T) :-
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), ignore((
    v_marker_object(_, ChildTerm, ChildObject, _),
    marker_update(ChildTerm, ChildObject, T),
    marker_timestamp(ChildObject, T)
  ))).

% TODO(daniel): also generate a speech marker message if for the timestamp
%    there is an active speech act for the updated marker.
%show_speech(Agent,Instant) :-
  %rdf_has(Ev, knowrob:'sender', Agent),
  %rdfs_individual_of(Ev, knowrob:'SpeechAct'),
  %occurs(Ev, Instant),
  %rdf_has(Ev, knowrob:'content', literal(type(_,Text))),
  %rdf_has(Ev, knowrob:'sender', Agent),
  %% find head
  %sub_component(pr2:'PR2Robot1', Head),
  %rdfs_individual_of(Head, knowrob:'Head-Vertebrate'),
  %rdf_has(Head, srdl2comp:urdfName, URDFVal),
  %strip_literal_type(URDFVal,URDF),
  %map_frame_name(MapFrame),
  %mng_lookup_transform(MapFrame, URDF, Instant, Transform),
  %matrix_translation(Transform, [X,Y,Z]),
  %Z_Offset is Z + 0.2,
  %marker(sprite_text('PR2_SPEECH'), MarkerObj),
  %marker_color(sprite_text('PR2_SPEECH'), [1.0,1.0,1.0]),
  %marker_translation(MarkerObj, [X,Y,Z_Offset]),
  %% Create styled html text
  %format(atom(TextHtml), '<div style="font-size: 18px; font-style: italic; font-family: Oswald,Arial,Helvetica,sans-serif; text-align: center;">~w</div>', [Text]),
  %marker_text(MarkerObj, TextHtml),
  %marker_scale(MarkerObj, [1.0,1.0,1.0]).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Highlighting marker
%

%% marker_highlight(+MarkerObject) is det.
%% marker_highlight(+MarkerObject, +Color) is det.
%
% Highlight markers identified by MarkerObject with given highlight color Color
% or the default color.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Color 3D/4D number list or HTML color atom (e.g., '#ff0000')
%
marker_highlight(MarkerObject) :-
  jpl_list_to_array([1.0,0.0,0.0,1.0], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

marker_highlight(MarkerObject, [R,G,B]) :-
  jpl_list_to_array([R,G,B,1.0], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

marker_highlight(MarkerObject, [R,G,B,A]) :-
  jpl_list_to_array([R,G,B,A], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

marker_highlight(MarkerTerm, ColorArg) :-
  v_marker_object(_, MarkerTerm, MarkerObject, _),
  marker_highlight(MarkerObject, ColorArg).

marker_highlight(MarkerName, ColorArg) :-
  v_marker_object(MarkerName, _, MarkerObject, _),
  marker_highlight(MarkerObject, ColorArg).

marker_highlight(interval(MarkerTerm,Start,Stop), ColorArg) :-
  v_marker_object(_, MarkerTerm, MarkerObject, _),
  marker_highlight(interval(MarkerObject,Start,Stop), ColorArg).

marker_highlight(interval(MarkerName,Start,Stop), ColorArg) :-
  v_marker_object(MarkerName, _, MarkerObject, _),
  marker_highlight(interval(MarkerObject,Start,Stop), ColorArg).

marker_highlight(interval(MarkerObject,Start,Stop), ColorArg) :-
  v_marker_object(_, _, MarkerObject, _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(
    member(ChildObject,Children), (
      marker_timestamp(ChildObject, MarkerTime),
      T is MarkerTime/1000000.0,
      (  time_between(T,Start,Stop)
      -> marker_highlight(ChildObject,ColorArg)
      ;  true
      )
  )).

marker_highlight(MarkerObject, ColorArg) :-
  v_marker_object(_, _, MarkerObject, _),
  once(( jpl_is_object(ColorArg) ; number(ColorArg) ; atom(ColorArg) )),
  jpl_call(MarkerObject, 'highlight', [ColorArg], _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(
    member(ChildObject,Children), 
    marker_highlight(ChildObject,ColorArg)
  ).

%% marker_highlight_remove(+MarkerObject) is det.
%
% Removes highlights from markers identified by MarkerObject.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
%
marker_highlight_remove(all) :-
  ignore( forall(
    v_marker_object(_, _, MarkerObject, _),
    ignore(marker_highlight_remove(MarkerObject))
  )), !.

marker_highlight_remove(MarkerTerm) :-
  v_marker_object(_, MarkerTerm, MarkerObject, _),
  marker_highlight_remove(MarkerObject), !.

marker_highlight_remove(MarkerName) :-
  v_marker_object(MarkerName, _, MarkerObject, _),
  marker_highlight_remove(MarkerObject), !.

marker_highlight_remove(MarkerObject) :-
  v_marker_object(_, _, MarkerObject, _),
  jpl_call(MarkerObject, 'hasHighlight', [], @(true)),
  jpl_call(MarkerObject, 'removeHighlight', [], _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), ignore((
    marker_highlight_remove(ChildObject)
  ))).

%% marker_highlight_toggle(+MarkerName) is det.
%
% Toggles marker highlighted state.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
%
marker_highlight_toggle(MarkerName) :-
  v_marker_object(MarkerName, _, MarkerObject, _),
  (  jpl_call(MarkerObject, 'hasHighlight', [], @(true))
  -> marker_highlight_remove(MarkerObject)
  ;  marker_highlight(MarkerObject)
  ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker queries
%

%% marker_queries(+MarkerName, ?Queries).
%
% Find all query representations that can be asked about this marker.
% These are queries described in an OWL ontology that can be asked about particular
% classes or individuals.
% Queries is a list of [atom Category,Title,Query].
%
marker_queries(MarkerName, Queries) :-
  v_marker_object(MarkerName, MarkerTerm, _, _),
  findall([Category,Title,Query], marker_query(MarkerName,MarkerTerm,Category,Title,Query), Queries).

marker_query(MarkerName, object(Individual), QueryGroup, QueryTitle, Query) :-
  marker_query_individual(MarkerName, individual(Individual), QueryGroup, QueryTitle, Query).

marker_query(MarkerName, object_without_children(Individual), QueryGroup, QueryTitle, Query) :-
  marker_query_individual(MarkerName, individual(Individual), QueryGroup, QueryTitle, Query).

marker_query(MarkerName, _, 'Marker Visualization', 'What is the name of this marker?', QueryAtom) :-
  atom_concat('Name = ', MarkerName, QueryAtom).

marker_query(MarkerName, _, 'Marker Visualization', 'Toggle marker highlight.', QueryAtom) :-
  QueryTerm=(marker_highlight_toggle(MarkerName), marker_publish),
  term_to_atom(QueryTerm,QueryAtom).

marker_query(MarkerName, _, 'Marker Visualization', 'Remove this marker.', QueryAtom) :-
  QueryTerm=(marker_remove(MarkerName), marker_publish),
  term_to_atom(QueryTerm,QueryAtom).

marker_query_individual(MarkerName, individual(Individual), QueryGroup, QueryTitle, Query) :-
  rdfs_individual_of(Individual, knowrob:'Designator'),
  rdf_split_url(Prefix, ObjName, Individual),
  atomic_list_concat([Prefix,'Object_',ObjName], Object),
  rdfs_individual_of(Object, knowrob:'SpatialThing-Localized'),
  marker_query_individual(MarkerName, individual(Object), QueryGroup, QueryTitle, Query).

marker_query_individual(_, individual(Individual), QueryGroup, QueryTitle, Query) :-
  rdf_has(QueryIndividual, knowrob:'queryAbout', Individual),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Individual, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).

marker_query_individual(_, individual(Individual), QueryGroup, QueryTitle, Query) :-
  rdfs_individual_of(Individual, IndividualClass),
  rdf_has(QueryIndividual, knowrob:'queryAbout', IndividualClass),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Individual, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).
  
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker properties
%

%% marker_properties(+Marker, Props) is det.
%
% Read or set marker properties.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Props List of properties (e.g., [type(T)|Tail])
%
marker_properties(Marker, [Key:Val|Args]) :-
  Prop=..[Key,Val],
  marker_property(Marker, Prop),
  marker_properties(Marker, Args).
marker_properties(_, []).

%% marker_property(+Marker, +Property) is det.
%
% Read or set marker property.
% @Property can be one of: type(Type), color(Color), scale(Scale), pose(Position,Orientation),
% text(Text)), timestamp(T).
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Property Property term.
%
marker_property(Marker, type(Type)) :-
  marker_type(Marker, Type).

marker_property(Marker, color(Color)) :-
  marker_color(Marker, Color).

marker_property(Marker, scale(Scale)) :-
  marker_scale(Marker, Scale).

marker_property(Marker, points(Points)) :-
  marker_points(Marker, Points).

marker_property(Marker, pose(Pose)) :-
  transform_data(Pose, (Position,Orientation)),
  marker_property(Marker, pose(Position,Orientation)), !.

marker_property(Marker, pose(Position,Orientation)) :-
  atom(Position), Position \= [], !,
  rdf_vector_prolog(Position, Position_v),
  marker_pose(Marker, Position_v, Orientation).

marker_property(Marker, pose(Position,Orientation)) :-
  atom(Orientation), Orientation \= [], !,
  rdf_vector_prolog(Orientation, Orientation_v),
  marker_pose(Marker, Position, Orientation_v).

marker_property(Marker, pose(Position,Orientation)) :-
  marker_pose(Marker, Position, Orientation).

marker_property(Marker, pose(mat(Mat))) :-
  marker_pose(Marker, mat(Mat)).

marker_property(Marker, mesh(Mesh)) :-
  marker_mesh_resource(Marker, Mesh).

marker_property(Marker, text(Text)) :-
  marker_text(Marker, Text).

marker_property(Marker, timestamp(T)) :-
  marker_timestamp(Marker, T).

%% marker_timestamp(+Marker, ?Value) is det.
%
% Read or set the timestamp property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_timestamp(Marker, T) :-
  marker_call(Marker,T,(get_marker_timestamp,set_marker_timestamp)).

%% marker_duration(+Marker, ?Value) is det.
%
% Read or set the duration property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_duration(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_duration,set_marker_duration)).

%% marker_type(+Marker, ?Value) is det.
%
% Read or set the type property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_type(Marker, Type) :-
  marker_call(Marker,Type,(get_marker_type,set_marker_type)).

%% marker_scale(+Marker, ?Value) is det.
%
% Read or set the scale property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_scale(Marker, Scale) :-
  marker_call(Marker,Scale,(get_marker_scale,set_marker_scale)).

%% marker_color(+Marker, ?Value) is det.
%
% Read or set the color property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_color(Marker, Color) :-
  marker_call(Marker,Color,(get_marker_color,set_marker_color)).

%% marker_alpha(+Marker, ?Value) is det.
%
% Read or set the alpha property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_alpha(Marker, Alpha) :-
  marker_call(Marker,Alpha,(get_marker_alpha,set_marker_alpha)).

%% marker_alpha(+MarkerList, +Alpha, +AlphaStep) is det.
%
% Apply alpha fade on markers in MarkerList.
%
% @param MarkerList List of MarkerObject instances or terms that identify the marker (e.g., trajectory('/base_link'))
% @param Alpha The alpha value for the first marker in MarkerList
% @param AlphaStep Delta alpha value
%
marker_alpha([Child|Rest], Alpha, AlphaStep) :-
  marker_alpha(Child, Alpha),
  Alpha_Next is Alpha + AlphaStep,
  marker_alpha(Rest, Alpha_Next, AlphaStep).
marker_alpha([], _, _).

%% marker_mesh_resource(+Marker, ?Value) is det.
%
% Read or set the mesh_resource property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_mesh_resource(Marker, Mesh) :-
  marker_call(Marker,Mesh,(get_marker_mesh,set_marker_mesh)).

%% marker_points(+Marker, ?Points) is det.
marker_points(Marker, Points) :-
  marker_call(Marker, Points, (get_marker_points,set_marker_points)).

%% marker_colors(+Marker, ?Colors) is det.
marker_colors(Marker, Colors) :-
  marker_call(Marker, Colors, (get_marker_colors,set_marker_colors)).

%% marker_pose(+Marker, ?Value) is det.
%
% Read or set the pose property of marker Marker.
% @Value must be unifyable with pose(Position,Orientation).
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_pose(Marker, pose(Position,Orientation)) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

marker_pose(Marker, mat(Matrix)) :-
  matrix(Matrix, Position, Orientation),
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

marker_pose(Marker, Position, Orientation) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

%% marker_orientation(+Marker, ?Value) is det.
%
% Read or set the orientation property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_orientation(Marker, Orientation) :-
  marker_call(Marker, Orientation, (get_marker_orientation,set_marker_orientation)).

%% marker_translation(+Marker, ?Value) is det.
%
% Read or set the translation property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_translation(Marker, Position) :-
  marker_call(Marker, Position, (get_marker_translation,set_marker_translation)).

%% marker_tf_prefix(+Marker, ?Value) is det.
%
% Read or set the TF prefix property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_tf_prefix(Marker, Prefix) :-
  marker_call(Marker, Prefix, (get_marker_tf_prefix,set_marker_tf_prefix)).

%% marker_text(+Marker, ?Value) is det.
%
% Read or set the text property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_text(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_text,set_marker_text)).

%% marker_has_visual(+Marker, ?Value) is det.
%
% Read or set the has_visual property of marker Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_has_visual(Marker, V) :-
  marker_call(Marker,V,(get_marker_has_visual,set_marker_has_visual)).

%% marker_call(+Marker, ?Value, +GetterSetter) is det.
%
% Read or set a property of marker(s) Marker.
% Marker can also be a list of markers.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
% @param Get The getter method
% @param Set The setter method
%
marker_call(Marker, Value, (Get,Set)) :-
  is_list(Marker),
  marker_call_seq(Marker, Value, (Get,Set)), !.

marker_call(recursive(Marker), Value, (Get,Set)) :-
  ( v_marker_object(Marker,_,MarkerObj,_) ; v_marker_object(_,Marker,MarkerObj,_) ),
  marker_call(recursive(MarkerObj), Value, (Get,Set)).

marker_call(Marker, Value, (Get,Set)) :-
  ( v_marker_object(Marker,_,MarkerObj,_) ; v_marker_object(_,Marker,MarkerObj,_) ),
  marker_call(MarkerObj, Value, (Get,Set)).

marker_call(recursive(Marker), Value, (Get,Set)) :-
  v_marker_object(_,_,Marker,_), ground(Value),
  marker_call([recursive(Marker)], Value, (Get,Set)).

marker_call(Marker, Value, (Get,Set)) :-
  v_marker_object(_,_,Marker,_), ground(Value),
  marker_call([Marker], Value, (Get,Set)).

marker_call(Marker, Value, (Get,_)) :-
  v_marker_object(_,_,Marker,_), not(ground(Value)),
  call(Get, Marker, Value).

marker_call_seq([recursive(Marker)|Rest], Value, (Get,Set)) :-
  v_marker_object(_,_,Marker,_),
  call(Set, Marker, Value),
  % recursively for all children
  forall(
    marker_child(Marker, MarkerChild), (
    marker_call_seq([recursive(MarkerChild)], Value, (Get,Set))
  )),
  % and for the list tail
  marker_call_seq(Rest, Value, (Get,Set)).

marker_call_seq([Marker|Rest], Value, (Get,Set)) :-
  v_marker_object(_,_,Marker,_),
  call(Set, Marker, Value),
  marker_call_seq(Rest, Value, (Get,Set)).

marker_call_seq([recursive(Marker)|Rest], Value, (Get,Set)) :-
  ( v_marker_object(Marker,_,MarkerObj,_) ; v_marker_object(_,Marker,MarkerObj,_) ),
  marker_call_seq([recursive(MarkerObj)|Rest], Value, (Get,Set)).

marker_call_seq([Marker|Rest], Value, (Get,Set)) :-
  ( v_marker_object(Marker,_,MarkerObj,_) ; v_marker_object(_,Marker,MarkerObj,_) ),
  marker_call_seq([MarkerObj|Rest], Value, (Get,Set)).

marker_call_seq([], _, _) :- true.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Getter/Setter for marker messages
%
  
get_marker_timestamp(MarkerObj, T) :-
  jpl_call(MarkerObj, 'getTimestamp', [], T).

set_marker_timestamp(MarkerObj, T) :-
  jpl_call(MarkerObj, 'setTimestamp', [T], _).
  
get_marker_duration(MarkerObj, T) :-
  jpl_call(MarkerObj, 'getDuration', [], T).

set_marker_duration(MarkerObj, T) :-
  jpl_call(MarkerObj, 'setDuration', [T], _).

get_marker_has_visual(MarkerObj, true) :-
  jpl_call(MarkerObj, 'getHasVisual', [], @(true)).

get_marker_has_visual(MarkerObj, false) :-
  jpl_call(MarkerObj, 'getHasVisual', [], @(false)).

set_marker_has_visual(MarkerObj, true) :-
  jpl_call(MarkerObj, 'setHasVisual', [@(true)], _).

set_marker_has_visual(MarkerObj, false) :-
  jpl_call(MarkerObj, 'setHasVisual', [@(false)], _).

get_marker_type(MarkerObj, Type) :-
  jpl_call(MarkerObj, 'getType', [], TypeId),
  marker_type(Type, TypeId).

set_marker_type(MarkerObj, Type) :-
  marker_prop_type(Type, TypeId),
  jpl_call(MarkerObj, 'setType', [TypeId], _).

get_marker_mesh(MarkerObj, Mesh) :-
  jpl_call(MarkerObj, 'getMeshResource', [], Mesh).

set_marker_mesh(MarkerObj, Mesh) :-
  jpl_call(MarkerObj, 'setMeshResource', [Mesh], _).

get_marker_text(MarkerObj, Text) :-
  jpl_call(MarkerObj, 'getText', [], Text).

set_marker_text(MarkerObj, Text) :-
  jpl_call(MarkerObj, 'setText', [Text], _).

get_marker_tf_prefix(MarkerObj, Prefix) :-
  jpl_call(MarkerObj, 'getTfPrefix', [], Prefix).

set_marker_tf_prefix(MarkerObj, Prefix) :-
  jpl_call(MarkerObj, 'setTfPrefix', [Prefix], _).

get_marker_scale(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getScale', [], ScaleArray),
  jpl_array_to_list(ScaleArray, [X,Y,Z]).

set_marker_scale(MarkerObj, [X,Y,Z]) :-
  atom(X),atom(Y),atom(Z),
  atom_number(X,X_N),
  atom_number(Y,Y_N),
  atom_number(Z,Z_N),
  set_marker_scale(MarkerObj, [X_N,Y_N,Z_N]).

set_marker_scale(MarkerObj, [X,Y,Z]) :-
  number(X),number(Y),number(Z),
  jpl_new(array(float), [X,Y,Z], ScaleArray),
  jpl_call(MarkerObj, 'setScale', [ScaleArray], _).

set_marker_scale(MarkerObj, Scale) :-
  number(Scale), set_marker_scale(MarkerObj, [Scale,Scale,Scale]).

get_marker_color(MarkerObj, [R,G,B,A]) :-
  jpl_call(MarkerObj, 'getColor', [], ColorArray),
  jpl_array_to_list(ColorArray,[R,G,B,A]).

get_marker_color(MarkerObj, [R,G,B]) :-
  jpl_call(MarkerObj, 'getColor', [], ColorArray),
  jpl_array_to_list(ColorArray,[R,G,B,_]).

set_marker_color(MarkerObj, [R,G,B]) :-
  atom(R),atom(G),atom(B),
  atom_number(R,R_N),
  atom_number(G,G_N),
  atom_number(B,B_N),
  set_marker_color(MarkerObj, [R_N,G_N,B_N,1.0]).

set_marker_color(MarkerObj, [R,G,B,A]) :-
  atom(R),atom(G),atom(B),atom(A),
  atom_number(R,R_N),
  atom_number(G,G_N),
  atom_number(B,B_N),
  atom_number(A,A_N),
  set_marker_color(MarkerObj, [R_N,G_N,B_N,A_N]).

set_marker_color(MarkerObj, [R,G,B]) :-
  number(R),number(G),number(B),
  jpl_new(array(float), [R,G,B,1.0], ColorArray),
  jpl_call(MarkerObj, 'setColor', [ColorArray], _).

set_marker_color(MarkerObj, [R,G,B,A]) :-
  number(R),number(G),number(B),number(A),
  jpl_new(array(float), [R,G,B,A], ColorArray),
  jpl_call(MarkerObj, 'setColor', [ColorArray], _).

set_marker_color(MarkerObj, Color) :-
  number(Color), set_marker_color(MarkerObj, [Color,Color,Color,1.0]).

get_marker_alpha(MarkerObj, A) :-
  jpl_call(MarkerObj, 'getAlpha', [], A).

set_marker_alpha(MarkerObj, A) :-
  atom(A), atom_number(A,A_N),
  set_marker_alpha(MarkerObj, A_N).

set_marker_alpha(MarkerObj, A) :-
  number(A),
  jpl_call(MarkerObj, 'setAlpha', [A], _).

get_marker_colors(MarkerObj, Colors) :-
  jpl_call(MarkerObj, 'getColors', [], ColorsArray),
  jpl_array_to_list(ColorsArray,Colors).

set_marker_colors(MarkerObj, Colors) :-
  jpl_new(array(double), Colors, ColorsArray),
  jpl_call(MarkerObj, 'setColors', [ColorsArray], _).

get_marker_points(MarkerObj, Points) :-
  jpl_call(MarkerObj, 'getPoints', [], PointsArray),
  jpl_array_to_list(PointsArray,Points).

set_marker_points(MarkerObj, Points) :-
  jpl_new(array(double), Points, PointsArray),
  jpl_call(MarkerObj, 'setPoints', [PointsArray], _).

get_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getTranslation', [], TranslationArray),
  jpl_array_to_list(TranslationArray,[X,Y,Z]).

get_marker_orientation(MarkerObj, [QX,QY,QZ,QW]) :-
  jpl_call(MarkerObj, 'getOrientation', [], OrientationArray),
  jpl_array_to_list(OrientationArray,[QX,QY,QZ,QW]).

get_marker_pose(MarkerObj, pose(Translation,Orientation)) :-
  get_marker_translation(MarkerObj, Translation),
  get_marker_orientation(MarkerObj, Orientation).

set_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_new(array(double), [X,Y,Z], Array),
  jpl_call(MarkerObj, 'setTranslation', [Array], _), !.
set_marker_translation(MarkerObj, Pos) :-
  atom(Pos), rdf_vector_prolog(Pos, Pos_v),
  set_marker_translation(MarkerObj, Pos_v).

set_marker_orientation(MarkerObj, [QX,QY,QZ,QW]) :-
  jpl_new(array(double), [QX,QY,QZ,QW], Array),
  jpl_call(MarkerObj, 'setOrientation', [Array], _), !.
set_marker_orientation(MarkerObj, O) :-
  atom(O), rdf_vector_prolog(O, O_v),
  set_marker_translation(MarkerObj, O_v).

set_marker_pose(MarkerObj, pose(Translation,Orientation)) :-
  set_marker_translation(MarkerObj, Translation),
  set_marker_orientation(MarkerObj, Orientation), !.
set_marker_pose(MarkerObj, pose(Pose)) :-
  atom(Pose),
  transform_data(Pose, pose(T,Q)),
  set_marker_pose(MarkerObj, pose(T,Q)).
