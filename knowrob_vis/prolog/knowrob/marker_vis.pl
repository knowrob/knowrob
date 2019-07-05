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
      marker/1,
      marker/3,
      marker_update/0,
      marker_update/1,
      marker_update/2,
      marker_update/3,
      marker_remove/1,
      marker_child/2,
      marker_term/2,
      marker_name/2,
      marker_publish/0,
      
      marker_properties/2,
      marker_type/2,
      marker_scale/2,
      marker_mesh_resource/2,
      marker_pose/2,
      marker_points/2,
      marker_translation/2,
      marker_text/2,
      marker_color/2,
      marker_colors/2,
      
      marker_primitive/4
    ]).
/** <module> Integration RViz marker visualization messages

  @author Daniel Beßler
  @license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('jpl')).
:- use_module(library('lists')). % for sum_list
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/transforms')).

:- rdf_meta marker(t),
            marker(t,?,?),
            marker_name(t,?),
            marker_update(t),
            marker_update(t,r),
            marker_term(t,?),
            marker_remove(t),
            marker_children(t,?),
            marker_type(t,?),
            marker_scale(t,?),
            marker_color(t,?),
            marker_mesh_resource(t,?),
            marker_pose(t,?),
            marker_translation(t,?),
            marker_text(t,?),
            marker_properties(t,?).

% define marker_new and marker_update as meta-predicate and allow the definitions
% to be in different source files
:- multifile marker_new/3,
             marker_update/3.

:- dynamic v_marker_object/3.

%% marker_publish is det.
%
% Publish all queued messages on the ROS /marker_visualisation topic
% and remove them from queue.
%
:- use_foreign_library('libmarker_vis.so').
             
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

marker_has_visual(Identifier) :-
  not(owl_individual_of(Identifier, srdl2comp:'UrdfJoint')),
  not(owl_individual_of(Identifier, knowrob:'RoomInAConstruction')),
  not(owl_individual_of(Identifier, knowrob:'SemanticEnvironmentMap')),
  not(rdf_has(Identifier, knowrob:'hasVisual', literal(type(_,false)))).

marker_children(Parent, Children) :-
  findall(Child, (
    (
      rdf_has(Child, knowrob:describedInMap, Parent);
      rdf_has(Parent, knowrob:'parts', Child);
      rdf_has(Parent, srdl2comp:'subComponent', Child);
      rdf_has(Parent, srdl2comp:'successorInKinematicChain', Child)
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

marker_tf_frame(_Marker, Identifier, TfFrame) :-
  marker_srdl_tf_frame(Identifier, UrdfName),
  atom_ensure_prefix(UrdfName, '/', TfFrame).

marker_lookup_transform(_, Identifier, T, (Translation,Orientation)) :-
  map_frame_name(MapFrame),
  object_pose_at_time(Identifier, T, [MapFrame, _, Translation, Orientation]), !.

marker_transform_at_time(Marker, Identifier, T, Pose) :-
  marker_lookup_transform(Marker, Identifier, T, Pose)
  -> true
  ;  marker_remove(Marker).

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
% Marker factory
%

%%
marker_create(MarkerName, MarkerTerm, Parent) :-
  atom(MarkerName),
  compound(MarkerTerm),
  assert( v_marker_object(MarkerName, MarkerTerm, Parent) ),
  marker_queue(MarkerName).

marker_create(MarkerTerm, Parent) :-
  compound(MarkerTerm),
  term_to_atom(MarkerTerm, MarkerAtom),
  marker_create(MarkerAtom, MarkerTerm, Parent).

%% marker_initialize_object(+Identifier,+Marker) is det.
%
% Read some default visualization properties from OWL individual.
%
% @param Identifier The name of the OWL individual
% @param Marker Marker name
%
marker_initialize_object(Identifier,Marker) :-
  marker_has_visual(Identifier),
  ( object_dimensions(Identifier, Depth, Width, Height) ->
    marker_scale(Marker, [Width, Depth, Height]) ;
    true
  ),
  ( object_color(Identifier, Color) ->
    marker_color(Marker, Color) ;
    true
  ),
  ( object_mesh_path(Identifier, Path) -> (
    marker_type(Marker, mesh_resource),
    marker_mesh_resource(Marker, Path),
    marker_color(Marker, [0.0,0.0,0.0,0.0]),
    ( rdf_has_prolog(Identifier, srdl2comp:'mesh_scale', Scale) ->
      marker_scale(Marker, Scale) ;
      marker_scale(Marker, [1.0,1.0,1.0])
    )) ;
    true
  ).

%% marker_primitive(+Type, +MarkerName, +MarkerTerm, +Parent) is det.
%
% @param Type The shape primitive type (defined by marker_prop_type/2)
% @param Marker A marker name
% @param MarkerTerm A marker term
% @param Parent parent marker name
%
marker_primitive(Type, Marker, MarkerTerm, Parent) :-
  marker_create(Marker, MarkerTerm, Parent),
  marker_type(Marker, Type),
  marker_color(Marker, [0.6,0.6,0.6,1.0]),
  marker_scale(Marker, [0.05,0.05,0.05]).

%% marker_remove(+MarkerTerm) is det.
%
% Removes a marker.
% marker_remove(all) removes all known markers.
% marker_remove(trajectories) removes all known trajectory markers.
% marker_remove(MarkerTerm) removes all markers that can be unified with
% the compound term MarkerTerm.
% 
% @param MarkerTerm a marker name or term.
%
marker_remove(all) :-
  forall(
    v_marker_object(Marker, _, _),
    marker_remove(Marker)
  ), !.

marker_remove(trajectories) :-
  forall(
    v_marker_object(Marker, trajectory(_), _),
    marker_remove(Marker)
  ), !.

marker_remove(MarkerTerm) :-
  compound(MarkerTerm),
  forall(
    v_marker_object(Marker, MarkerTerm, _),
    marker_remove(Marker)
  ).

marker_remove(Marker) :-
  v_marker_object(Marker, _, _),
  marker_erase(Marker).

%% marker_child(+MarkerTerm,?MarkerChildTerm) is nondet.
%
% Query the children markers of the marker identified by MarkerTerm.
% 
% @param MarkerTerm A marker term
% @param MarkerChildTerm A marker term
%
marker_child(MarkerTerm, MarkerChildTerm) :-
  marker(_, MarkerTerm, Child),
  marker(Child, MarkerChildTerm, _).

%% marker(?MarkerTerm) is det.
%
% Selects marker that corresponds to MarkerTerm.
% Possibly creates a new marker if MarkerTerm is unknown.
% 
% @param MarkerTerm A marker term
%
marker(MarkerTerm) :-
  compound(MarkerTerm),!,
  term_to_atom(MarkerTerm,Marker),
  marker(Marker, MarkerTerm, _).

marker(Marker) :-
  atom(Marker),!,
  term_to_atom(MarkerTerm,Marker),
  marker(Marker,MarkerTerm,_).

marker(Marker) :-
  var(Marker),!,
  v_marker_object(Marker,_,_).

%% marker(?Marker,?MarkerTerm,+Parent) is det.
%
% Selects MarkerObject instance that corresponds to MarkerTerm.
% Possibly creates a new MarkerObject if MarkerTerm is unknown.
% 
% @param Marker a marker name
% @param MarkerTerm a marker term
% @param Parent a marker name
%
marker(Marker, MarkerTerm, Parent) :-
  ( v_marker_object(Marker, MarkerTerm, Parent) *-> true ; (
    marker_new(Marker, MarkerTerm, Parent)
  )).

marker_new(Marker, cube(Name), Parent) :-
  marker_primitive(cube, Marker, cube(Name), Parent).

marker_new(Marker, sphere(Name), Parent) :-
  marker_primitive(sphere, Marker, sphere(Name), Parent).

marker_new(Marker, arrow(Name), Parent) :-
  marker_primitive(arrow, Marker, arrow(Name), Parent).

marker_new(Marker, cylinder(Name), Parent) :-
  marker_primitive(cylinder, Marker, cylinder(Name), Parent).

marker_new(Marker, cylinder_tf(From,To), Parent) :-
  marker_primitive(cylinder, Marker, cylinder_tf(From,To), Parent).

marker_new(Marker, link(Link), Parent) :-
  marker_primitive(arrow, Marker, link(Link), Parent).

marker_new(Marker, trajectory(Link), Parent) :-
  marker_primitive(sphere_list, Marker, trajectory(Link), Parent),
  marker_color(Marker, [1.0,1.0,0.0,1.0]).

marker_new(Marker, pointer(From,To), Parent) :-
  marker_primitive(arrow, Marker, pointer(From,To), Parent).

marker_new(Marker, mesh(Name), Parent) :-
  marker_create(Marker, mesh(Name), Parent),
  marker_type(Marker, mesh_resource),
  marker_color(Marker, [0.0,0.0,0.0,0.0]),
  marker_scale(Marker, [1.0,1.0,1.0]).

marker_new(Marker, mesh(Name,MeshFile), Parent) :-
  marker_new(Marker, mesh(Name), Parent),
  marker_mesh_resource(Marker, MeshFile).

marker_new(Marker, object_without_children(Identifier), Parent) :-
  marker_primitive(cube, Marker, object_without_children(Identifier), Parent),
  marker_initialize_object(Identifier, Marker).

marker_new(Marker, object(Identifier), Parent) :-
  marker_primitive(cube, Marker, object(Identifier), Parent),
  marker_initialize_object(Identifier, Marker),
  marker_children(Identifier,Children),
  forall(
    member( Child,Children ), ignore((
      marker_term(Child, ChildTerm),
      marker_child_name(object(Identifier), Marker, ChildTerm, ChildName),
      marker(ChildName, ChildTerm, Marker)
    ))
  ).

marker_new(Marker, agent(Identifier), Parent) :-
  marker_new(Marker, kinematic_chain(Identifier,agent(Identifier)), Parent).

marker_new(Marker, kinematic_chain(Identifier), Parent) :-
  marker_new(Marker, kinematic_chain(Identifier,kinematic_chain(Identifier)), Parent).

marker_new(Marker, kinematic_chain(Identifier,Term), Parent) :-
  marker_primitive(arrow, Marker, Term, Parent),
  marker_initialize_object(Identifier,Marker),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker_child_name(Term, Marker, object_without_children(Link), ChildName),
    marker( ChildName, object_without_children(Link), Marker )
  ))).

marker_new(Marker, stickman(Identifier), Parent) :-
  StickManColor = [1.0,1.0,0.0,1.0],
  marker_primitive(sphere, Marker, stickman(Identifier), Parent),
  marker_initialize_object(Identifier,Marker),
  marker_color(Marker, StickManColor),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker_child_name(stickman(Identifier), Marker, object_without_children(Link), ChildName0),
    marker( ChildName0, object_without_children(Link), Marker ),
    marker_type(ChildName0, sphere),
    marker_color(ChildName0, StickManColor),
    
    marker_succeeding_links(Link, SucceedingLinks),
    forall(
      member( SucceedingLink,SucceedingLinks ), (
      marker_child_name(stickman(Identifier), Marker, cylinder_tf(Link,SucceedingLink), ChildName1),
      marker( ChildName1, cylinder_tf(Link,SucceedingLink), Marker),
      marker_color(ChildName1, StickManColor)
    ))
  ))).

marker_new(Marker, black(Primitive,Term), Parent) :-
  marker_primitive(Primitive, Marker, Term, Parent),
  marker_color(Marker, [0.0,0.0,0.0,1.0]),
  marker_scale(Marker, [1.0,1.0,1.0]).

marker_new(Marker, text(Id), Parent) :-
  marker_new(Marker, black(text_view_facing,text(Id)), Parent).

marker_new(Marker, hud_text(Id), Parent) :-
  marker_new(Marker, black(hud_text,hud_text(Id)), Parent),
  set_marker_translation(Marker, [2.0,34.0,0]).

marker_new(Marker, hud_image(Id), Parent) :-
  marker_new(Marker, black(hud_image,hud_image(Id)), Parent).

marker_new(Marker, sprite(Name), Parent) :-
  marker_new(Marker, black(sprite,sprite(Name)), Parent).

marker_new(Marker, sprite_text(Id), Parent) :-
  marker_new(Marker, black(sprite_text,sprite_text(Id)), Parent),
  marker_scale(Marker, [200.0,150.0,1.0]).

marker_new(Marker, sprite_scaled(Id), Parent) :-
  marker_new(Marker, black(sprite_scaled,sprite_scaled(Id)), Parent),
  marker_scale(Marker, [200.0,150.0,1.0]).

marker_new(Marker, background_image(Id), Parent) :-
  marker_new(Marker, black(background_image,background_image(Id)), Parent).

marker_child_name(ParentTerm, ParentName, ChildTerm, ChildName) :-
  term_to_atom(ParentTerm, N), N = ParentName,
  term_to_atom(ChildTerm, ChildName), !.
marker_child_name(_, ParentName, ChildTerm, ChildName) :-
  term_to_atom(ChildTerm, ChildAtom),
  atom_concat(ParentName, '_', Buf),
  atom_concat(Buf, ChildAtom, ChildName).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Updating marker for given timepoint/timerange
%

show :- marker_update.

show(X) :-
  is_list(X), !,
  show_next,
  forall( member(MarkerDescr, X), (
    T =.. [show|MarkerDescr], call(T)
  )).

show(X) :-
  get_timepoint(Instant),
  show(X,Instant,[]).

show(X, Properties) :-
  is_list(Properties),
  get_timepoint(Instant),
  show(X,Instant,Properties).

show(X, Instant) :-
  show(X, Instant, []).

show(X, Instant, Properties) :-
  is_list(Properties),
  marker_term(X,MarkerTerm),
  v_marker_object(Marker,MarkerTerm,_),
  marker_update(Marker,Instant),!,
  marker_properties(Marker, Properties).

show_next :-
  marker_remove(trajectories).

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
    v_marker_object(Name, _, _),
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

marker_update(MarkerTerm, T) :-
  atom(T), !, time_term(T, T_Term),
  marker_update(MarkerTerm, T_Term).

marker_update(MarkerTerm, T) :-
  number(T), !,
  marker_update(MarkerTerm, time(T,T)).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  number(T0), number(T1), !,
  marker_update(MarkerTerm, time(T0,interval(T0,T1,Interval))).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  atom(T0), not(is_list(T0)), time_term(T0, T0_Term),
  atom(T1), not(is_list(T1)), time_term(T1, T1_Term), !,
  marker_update(MarkerTerm, interval(T0_Term,T1_Term,Interval)).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  is_list(T0), is_list(T1), !,
  findall(T_Term, (member(T,T0), time_term(T, T_Term)), T0_Terms),
  findall(T_Term, (member(T,T1), time_term(T, T_Term)), T1_Terms),
  marker(Marker,MarkerTerm,_),
  marker_update(Marker, MarkerTerm, interval(T0_Terms,T1_Terms,Interval)).

marker_update(Marker, MarkerTerm, time(T,Arg)) :-
  number(T),
  T_float is float(T),
  marker_timestamp(Marker, Last_T),
  % Only update once for given timestamp
  ( Last_T is T_float ; once((
    marker_timestamp(Marker, T_float),
    marker_update(Marker, MarkerTerm, Arg)
  ))), !.

marker_update(Marker, object_without_children(Identifier), T) :-
  ignore(once((
    marker_transform_at_time(Marker,Identifier,T,(Translation,Orientation)),
    marker_pose(Marker,Translation,Orientation)
  ))).

marker_update(Marker, object(Identifier), T) :-
  ignore(once((
    marker_transform_at_time(Marker,Identifier,T,(Translation,Orientation)),
    ground((Translation,Orientation)),
    marker_pose(Marker,Translation,Orientation)
  ))),
  marker_timestamp(Marker, T),
  forall(v_marker_object(Child, ChildTerm, Marker),
    once((
      once(( marker_update(Child, ChildTerm, T) ; 
             marker_remove(Child) )),
      marker_timestamp(Child, T)
    ))
  ).

marker_update(Marker, link(Link), T) :-
  marker_transform_at_time(Marker,Link,T,(Translation,Orientation)),
  marker_pose(Marker,Translation,Orientation).

marker_update(Marker, pointer(From,To), T) :-
  marker_tf_frame(Marker, From, FromResolved),
  marker_tf_frame(Marker, To, ToResolved),
  mng_lookup_transform(FromResolved, ToResolved, pose(Translation, Orientation), T),
  marker_pose(Marker,Translation,Orientation).

marker_update(Marker, cylinder_tf(From,To), T) :-
  marker_tf_frame(Marker, From, FromResolved),
  marker_tf_frame(Marker, To, ToResolved),
  map_frame_name(MapFrame),
  mng_lookup_transform(MapFrame, FromResolved, pose([X0,Y0,Z0], _), T),
  mng_lookup_transform(MapFrame, ToResolved, pose([X1,Y1,Z1], _), T),
  DX is X1-X0, DY is Y1-Y0, DZ is Z1-Z0,
  Distance is sqrt(DX*DX + DY*DY + DZ*DZ),
  QX is -DY, QY is DX, QZ is 0.0, QW is Distance + DZ,
  X is 0.5*(X0+X1), Y is 0.5*(Y0+Y1), Z is 0.5*(Z0+Z1),
  marker_scale(Marker, [0.1,0.1,Distance]),
  marker_pose(Marker, [X,Y,Z], [QX,QY,QZ,QW]), !.

marker_update(Marker, cylinder_tf(_,_), _) :-
  marker_remove(Marker).

marker_update(Marker, trajectory(Link), interval(T0,T1)) :-
  marker_update(Marker, trajectory(Link), interval(T0,T1,dt(0.5))).

marker_update(Marker, trajectory(Link), interval(T0,T1,Dt)) :-
  object_trajectory(Link, [T0,T1], Dt, TrajectoryData),
  findall(P, member([_,_,P,_],TrajectoryData), TrajectoryPositions),
  marker_points(Marker, TrajectoryPositions).

marker_update(Marker, stickman(_), T) :-
  marker_updatechildren(Marker, T).
marker_update(Marker, agent(_), T) :-
  marker_updatechildren(Marker, T).

% primitive types don't have update hooks
marker_update(Marker, MarkerTerm, _) :-
  v_marker_object(Marker,MarkerTerm,_),
  MarkerTerm =.. [Type|_],
  marker_prop_type(Type,_).

marker_updatechildren(Marker, T) :-
  forall(v_marker_object(Child,ChildTerm,Marker),
    ignore((
      marker_update(Child,ChildTerm,T),
      marker_timestamp(Child,T)
    ))
  ).
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker properties
%

%% marker_properties(+Marker, Props) is det.
%
% Read or set marker properties.
%
% @param Marker the name of a marker
% @param Props List of properties (e.g., [type(T)|Tail])
%
marker_properties(Marker, [Key:Val|Args]) :-
  Prop=..[Key,Val],
  ignore(marker_property(Marker, Prop)),
  marker_properties(Marker, Args).
marker_properties(_, []).

%% marker_property(+Marker, +Property) is det.
%
% Read or set marker property.
% @Property can be one of: type(Type), color(Color), scale(Scale), pose(Position,Orientation),
% text(Text)), timestamp(T).
%
% @param Marker the name of a marker
% @param Property Property term.
%
marker_property(Marker, type(Type))     :- marker_type(Marker, Type).
marker_property(Marker, color(Color))   :- marker_color(Marker, Color).
marker_property(Marker, scale(Scale))   :- marker_scale(Marker, Scale).
marker_property(Marker, points(Points)) :- marker_points(Marker, Points).
marker_property(Marker, mesh(Mesh))     :- marker_mesh_resource(Marker, Mesh).
marker_property(Marker, text(Text))     :- marker_text(Marker, Text).
marker_property(Marker, timestamp(T))   :- marker_timestamp(Marker, T).
marker_property(Marker, pose(Pose))     :- marker_property(Marker, Pose).
marker_property(Marker, pose(P,Q))      :- marker_pose(Marker, P, Q).

%% marker_timestamp(+Marker, ?Value) is det.
%
% Read or set the timestamp property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_timestamp(Marker, T) :-
  marker_call(Marker,T,(get_marker_timestamp,set_marker_timestamp)).

%% marker_duration(+Marker, ?Value) is det.
%
% Read or set the duration property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_duration(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_duration,set_marker_duration)).

%% marker_type(+Marker, ?Value) is det.
%
% Read or set the type property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_type(Marker, Type) :-
  marker_prop_type(Type,X),!,
  marker_type(Marker, X).

marker_type(Marker, Type) :-
  marker_call(Marker,Type,(get_marker_type,set_marker_type)).

%% marker_scale(+Marker, ?Value) is det.
%
% Read or set the scale property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_scale(Marker, Scale) :-
  marker_call(Marker,Scale,(get_marker_scale,set_marker_scale)).

%% marker_color(+Marker, ?Value) is det.
%
% Read or set the color property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_color(Marker, Color) :-
  marker_call(Marker,Color,(get_marker_color,set_marker_color)).

%% marker_alpha(+Marker, ?Value) is det.
%
% Read or set the alpha property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_alpha(Marker, Alpha) :-
  marker_call(Marker,Alpha,(get_marker_alpha,set_marker_alpha)).

%% marker_alpha(+MarkerList, +Alpha, +AlphaStep) is det.
%
% Apply alpha fade on markers in MarkerList.
%
% @param MarkerList List of marker names
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
% @param Marker A marker name
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
% @param Marker A marker name
% @param Value The property value
%
marker_pose(Marker, Pose) :-
  marker_call(Marker, Pose, (get_marker_pose,set_marker_pose)).

marker_pose(Marker, Position, Orientation) :-
  marker_translation(Marker, Position),
  marker_orientation(Marker, Orientation).

%% marker_orientation(+Marker, ?Value) is det.
%
% Read or set the orientation property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_orientation(Marker, Orientation) :-
  marker_call(Marker, Orientation, (get_marker_orientation,set_marker_orientation)).

%% marker_translation(+Marker, ?Value) is det.
%
% Read or set the translation property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_translation(Marker, Position) :-
  marker_call(Marker, Position, (get_marker_translation,set_marker_translation)).

%% marker_text(+Marker, ?Value) is det.
%
% Read or set the text property of marker Marker.
%
% @param Marker A marker name
% @param Value The property value
%
marker_text(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_text,set_marker_text)).

marker_name(MarkerTerm, Name) :-
  v_marker_object(Marker,MarkerTerm,_),!,
  marker_name(Marker, Name).

marker_name(Marker, Name) :-
  v_marker_object(Marker,_,_),
  get_marker_id(Marker,Id),
  get_marker_ns(Marker,NS),
  atom_number(Id_atom,Id),
  atom_concat(NS,Id_atom,Name).

%% marker_call(+Marker, ?Value, +GetterSetter) is det.
%
% Read or set a property of marker(s) Marker.
% Marker can also be a list of markers.
%
% @param Marker A marker name or term
% @param Value The property value
% @param Get The getter method
% @param Set The setter method
%
marker_call(Marker, Value, (Get,Set)) :-
  is_list(Marker),
  marker_call_seq(Marker, Value, (Get,Set)), !.

marker_call(X, Value, (Get,Set)) :-
  marker_call_seq([X], Value, (Get,Set)).

marker_call_seq([recursive(Marker)|Rest], Value, (Get,Set)) :-
  v_marker_object(Marker,_,_),
  call(Set, Marker, Value),
  % recursively for all children
  forall(
    marker_child(Marker, MarkerChild), (
    marker_call_seq([recursive(MarkerChild)], Value, (Get,Set))
  )),
  % and for the list tail
  marker_call_seq(Rest, Value, (Get,Set)).

marker_call_seq([Marker|Rest], Value, (Get,Set)) :-
  v_marker_object(Marker,_,_),
  call(Set, Marker, Value),
  marker_call_seq(Rest, Value, (Get,Set)).

marker_call_seq([recursive(MarkerTerm)|Rest], Value, (Get,Set)) :-
  v_marker_object(Marker,MarkerTerm,_),
  marker_call_seq([recursive(Marker)|Rest], Value, (Get,Set)).

marker_call_seq([MarkerTerm|Rest], Value, (Get,Set)) :-
  v_marker_object(Marker,MarkerTerm,_),
  marker_call_seq([Marker|Rest], Value, (Get,Set)).

marker_call_seq([], _, _) :- true.
