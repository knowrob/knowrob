

:- module(knowrob_marker,
    [
      marker_publish/0,
      marker_republish/0,
      
      marker/2,
      marker_update/0,
      marker_update/1,
      marker_update/2,
      marker_remove/1,
      marker_child/2,
      
      marker_properties/2,
      marker_type/2,
      marker_scale/2,
      marker_mesh_resource/2,
      marker_pose/2,
      marker_translation/2,
      marker_text/2,
      marker_color/2,
      marker_has_visual/2,
      
      marker_highlight/1,
      marker_highlight/2,
      marker_highlight_remove/1,
      
      marker_distance/3,
      marker_trajectory_length/2
    ]).
    
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).

:- rdf_meta marker(t,?),
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
            marker_mesh_resource(t,?),
            marker_pose(t,?),
            marker_translation(t,?),
            marker_text(t,?),
            marker_properties(t,?),
            marker_trajectory_length(t,?),
            marker_distance(t,t,?).


marker_has_visual(Identifier) :-
  not(owl_individual_of(Identifier, srdl2comp:'UrdfJoint')),
  not(owl_individual_of(Identifier, knowrob:'Agent-Generic')),
  not(owl_individual_of(Identifier, knowrob:'RoomInAConstruction')),
  not(owl_individual_of(Identifier, knowrob:'SemanticEnvironmentMap')),
  not(rdf_has(Identifier, knowrob:'hasVisual', literal(type(_,false)))).

marker_children(Parent, Children) :-
  findall(Child, (
    rdf_reachable(Part, knowrob:describedInMap, Parent),
    (
      rdf_reachable(Part, knowrob:'properPhysicalParts', Child);
      rdf_reachable(Part, srdl2comp:'subComponent', Child);
      rdf_reachable(Part, srdl2comp:'successorInKinematicChain', Child)
    ),
    not(Child = Parent)
  ), ChildrenList),
  list_to_set(ChildrenList, Children).

marker_links(Identifier, Links) :-
  findall(Link, (
    owl_has(Identifier, srdl2comp:'subComponent', Component),
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

marker_tf_frame(Identifier, UrdfName) :-
  rdf_has(Identifier, srdl2comp:'urdfName', literal(Tf)),
  atomic_list_concat(['/', Tf], UrdfName), !.
marker_tf_frame(UrdfName, UrdfName).

marker_lookup_transform(Identifier, T, (Translation,Orientation)) :-
  object_pose_at_time(Identifier, T, Translation, Orientation).

marker_lookup_transform(Identifier, T, (Translation,Orientation)) :-
  marker_lookup_transform(Identifier, '/map', T, (Translation,Orientation)).

marker_lookup_transform(Identifier, TargetFrame, T, (Translation,Orientation)) :-
  marker_tf_frame(Identifier, TfFrame),
  not( atom_prefix(TfFrame, 'http') ),
  mng_lookup_transform(TargetFrame, TfFrame, T, Pose),
  matrix_rotation(Pose, Orientation),
  matrix_translation(Pose, Translation).


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

%% marker_create(+MarkerTerm, -MarkerObject, +Parent) is det.
%
% Creates a new MarkerObject instance that is identified by @MarkerTerm.
%
% @param MarkerTerm A term that identifies the new marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject object created
% @param Parent The parent Java object (i.e., MarkerPublisher or MarkerObject instance)
%
marker_create(MarkerTerm, MarkerObject, Parent) :-
  compound(MarkerTerm),
  % Ensures that there are no unbound variables in compound term
  numbervars(MarkerTerm,0,0),
  term_to_atom(MarkerTerm, MarkerAtom),
  jpl_call(Parent, 'createMarker', [MarkerAtom], MarkerObject),
  assert( v_marker_object(MarkerTerm, MarkerObject) ).

%% marker_create(+Prefix, +Parent, +Count, -MarkerObjects) is det.
%
% Creates @Count new MarkerObject instances identified by indexed @Prefix.
%
% @param Prefix The marker name prefix
% @param Parent The parent Java object (i.e., MarkerPublisher or MarkerObject instance)
% @param Count The number of new MarkerObject instances
% @param MarkerObjects List of new MarkerObject instances
%
marker_create(Prefix, Parent, Count, [X|Xs]) :-
  Count > 0,
  atom_concat(Prefix, Count, MarkerName),
  jpl_call(Parent, 'createMarker', [MarkerName], X),
  assert( v_marker_object(MarkerName, X) ),
  Count_next is Count - 1,
  marker_create(Prefix, Parent, Count_next, Xs).
marker_create(_, _, 0, []).

%% marker_object(?Term,?Object) is nondet.
%
% Maps marker identification term to MarkerObject instance.
%
% @param Term The marker identification term
% @param Object The MarkerObject instance
%
marker_object(Term,Object) :-
  current_predicate(v_marker_object,_),
  v_marker_object(Term,Object).

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
    get_model_path(Identifier, Path),
    marker_type(MarkerObject, mesh_resource),
    marker_mesh_resource(MarkerObject, Path),
    marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
    marker_scale(MarkerObject, [1.0,1.0,1.0])
  )),
  ignore((
    object_dimensions(Identifier, X, Y, Z),
    marker_scale(MarkerObject, [X, Y, Z])
  )),
  ignore((
    object_color(Identifier, Color),
    parse_vector(Color, ColorList),
    marker_color(MarkerObject, ColorList)
  )).

%% marker_primitive(+Type, +MarkerTerm, -MarkerObject, +Parent) is det.
%
% Creates MarkerObject instance with primitive geometry shape (e.g., cube, sphere, ...)
% and some default properties.
%
% @param Type The shape primitive type (defined by marker_prop_type/2)
% @param MarkerTerm A term that identifies the new marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject object created
% @param Parent The parent Java object (i.e., MarkerPublisher or MarkerObject instance)
%
marker_primitive(Type, MarkerTerm, MarkerObject, Parent) :-
  marker_create(MarkerTerm, MarkerObject, Parent),
  marker_type(MarkerObject, Type),
  marker_color(MarkerObject, [0.6,0.6,0.6,1.0]),
  marker_scale(MarkerObject, [0.05,0.05,0.05]).

%% marker_remove(all) is det.
%% marker_remove(trajectories) is det.
%% marker_remove(Term) is det.
%
% Removes MarkerObject instances that match given term.
% marker_remove(all) removes all known markers.
% marker_remove(trajectories) removes all known trajectory markers.
% marker_remove(Term) removes all markers that can be unified with
% the compund term @Term.
% 
% @param Term Marker identification term.
%
marker_remove(all) :-
  marker_object(Term, _),
  marker_remove(Term).

marker_remove(trajectories) :-
  forall(
    marker_object(trajectory(Link), _),
    marker_remove(trajectory(Link))
  ).

marker_remove(Term) :-
  compound(Term),
  v_marker_object(Term, MarkerObject),
  marker_remove(MarkerObject).

marker_remove(MarkerObject) :-
  jpl_is_object(MarkerObject),
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'eraseMarker', [MarkerObject], _),
  ignore(( % using ignore because trajectory markers not asserted
    v_marker_object(Term, MarkerObject),
    retract( v_marker_object(Term, _) )
  )),
  
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall( member(ChildObject,Children), marker_remove(ChildObject) ).

%% marker(+MarkerTerm,?MarkerChildTerm) is nondet.
%
% Query the children markers of the marker identified by @MarkerTerm.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerChildTerm A term that identifies the marker child
marker_child(MarkerTerm, MarkerChildTerm) :-
  marker(MarkerTerm, MarkerObject),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  member(ChildObject,Children),
  v_marker_object(MarkerChildTerm, ChildObject).

%% marker(?MarkerTerm,?MarkerObject) is det.
%
% Selects @MarkerObject instance that corresponds to @MarkerTerm.
% Possibly creates a new @MarkerObject if @MarkerTerm is unknown.
% The MarkerPublisher instance is automatically selected as parent
% if the marker object is newly created with this call.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
%
marker(MarkerTerm, MarkerObject) :-
  marker_visualisation(MarkerVis),
  marker(MarkerTerm, MarkerObject, MarkerVis).

%% marker(?MarkerTerm,?MarkerObject,+Parent) is det.
%
% Selects @MarkerObject instance that corresponds to @MarkerTerm.
% Possibly creates a new @MarkerObject if @MarkerTerm is unknown.
% 
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
% @param Parent The parent of MarkerObject instance
%
marker(MarkerTerm, MarkerObject, _) :-
  % Cache lookup, ! avoids the solution where a new object is created
  marker_object(MarkerTerm, MarkerObject), !.

marker(cube(Name), MarkerObject, Parent) :-
  marker_primitive(cube, cube(Name), MarkerObject, Parent).

marker(sphere(Name), MarkerObject, Parent) :-
  marker_primitive(sphere, sphere(Name), MarkerObject, Parent).

marker(arrow(Name), MarkerObject, Parent) :-
  marker_primitive(arrow, arrow(Name), MarkerObject, Parent).

marker(cylinder(Name), MarkerObject, Parent) :-
  marker_primitive(cylinder, cylinder(Name), MarkerObject, Parent).

marker(cylinder_tf(From,To), MarkerObject, Parent) :-
  marker_primitive(cylinder, cylinder_tf(From,To), MarkerObject, Parent).

marker(link(Link), MarkerObject, Parent) :-
  marker_primitive(arrow, link(Link), MarkerObject, Parent).

marker(trajectory(Link), MarkerObject, Parent) :-
  marker_primitive(arrow, trajectory(Link), MarkerObject, Parent),
  marker_color(MarkerObject, [1.0,1.0,0.0,1.0]),
  marker_has_visual(MarkerObject, false).

marker(trail(Link), MarkerObject, Parent) :-
  marker_primitive(sphere, trajectory(Link), MarkerObject, Parent),
  marker_color(MarkerObject, [1.0,1.0,0.0,1.0]),
  marker_has_visual(MarkerObject, false).

marker(average_trajectory(Link), MarkerObject, Parent) :-
  marker_primitive(arrow, average_trajectory(Link), MarkerObject, Parent),
  marker_has_visual(MarkerObject, false).

marker(pointer(From,To), MarkerObject, Parent) :-
  marker_primitive(arrow, pointer(From,To), MarkerObject, Parent).

marker(mesh(Name), MarkerObject, Parent) :-
  marker_create(mesh(Name), MarkerObject, Parent),
  marker_type(MarkerObject, mesh_resource),
  marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker(mesh(Name,MeshFile), MarkerObject, Parent) :-
  marker(mesh(Name), MarkerObject, Parent),
  marker_mesh_resource(MarkerObject, MeshFile).

marker(object_without_children(Identifier), MarkerObject, Parent) :-
  marker_primitive(cube, object_without_children(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier, MarkerObject).

marker(object(Identifier), MarkerObject, Parent) :-
  marker_primitive(cube, object(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier, MarkerObject),
  marker_children(Identifier,Children),
  forall(
    member( Child,Children ), ignore((
      marker( object(Child), _, MarkerObject )
    ))
  ).

marker(agent(Identifier), MarkerObject, Parent) :-
  marker(kinematic_chain(Identifier,agent(Identifier)), MarkerObject, Parent).

marker(kinematic_chain(Identifier), MarkerObject, Parent) :-
  marker(kinematic_chain(Identifier,kinematic_chain(Identifier)), MarkerObject, Parent).

marker(kinematic_chain(Identifier,Name), MarkerObject, Parent) :-
  marker_primitive(arrow, Name, MarkerObject, Parent),
  marker_initialize_object(Identifier,MarkerObject),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker_has_visual( Link ),
    marker( object_without_children(Link), _, MarkerObject )
  ))).

marker(stickman(Identifier), MarkerObject, Parent) :-
  marker_primitive(sphere, stickman(Identifier), MarkerObject, Parent),
  marker_initialize_object(Identifier,MarkerObject),
  marker_color(MarkerObject, [1.0,1.0,0.0,1.0]),
  
  marker_links(Identifier, Links),
  forall( member( Link,Links ), ignore((
    marker( object_without_children(Link), LinkMarker, MarkerObject ),
    marker_type(LinkMarker, sphere),
    
    marker_succeeding_links(Link, SucceedingLinks),
    forall(
      member( SucceedingLink,SucceedingLinks ),
      once( marker( cylinder_tf(Link,SucceedingLink), _, MarkerObject) )
    )
  ))).

marker(text(Id), MarkerObject, Parent) :-
  marker_primitive(text_view_facing, text(Id), MarkerObject, Parent),
  marker_color(MarkerObject, [0.6,0.9,0.6,1.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker(text(Id,Text), MarkerObject, Parent) :-
  marker(text(Id), MarkerObject, Parent),
  marker_text(MarkerObject, Text).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Updating marker for given timepoint/timerange
%

%% marker_update is det.
%
% Updates all markers for the current local time.
%
marker_update :-
  get_timepoint(T),
  marker_update(T).

%% marker_update(+MarkerTerm) is det.
%
% Updates the marker identified by @MarkerTerm.
%
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
%
marker_update(MarkerTerm) :-
  compound(MarkerTerm),
  get_timepoint(T),
  marker_update(MarkerTerm,T).

%% marker_update(+T) is det.
%
% Updates all markers for the time identified by @T.
%
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604) or an interval (e.g., interval(1396512604, 1396512608, 0.5)).
%
marker_update(T) :-
  ( atom(T) ; number(T) ),
  forall(
    marker(MarkerTerm, _),
    ignore(marker_update(MarkerTerm, T))
  ).

%% marker_update(+MarkerTerm, +T) is det.
%
% Updates the marker identified by @MarkerTerm
% for the time identified by @T.
%
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604) or an interval (e.g., interval(1396512604, 1396512608[, 0.5])).
%
marker_update(MarkerTerm, T) :-
  atom(T), time_term(T, T_Term),
  marker_update(MarkerTerm, T_Term).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  atom(T0), time_term(T0, T0_Term),
  atom(T1), time_term(T1, T1_Term),
  marker_update(MarkerTerm, interval(T0_Term,T1_Term,Interval)).

marker_update(MarkerTerm, T) :-
  number(T),
  marker_update(MarkerTerm, time(T,T)).

marker_update(MarkerTerm, interval(T0,T1,Interval)) :-
  number(T0), number(T1),
  marker_update(MarkerTerm, time(T0,interval(T0,T1,Interval))).

marker_update(MarkerTerm, time(T,Arg)) :-
  number(T),
  T_float is float(T),
  marker(MarkerTerm, MarkerObject),
  marker_timestamp(MarkerObject, Last_T),
  % Only update once for given timestamp
  ( Last_T is T_float ; once((
    marker_timestamp(MarkerObject, T_float),
    marker_update(MarkerTerm, MarkerObject, Arg)
  ))).

%% marker_update(+MarkerTerm, +MarkerObject, +T) is det.
%
% Updates the marker @MarkerObject that is identified by @MarkerTerm
% for the time identified by @T.
%
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604) or an interval (e.g., interval(1396512604, 1396512608, 0.5)).
%
marker_update(object_without_children(Identifier), MarkerObject, T) :-
  marker_update(object(Identifier), MarkerObject, T).

marker_update(object(Identifier), MarkerObject, T) :-
  ignore((
    marker_lookup_transform(Identifier,T,(Translation,Orientation)),
    marker_pose(MarkerObject,Translation,Orientation)
  )),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), once((
    marker_object(ChildTerm, ChildObject),
    marker_update(ChildTerm, ChildObject, T),
    marker_timestamp(MarkerObject, T)
  ))).

marker_update(link(Link), MarkerObject, T) :-
  marker_lookup_transform(Link,T,(Translation,Orientation)),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(pointer(From,To), MarkerObject, T) :-
  mng_lookup_transform(From, To, T, Pose),
  matrix_rotation(Pose, Orientation),
  matrix_translation(Pose, Translation),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(cylinder_tf(From,To), MarkerObject, T) :-
  marker_tf_frame(From, FromFrame),
  marker_tf_frame(To, ToFrame),
  mng_lookup_transform('/map', FromFrame, T, Pose0),
  mng_lookup_transform('/map', ToFrame, T, Pose1),
  matrix_translation(Pose0, [X0,Y0,Z0]),
  matrix_translation(Pose1, [X1,Y1,Z1]),
  DX is X1-X0, DY is Y1-Y0, DZ is Z1-Z0,
  Distance is sqrt(DX*DX + DY*DY + DZ*DZ),
  QX is -DY, QY is DX, QZ is 0.0, QW is Distance + DZ,
  X is 0.5*(X0+X1), Y is 0.5*(Y0+Y1), Z is 0.5*(Z0+Z1),
  marker_scale(MarkerObject, [0.1,0.1,Distance]),
  marker_pose(MarkerObject, [X,Y,Z], [QW,QX,QY,QZ]).

marker_update(trajectory(Link), MarkerObject, T1) :-
  number(T1),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  length(Children, MarkerCount),
  (  MarkerCount > 1
  -> (
    Children = [M0|[M1|_]],
    marker_timestamp(M0, T_Marker0),
    marker_timestamp(M1, T_Marker1),
    T_step is T_Marker1 - T_Marker0,
    T0 is T1 - T_step * MarkerCount,
    marker_update(trajectory(Link), MarkerObject, interval(T0,T1,dt(T_step)))
  ) ; (
    T0 is T1 - 10.0,
    marker_update(trajectory(Link), MarkerObject, interval(T0,T1,dt(0.5)))
  )).

marker_update(trajectory(Link), MarkerObject, interval(T0,T1)) :-
  marker_update(trajectory(Link), MarkerObject, (T0,T1,dt(0.5))).

marker_update(trajectory(Link), MarkerObject, interval(T0,T1,Interval)) :-
  jpl_call(MarkerObject, 'clear', [], _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall( member(ChildObject,Children), ignore((
    v_marker_object(ChildTerm, ChildObject),
    marker_remove(ChildTerm)
  ))),
  marker_update_trajectory(trajectory(Link), MarkerObject, interval(T0,T1,Interval)).

marker_update(trail(Link), MarkerObject, T) :-
  marker_update(trajectory(Link), MarkerObject, T),
  % Alpha fade the trail
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  length(Children, MarkerCount),
  ignore((
    MarkerCount > 1,
    Alpha_Step is 1.0 / MarkerCount,
    marker_alpha(Children, 0.0, Alpha_Step)
  )).

marker_update(average_trajectory(Link), MarkerObject, interval(T0,T1,Interval)) :-
  jpl_call(MarkerObject, 'clear', [], _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall( member(ChildObject,Children), ignore((
    v_marker_object(ChildTerm, ChildObject),
    marker_remove(ChildTerm)
  ))),
  marker_update_trajectory(average_trajectory(Link), MarkerObject, interval(T0,T1,Interval)).

marker_update(_, MarkerObject, T) :-
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), ignore((
    marker(ChildTerm, ChildObject),
    marker_update(ChildTerm, ChildObject, T),
    marker_timestamp(ChildObject, T)
  ))).

%% marker_update_trajectory(+MarkerTerm, +MarkerObject, +T) is det.
%
% Updates the trajectory marker @MarkerObject that is identified by @MarkerTerm
% for the time identified by @T.
%
% @param MarkerTerm A term that identifies the marker (e.g., trajectory('/base_link'))
% @param MarkerObject The MarkerObject instance
% @param T A time atom (e.g., 'timepoint_1396512604'), time number (e.g., 1396512604) or an interval (e.g., interval(1396512604, 1396512608, dt(0.5))).
%
marker_update_trajectory(trajectory(Link), MarkerObject, interval(T0,T1,count(Count))) :-
  Interval is (T1 - T0) / Count,
  marker_update_trajectory(trajectory(Link), MarkerObject, interval(T0,T1,dt(Interval))).
marker_update_trajectory(trajectory(Link), MarkerObject, interval(T0,T1,dt(Interval))) :-
  T0 =< T1,
  atom_concat(Link, T0, MarkerName),
  jpl_call(MarkerObject, 'createMarker', [MarkerName], ChildMarker),
  marker_lookup_transform(Link, T0, (Translation,Orientation)),
  marker_pose(ChildMarker, Translation, Orientation),
  T_next is T0 + Interval,
  marker_update_trajectory(trajectory(Link), MarkerObject, interval(T_next,T1,dt(Interval))).
marker_update_trajectory(_, _, interval(T0,T1,_)) :- T0 > T1.

marker_update_trajectory(average_trajectory(Link), MarkerObject, interval(T0,T1,Interval)) :-
  marker_trajectories_sample(Link, interval(T0,T1,Interval), TrajectoryList),
  marker_trajectory_average(TrajectoryList, AverageTrajectory),
  marker_update_trajectory(average_trajectory(Link), MarkerObject, 0, AverageTrajectory).

marker_update_trajectory(average_trajectory(_), _, _, []).
marker_update_trajectory(average_trajectory(Link), MarkerObject, Index, [(Translation,Orientation)|Rest]) :-
  atom_concat(Link, Index, MarkerName),
  jpl_call(MarkerObject, 'createMarker', [MarkerName], ChildMarker),
  marker_pose(ChildMarker, Translation, Orientation),
  Index_Next is Index + 1,
  marker_update_trajectory(average_trajectory(Link), MarkerObject, Index_Next, Rest).

% TODO: Should be moved to another module
marker_trajectories_sample(_, interval([],_,_), []).
marker_trajectories_sample(_, interval(_,[],_), []).
marker_trajectories_sample(Link, interval([T0|T0s],[T1|T1s],Interval), [Trajectory|Rest]) :-
  marker_trajectory_sample(Link, interval(T0,T1,Interval), Trajectory),
  marker_trajectories_sample(Link, interval(T0s,T1s,Interval), Rest).

% TODO: Should be moved to another module
marker_trajectory_sample(_, interval(T0,T1,_), []) :- T0 > T1.
marker_trajectory_sample(Link, interval(T0,T1,count(Count)), Trajectory) :-
  Interval is (T1 - T0) / Count,
  marker_trajectory_sample(Link, interval(T0,T1,dt(Interval)), Trajectory).
marker_trajectory_sample(Link, interval(T0,T1,dt(Interval)), [Pose|Rest]) :-
  T0 =< T1,
  marker_lookup_transform(Link, T0, Pose),
  T0_next is T0 + Interval,
  marker_trajectory_sample(Link, interval(T0_next,T1,dt(Interval)), Rest).

% TODO: Should be moved to another module
marker_trajectory_average(TrajectoryList, AverageTrajectory) :-
  marker_trajectory_average(TrajectoryList, AverageTrajectory, 0).
marker_trajectory_average(TrajectoryList, [AvgPose,Rest], Index) :-
  findall(Pose, (
    member(Trajectory, TrajectoryList),
    nth0(Index, Trajectory, Pose)
  ), Poses),
  length(Poses,NumSamples), NumSamples > 0,
  marker_pose_average(Poses, AvgPose),
  Index_Next is Index + 1,
  marker_trajectory_average(TrajectoryList, Rest, Index_Next).

% TODO: Should be moved to another module
marker_pose_average(PoseList, (AvgPos,AvgOrientation)) :-
  marker_position_average(PoseList, AvgPos),
  marker_orientation_average(PoseList, AvgOrientation).

% TODO: Should be moved to another module
marker_position_average(PoseList, [X_Avg,Y_Avg,Z_Avg]) :-
  findall(X, member(([X,_,_],_), PoseList), Xs),
  findall(Y, member(([_,Y,_],_), PoseList), Ys),
  findall(Z, member(([_,_,Z],_), PoseList), Zs),
  length(PoseList, NumSamples),
  sum_list(Xs, X_Sum), X_Avg is X_Sum / NumSamples,
  sum_list(Ys, Y_Sum), Y_Avg is Y_Sum / NumSamples,
  sum_list(Zs, Z_Sum), Z_Avg is Z_Sum / NumSamples.

% TODO: Should be moved to another module
marker_orientation_average(PoseList, [W_Avg,X_Avg,Y_Avg,Z_Avg]) :-
  findall(W, member((_,[W,_,_,_]), PoseList), Ws),
  findall(X, member((_,[_,X,_,_]), PoseList), Xs),
  findall(Y, member((_,[_,_,Y,_]), PoseList), Ys),
  findall(Z, member((_,[_,_,_,Z]), PoseList), Zs),
  length(PoseList, NumSamples),
  sum_list(Ws, W_Sum), W_Avg is W_Sum / NumSamples,
  sum_list(Xs, X_Sum), X_Avg is X_Sum / NumSamples,
  sum_list(Ys, Y_Sum), Y_Avg is Y_Sum / NumSamples,
  sum_list(Zs, Z_Sum), Z_Avg is Z_Sum / NumSamples.

marker_trajectory_length(MarkerTerm, Length) :-
  marker(MarkerTerm, MarkerObject),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  length(Children, MarkerCount),
  marker_trajectory_length(MarkerTerm, Length, Children, MarkerCount, 0).

marker_trajectory_length(MarkerTerm, Length, Children, MarkerCount, Index) :-
  (  Index > MarkerCount - 2
  -> Length = 0.0
  ;  (
    Index_Next is Index + 1,
    nth0(Index, Children, Child0),
    nth0(Index_Next, Children, Child1),
    marker_distance(Child0, Child1, D0),
    marker_trajectory_length(MarkerTerm, D1, Children, MarkerCount, Index_Next),
    Length is D0 + D1
  )).

marker_distance(Marker0, Marker1, Distance) :-
  marker_translation(Marker0, [X0,Y0,Z0]),
  marker_translation(Marker1, [X1,Y1,Z1]),
  Distance is sqrt( (X1-X0)^2 + (Y1-Y0)^2 + (Z1-Z0)^2 ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Highlighting marker
%

%% marker_highlight(+MarkerObject) is det.
%
% Highlight markers identified by @MarkerObject with default highlight color.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
%
marker_highlight(MarkerObject) :-
  jpl_list_to_array([1.0,0.0,0.0,1.0], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

%% marker_highlight(+MarkerObject, +Color) is det.
%
% Highlight markers identified by @MarkerObject with given highlight color @Color.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Color 3D/4D number list or HTML color atom (e.g., '#ff0000')
%
marker_highlight(MarkerObject, [R,G,B]) :-
  jpl_list_to_array([R,G,B,1.0], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

marker_highlight(MarkerObject, [R,G,B,A]) :-
  jpl_list_to_array([R,G,B,A], ColorArray),
  marker_highlight(MarkerObject, ColorArray).

marker_highlight(MarkerTerm, ColorArg) :-
  compound(MarkerTerm),
  marker(MarkerTerm, MarkerObject),
  marker_highlight(MarkerObject, ColorArg).

marker_highlight(MarkerObject, ColorArg) :-
  jpl_is_object(MarkerObject),
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
% Removes highlights from markers identified by @MarkerObject.
%
% @param MarkerObject MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
%
marker_highlight_remove(MarkerTerm) :-
  not( jpl_is_object(MarkerTerm) ),
  marker(MarkerTerm, MarkerObject),
  marker_highlight_remove(MarkerObject).

marker_highlight_remove(MarkerObject) :-
  jpl_is_object(MarkerObject),
  jpl_call(MarkerObject, 'hasHighlight', [], @(true)),
  jpl_call(MarkerObject, 'removeHighlight', [], _),
  jpl_call(MarkerObject, 'getChildren', [], ChildrenArray),
  jpl_array_to_list(ChildrenArray,Children),
  forall(member(ChildObject,Children), ignore((
    marker_highlight_remove(ChildObject)
  ))).


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
marker_properties(Marker, [X|Args]) :-
  marker_property(Marker, X),
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

marker_property(Marker, pose(Position,Orientation)) :-
  marker_pose(Marker, Position, Orientation).

marker_property(Marker, mesh(Mesh)) :-
  marker_mesh_resource(Marker, Mesh).

marker_property(Marker, text(Text)) :-
  marker_text(Marker, Text).

marker_property(Marker, timestamp(T)) :-
  marker_timestamp(Marker, T).

%% marker_timestamp(+Marker, ?Value) is det.
%
% Read or set the timestamp property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_timestamp(Marker, T) :-
  marker_call(Marker,T,(get_marker_timestamp,set_marker_timestamp)).

%% marker_duration(+Marker, ?Value) is det.
%
% Read or set the duration property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_duration(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_duration,set_marker_duration)).

%% marker_type(+Marker, ?Value) is det.
%
% Read or set the type property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_type(Marker, Type) :-
  marker_call(Marker,Type,(get_marker_type,set_marker_type)).

%% marker_scale(+Marker, ?Value) is det.
%
% Read or set the scale property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_scale(Marker, Scale) :-
  marker_call(Marker,Scale,(get_marker_scale,set_marker_scale)).

%% marker_color(+Marker, ?Value) is det.
%
% Read or set the color property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_color(Marker, Color) :-
  marker_call(Marker,Color,(get_marker_color,set_marker_color)).

%% marker_alpha(+Marker, ?Value) is det.
%
% Read or set the alpha property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_alpha(Marker, Alpha) :-
  marker_call(Marker,Alpha,(get_marker_alpha,set_marker_alpha)).

%% marker_alpha(+MarkerList, +Alpha, +AlphaStep) is det.
%
% Apply alpha fade on markers in @MarkerList.
%
% @param MarkerList List of MarkerObject instances or terms that identify the marker (e.g., trajectory('/base_link'))
% @param Alpha The alpha value for the first marker in @MarkerList
% @param AlphaStep Delta alpha value
%
marker_alpha([Child|Rest], Alpha, AlphaStep) :-
  marker_alpha(Child, Alpha),
  Alpha_Next is Alpha + AlphaStep,
  marker_alpha(Rest, Alpha_Next, AlphaStep).
marker_alpha([], _, _).

%% marker_mesh_resource(+Marker, ?Value) is det.
%
% Read or set the mesh_resource property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_mesh_resource(Marker, Mesh) :-
  marker_call(Marker,Mesh,(get_marker_mesh,set_marker_mesh)).

%% marker_pose(+Marker, ?Value) is det.
%
% Read or set the pose property of marker @Marker.
% @Value must be unifyable with pose(Position,Orientation).
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_pose(Marker, pose(Position,Orientation)) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

marker_pose(Marker, Position, Orientation) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

%% marker_orientation(+Marker, ?Value) is det.
%
% Read or set the orientation property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_orientation(Marker, Orientation) :-
  marker_call(Marker, Orientation, (get_marker_orientation,set_marker_orientation)).

%% marker_translation(+Marker, ?Value) is det.
%
% Read or set the translation property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_translation(Marker, Position) :-
  marker_call(Marker, Position, (get_marker_translation,set_marker_translation)).

%% marker_text(+Marker, ?Value) is det.
%
% Read or set the text property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_text(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_text,set_marker_text)).

%% marker_has_visual(+Marker, ?Value) is det.
%
% Read or set the has_visual property of marker @Marker.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
%
marker_has_visual(Marker, V) :-
  marker_call(Marker,V,(get_marker_has_visual,set_marker_has_visual)).

%% marker_call(+Marker, ?Value, (Get,Set)) is det.
%
% Read or set a property of marker(s) @Marker.
% @Marker can also be a list of markers.
%
% @param Marker MarkerObject instance or a term that identifies the marker (e.g., trajectory('/base_link'))
% @param Value The property value
% @param Get The getter method
% @param Set The setter method
%
marker_call([Marker|Rest], Value, (Get,Set)) :-
  jpl_is_object(Marker),
  call(Set, Marker, Value),
  marker_call(Rest, Value, (Get,Set)).

marker_call([Marker|Rest], Value, (Get,Set)) :-
  marker(Marker,MarkerObj),
  marker_call([MarkerObj|Rest], Value, (Get,Set)).

marker_call([], _, _) :- true.

marker_call(Marker, Value, (Get,Set)) :-
  compound(Marker),
  marker(Marker,MarkerObj),
  marker_call(MarkerObj, Value, (Get,Set)).

marker_call(Marker, Value, (Get,Set)) :-
  ground(Value),
  marker_call([Marker], Value, (Get,Set)).

marker_call(Marker, Value, (Get,_)) :-
  not(ground(Value)), jpl_is_object(Marker),
  call(Get, Marker, Value).

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
  jpl_list_to_array([X,Y,Z], ScaleArray),
  jpl_call(MarkerObj, 'setScale', [ScaleArray], _).

set_marker_scale(MarkerObj, Scale) :-
  number(Scale), set_marker_scale(MarkerObj, [Scale,Scale,Scale]).

get_marker_color(MarkerObj, [R,G,B,A]) :-
  jpl_call(MarkerObj, 'getColor', [], ColorArray),
  jpl_array_to_list(ColorArray,[R,G,B,A]).

set_marker_color(MarkerObj, [R,G,B,A]) :-
  atom(R),atom(G),atom(B),atom(A),
  atom_number(R,R_N),
  atom_number(G,G_N),
  atom_number(B,B_N),
  atom_number(A,A_N),
  set_marker_color(MarkerObj, [R_N,G_N,B_N,A_N]).

set_marker_color(MarkerObj, [R,G,B,A]) :-
  number(R),number(G),number(B),number(A),
  jpl_list_to_array([R,G,B,A], ColorArray),
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

get_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getTranslation', [], TranslationArray),
  jpl_array_to_list(TranslationArray,[X,Y,Z]).

get_marker_orientation(MarkerObj, [QW,QX,QY,QZ]) :-
  jpl_call(MarkerObj, 'getOrientation', [], OrientationArray),
  jpl_array_to_list(OrientationArray,[QW,QX,QY,QZ]).

get_marker_pose(MarkerObj, pose(Translation,Orientation)) :-
  get_marker_translation(MarkerObj, Translation),
  get_marker_orientation(MarkerObj, Orientation).

set_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_list_to_array([X,Y,Z], Array),
  jpl_call(MarkerObj, 'setTranslation', [Array], _).

set_marker_orientation(MarkerObj, [QW,QX,QY,QZ]) :-
  jpl_list_to_array([QW,QX,QY,QZ], Array),
  jpl_call(MarkerObj, 'setOrientation', [Array], _).

set_marker_pose(MarkerObj, pose(Translation,Orientation)) :-
  set_marker_translation(MarkerObj, Translation),
  set_marker_orientation(MarkerObj, Orientation).
