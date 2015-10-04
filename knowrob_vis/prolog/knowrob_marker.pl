

:- module(knowrob_marker,
    [
      marker_publish/0,
      
      marker/2,
      marker_update/2,
      
      marker_properties/2,
      marker_type/2,
      marker_scale/2,
      marker_color/2,
      marker_mesh_resource/2,
      marker_pose/2,
      marker_translation/2,
      marker_text/2
      
      marker_remove_trajectories/0,
      marker_remove_all/0,
      marker_remove/1
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('jpl')).


trajectory_sample(Frame, T, T_end, Inteval, [Pose|Rest]) :-
  T =< T_end, T_next is T + Inteval,
  object_lookup_transform(Frame, T, Pose),
  trajectory_sample(Frame, T_next, T_end, Rest).

trajectory_sample(Frame, T, T_end, Inteval, [Pose]) :-
  T >= T_end, object_lookup_transform(Frame, T_end, Pose).


object_has_visual(Identifier) :-
  not(owl_individual_of(Identifier, 'http://knowrob.org/kb/srdl2-comp.owl#UrdfJoint')),
  not(owl_individual_of(Identifier, 'http://knowrob.org/kb/knowrob.owl#RoomInAConstruction')),
  not(rdf_has(Identifier, knowrob:'hasVisual', literal(type(_,false)))).

object_children(Parent, Children) :-
  findall(Child, (
    rdf_reachable(Part, knowrob:describedInMap, Parent),
    (
      rdf_reachable(Part, knowrob:properPhysicalParts, Child);
      rdf_reachable(Part, srdl2comp:'subComponent', Child);
      rdf_reachable(Part, srdl2comp:'successorInKinematicChain', Child)
    )
  ), Children).

object_links(Identifier, Links) :-
  findall(Link, (
    owl_has(Identifier, srdl2comp:'subComponent', Component),
    owl_has(Component, srdl2comp:'baseLinkOfComposition', BaseLink),
    rdf_reachable(BaseLink, srdl2comp:'successorInKinematicChain', Link),
    owl_individual_of(Link, srdl2comp:'UrdfLink')
  ), Links).

object_frame(Identifier, UrdfName) :-
  rdf_has(Identifier, 'http://knowrob.org/kb/srdl2-comp.owl#urdfName', literal(Tf)),
  atomic_list_concat(['/', Tf], UrdfName).

object_frame(UrdfName, UrdfName).

object_lookup_transform(Identifier, T, (Translation,Orientation)) :-
  object_pose_at_time(Identifier, T, Translation, Orientation).

% TODO: look at object pose at time
object_lookup_transform(Identifier, T, (Translation,Orientation)) :-
  object_lookup_transform(Identifier, '/map', T, (Translation,Orientation)).

object_lookup_transform(Identifier, TargetFrame, T, (Translation,Orientation)) :-
  object_frame(Identifier, TfFrame),
  mng_lookup_transform(TfFrame, TargetFrame, T, Pose),
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


marker_publish :-
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'publishMarker', [], _).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker factory
%

marker_create(Identifier, null, MarkerObject) :-
  marker_create(Identifier, MarkerObject).

marker_create(Identifier, Parent, MarkerObject) :-
  jpl_call(Parent, 'createMarker', [Identifier], MarkerObject),
  not(MarkerObject = @(null)).

marker_create(Identifier, MarkerObject) :-
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'createMarker', [Identifier], MarkerObject),
  not(MarkerObject = @(null)).


marker(Identifier, MarkerObject) :-
  not(Identifier = @(null)),
  jpl_is_object(Identifier),
  MarkerObject = Identifier.

marker(Identifier, MarkerObject) :-
  var(Identifier),
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'getMarkerNames', [], IdentifierArray),
  jpl_list_to_array(IdentifierList, IdentifierArray),
  member(Identifier, IdentifierList),
  marker(Identifier, MarkerObject).

marker(Identifier, MarkerObject) :-
  atom(Identifier),
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'getMarker', [Identifier], MarkerObject),
  not(MarkerObject = @(null)).

marker(primitive(Type,Name), MarkerObject) :-
  marker_create(Name, MarkerObject),
  marker_type(MarkerObject, Type),
  marker_color(MarkerObject, [0.6,0.6,0.6,1.0]),
  marker_scale(MarkerObject, [0.05,0.05,0.05]).

marker(cube(Name), MarkerObject) :-
  marker(primitive(cube,Name), MarkerObject).

marker(sphere(Name), MarkerObject) :-
  marker(primitive(sphere,Name), MarkerObject).

marker(arrow(Name), MarkerObject) :-
  marker(primitive(arrow,Name), MarkerObject).

marker(cylinder(Name), MarkerObject) :-
  marker(primitive(cylinder,Name), MarkerObject).

marker(mesh(Name), MarkerObject) :-
  marker_create(Name, MarkerObject),
  marker_type(MarkerObject, mesh_resource),
  marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker(mesh(Name,MeshFile), MarkerObject) :-
  marker(mesh(Name), MarkerObject),
  marker_mesh_resource(MarkerObject, MeshFile).

marker(object(Identifier), MarkerObject) :-
  marker(object(Identifier,null), MarkerObject).

marker(object(Identifier,Parent), MarkerObject) :-
  marker(cube(object(Identifier)), MarkerObject),
  (  object_has_visual(Identifier)
  -> jpl_call(MarkerObject, 'setHasVisual', [true], _)
  ;  jpl_call(MarkerObject, 'setHasVisual', [false], _)
  ),
  ignore((
    get_model_path(Marker, Path),
    marker_type(MarkerObject, mesh_resource),
    marker_mesh_resource(MarkerObject, Path),
    marker_color(MarkerObject, [0.0,0.0,0.0,0.0]),
    marker_scale(MarkerObject, [1.0,1.0,1.0])
  )),
  ignore(( object_dimensions(Identifier, Scale), marker_scale(MarkerObject, Scale) ))
  ignore(( object_color(Identifier, Color), marker_color(MarkerObject, Color) )).

marker(object_with_children(Identifier), MarkerObject) :-
  marker(object_with_children(Identifier,null), MarkerObject).

marker(object_with_children(Identifier,Parent), MarkerObject) :-
  marker(object(Identifier,Parent), MarkerObject),
  object_children(Identifier,Children),
  forall(
    member( Child,Children ),
    marker( object_with_children(Child,MarkerObject), _ )
  ).

marker(kinematic_chain(Identifier), MarkerObject) :-
  marker(object(kinematic_chain(Identifier)), MarkerObject),
  object_links(Identifier, Links),
  forall(
    member( Link,Links ),
    marker( object(Link,MarkerObject), _ )
  ).

marker(stickman(Identifier), MarkerObject) :-
  marker( object(stickman(Identifier)), MarkerObject ),
  object_links(Identifier, Links),
  forall(
    member( Link,Links ),
    marker( object(Link,MarkerObject), LinkMarker ),
    marker_type(LinkMarker, sphere),
    marker_color(LinkMarker, [1.0,1.0,0.0,1.0]),
    succeeding_links(Link, SucceedingLinks),
    forall(
      member( SucceedingLink,SucceedingLinks ),
      false % TODO
    )
  ).

marker(link(Link), MarkerObject) :-
  marker(arrow(link(Link), MarkerObject)).

marker(trajectory(Link), MarkerObject) :-
  marker(arrow(trajectory(Link), MarkerObject)).

marker(average_trajectory(Link), MarkerObject) :-
  marker(arrow(average_trajectory(Link), MarkerObject)).

marker(pointer(From,To), MarkerObject) :-
  marker(arrow(pointer(From,To), MarkerObject)).

marker(text(Id), MarkerObject) :-
  marker_create(text(Id), MarkerObject),
  marker_type(MarkerObject, text_view_facing),
  marker_color(MarkerObject, [0.6,0.9,0.6,1.0]),
  marker_scale(MarkerObject, [1.0,1.0,1.0]).

marker(text(Id,Text), MarkerObject) :-
  marker(text(Id), MarkerObject),
  marker_text(MarkerObject, Text).


marker_remove_all :-
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'eraseAllMarker', [], _).

marker_remove(Identifier) :-
  atom(Identifier),
  marker_visualisation(MarkerVis),
  jpl_call(MarkerVis, 'eraseMarker', [Identifier], _).

marker_remove_trajectories :-
  forall(
    % FIXME: query with name pattern
    marker(trajectory(Link), MarkerObj),
    marker_remove(trajectory(Link))
  ).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Updating marker
%

marker_update(T) :-
  forall(marker(Identifier, MarkerObject), (
    marker_update(Identifier, MarkerObject, T),
    marker_timestamp(MarkerObject, T)
  )).

marker_update(Identifier, T) :-
  marker(Identifier, MarkerObject),
  (  get_marker_timestamp(MarkerObject,T) ;
  -> true
  ;  (
    marker_timestamp(MarkerObject, T),
    marker_update(Identifier, MarkerObject, T)
  )).

marker_update(object_with_children(Identifier), MarkerObject, T) :-
  marker_update(object(Identifier), MarkerObject, T).

marker_update(object(Identifier), MarkerObject, T) :-
  ignore((
    object_lookup_transform(Identifier,T,(Translation,Orientation)),
    marker_pose(MarkerObject,Translation,Orientation)
  )),
  jpl_call(MarkerObject, 'getChildrenNames', [], ChildrenArray),
  jpl_list_to_array(Children, ChildrenArray),
  forall(member(Child,Children), (
    atom(Child,ChildAtom),
    marker_update(ChildAtom, T)
  )).

marker_update(link(Link), MarkerObject, T) :-
  object_lookup_transform(Link,T,(Translation,Orientation)),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(pointer(From,To), MarkerObject, T) :-
  mng_lookup_transform(From, To, T, Pose),
  matrix_rotation(Pose, Orientation),
  matrix_translation(Pose, Translation),
  marker_pose(MarkerObject,Translation,Orientation).

marker_update(trajectory(Link), MarkerObject, T) :-
  false. % TODO

marker_update(trajectory(Link), MarkerObject, (T0,T1,Interval)) :-
  jpl_call(MarkerObject, 'clear', [], _),
  trajectory_sample(Link,T0,T1,Interval,Samples),
  forall( member((Translation,Orientation),Samples), (
    jpl_call(MarkerObject, 'createMaker', [], Maker),
    marker_pose(Maker,Translation,Orientation)
  )).

marker_update(average_trajectory(Link), MarkerObject, (T0,T1,Interval)) :-
  jpl_call(MarkerObject, 'clear', [], _),
  false. % TODO

marker_update(stickman(Id), MarkerObject, T) :-
  false. % TODO

marker_update(text(Id), MarkerObject, translation(Translation)) :-
  marker_translation(MarkerObject, Translation).

marker_update(MarkerName, MarkerObject, T) :-
  jpl_call(MarkerObject, 'getChildrenNames', [], ChildrenArray),
  jpl_list_to_array(Children, ChildrenArray),
  forall(member(Child,Children), (
    atom(Child, ChildAtom),
    marker_update(ChildAtom, T)
  )).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% Marker properties
%

marker_properties(Marker, Props) :-
  var(Props),
  false. % TODO: read all properties


marker_properties(Marker, [X|Args]) :-
  marker_property(Marker, X),
  marker_properties(Marker, Args).

marker_properties(Marker, []).


marker_property(Marker, type(Type)) :-
  marker_type(Marker, Type).

marker_property(Marker, color(Color)) :-
  marker_color(Marker, Color).

marker_property(Marker, scale(Scale)) :-
  marker_scale(Marker, Color).

marker_property(Marker, pose(Position,Orientation)) :-
  marker_pose(Marker, Position, Orientation).

marker_property(Marker, mesh(Mesh)) :-
  marker_mesh_resource(Marker, Mesh).

marker_property(Marker, text(Text)) :-
  marker_text(Marker, Text).

marker_property(Marker, timestamp(T)) :-
  marker_timestamp(Marker, T).


marker_timestamp(Marker, T) :-
  marker_call(Marker,T,(get_marker_timestamp,set_marker_timestamp)).

marker_duration(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_duration,set_marker_duration)).

marker_type(Marker, Type) :-
  marker_call(Marker,Type,(get_marker_type,set_marker_type)).

marker_scale(Marker, Scale) :-
  marker_call(Marker,Scale,(get_marker_scale,set_marker_scale)).

marker_color(Marker, Color) :-
  marker_call(Marker,Scale,(get_marker_color,set_marker_color)).

marker_mesh_resource(Marker, Mesh) :-
  marker_call(Marker,Mesh,(get_marker_mesh,set_marker_mesh)).

marker_pose(Marker, pose(Position,Orientation)) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

marker_pose(Marker, Position, Orientation) :-
  marker_call(Marker, pose(Position,Orientation), (get_marker_pose,set_marker_pose)).

marker_translation(Marker, Position) :-
  marker_call(Marker, Position, (get_marker_translation,set_marker_translation)).

marker_translation(Marker, Position) :-
  marker_call(Marker, Position, (get_marker_translation,set_marker_translation)).

marker_text(Marker, Text) :-
  marker_call(Marker,Text,(get_marker_text,set_marker_text)).


marker_call([Marker|Rest], Value, (Get,Set)) :-
  (  marker(Marker,MarkerObj),
  -> call(Set, MarkerObj, Value)
  ;  true
  ),
  marker_call(Rest, Value, (Get,Set)).
marker_call([], _, _) :- true.

marker_call(Marker, Value, (Get,Set)) :-
  atom(Marker), nonvar(Value),
  marker_call([Marker], Value, (Get,Set)).

marker_call(Marker, Value, (Get,_)) :-
  atom(Marker), var(Value),
  marker_object(Marker,MarkerObj),
  call(Get, MarkerObj, Value).


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
  jpl_call(MarkerObj, 'getScale', [], Scale),
  jpl_call(Scale, 'getX', [], X),
  jpl_call(Scale, 'getY', [], Y),
  jpl_call(Scale, 'getZ', [], Z).

set_marker_scale(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getScale', [], Scale),
  jpl_call(Scale, 'setX', [X], _),
  jpl_call(Scale, 'setY', [Y], _),
  jpl_call(Scale, 'setZ', [Z], _).

set_marker_scale(MarkerObj, Scale) :-
  number(Scale), set_marker_scale(MarkerObj, [Scale,Scale,Scale]).

get_marker_color(MarkerObj, [R,G,B,A]) :-
  jpl_call(MarkerObj, 'getColor', [], Color),
  jpl_call(Color, 'getR', [], R),
  jpl_call(Color, 'getG', [], G),
  jpl_call(Color, 'getB', [], B),
  jpl_call(Color, 'getA', [], A).

set_marker_color(MarkerObj, [R,G,B,A]) :-
  jpl_call(MarkerObj, 'getColor', [], Color),
  jpl_call(Color, 'setR', [R], _),
  jpl_call(Color, 'setG', [G], _),
  jpl_call(Color, 'setB', [B], _),
  jpl_call(Color, 'setA', [A], _).

set_marker_color(MarkerObj, Color) :-
  number(Color), set_marker_color(MarkerObj, [Color,Color,Color,1.0]).

get_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getPose', [], Pose),
  jpl_call(Pose, 'getPosition', [], Position),
  jpl_call(Position, 'getX', [], X),
  jpl_call(Position, 'getY', [], Y),
  jpl_call(Position, 'getZ', [], Z).

set_marker_translation(MarkerObj, [X,Y,Z]) :-
  jpl_call(MarkerObj, 'getPose', [], Pose),
  jpl_call(Pose, 'getPosition', [], Position),
  jpl_call(Position, 'setX', [X], _),
  jpl_call(Position, 'setY', [Y], _),
  jpl_call(Position, 'setZ', [Z], _).

get_marker_pose(MarkerObj, pose([X,Y,Z],[QW,QX,QY,QZ])) :-
  jpl_call(MarkerObj, 'getPose', [], Pose),
  jpl_call(Pose, 'getOrientation', [], Orientation),
  jpl_call(Orientation, 'getW', [], QW),
  jpl_call(Orientation, 'getX', [], QX),
  jpl_call(Orientation, 'getY', [], QY),
  jpl_call(Orientation, 'getZ', [], QZ),
  jpl_call(Pose, 'getPosition', [], Position),
  jpl_call(Position, 'getX', [], X),
  jpl_call(Position, 'getY', [], Y),
  jpl_call(Position, 'getZ', [], Z).

set_marker_pose(MarkerObj, pose([X,Y,Z],[QW,QX,QY,QZ])) :-
  jpl_call(MarkerObj, 'getPose', [], Pose),
  jpl_call(Pose, 'getOrientation', [], Orientation),
  jpl_call(Orientation, 'setW', [QW], _),
  jpl_call(Orientation, 'setX', [QX], _),
  jpl_call(Orientation, 'setY', [QY], _),
  jpl_call(Orientation, 'setZ', [QZ], _),
  jpl_call(Pose, 'getPosition', [], Position),
  jpl_call(Position, 'setX', [X], _),
  jpl_call(Position, 'setY', [Y], _),
  jpl_call(Position, 'setZ', [Z], _).
