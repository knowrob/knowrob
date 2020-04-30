
:- module(object_state_publisher,
    [
      mark_dirty_objects/1
    ]).
/** <module> TODO
  
  @author Daniel Beßler
  @license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/model/Object')).
:- use_module(library('knowrob/memory/object_pose'), [
    object_pose_data/3,
    current_object_pose/2
]).

:-  rdf_meta
    mark_dirty_objects(t).

:- use_foreign_library('libobject_state_publisher.so').

% TODO: 
%notify(object_changed(Object)) :-
  %fail.

%notify(objects_changed(Object)) :-
  %fail.

%%
mark_dirty_objects([]) :- !.
mark_dirty_objects(Objects) :-
  %
  findall(ObjState, (
    member(Obj,Objects),
    object_pose_data(Obj,_,_),
    object_state(Obj,ObjState)
  ), ObjStates),
  object_state_add_cpp(ObjStates).

%% 
object_state(Obj, State) :-
  object_state(Obj, State, _{}).

object_state(Obj, State, Properties_list) :-
  is_list(Properties_list),!,
  findall(Key-Val, (
    member(X,Properties_list),
    X=..[Key,Val]
  ), Pairs),
  dict_pairs(Dict, _, Pairs),
  object_state(Obj, State, Dict).

object_state(Obj, [
   Obj,       % object_id
   FrameName, % frame_name
   TypeName,  % object_type
   Shape,     % shape
   Mesh,      % mesh_path
   [R,G,B,A], % color
   [D,W,H],   % size
   Pose,      % pose
   StaticTransforms % static_transforms
], Properties) :-
  ( get_dict(timestamp,Properties,Time) ;
    get_time(Time)
  ),
  %
  object_frame_name(Obj,FrameName),
  once(is_a(Obj,Type)),
  % FIXME: rdf_split_url is deprecated
  %iri_xml_namespace(_,TypeName,Type),
  rdf_split_url(_,TypeName,Type),
  % get the shape, default to BoxShape
  ( get_dict(shape,Properties,ShapeIri) ;
    object_shape_type(Obj,ShapeIri);
    rdf_equal(ShapeIri,ease_obj:'BoxShape')
  ),
  object_state_shape_(ShapeIri,Shape),
  % get the color, default to grey color
  ( get_dict(color,Properties,[R,G,B,A]) ;
    object_color(Obj,[R,G,B,A]);
    [R,G,B,A]=[0.5,0.5,0.5,1.0]
  ),
  % get mesh path or empty string
  ( get_dict(mesh,Properties,Mesh) ;
    object_mesh_path(Obj,Mesh);
    Mesh=''
  ),
  % get the object bounding box
  ( get_dict(bbox,Properties,[D,W,H]) ;
    object_dimensions(Obj,D,W,H);
    [D,W,H] = [0.05,0.05,0.05]
  ), !,
  % handle transforms
  ( object_pose(Obj, Pose, Time); (
    print_message(warning, unlocalized(Obj)),
    fail
  )),
  findall([ObjFrame,FeatureFrame,Pos,Rot], (
    object_feature(Obj,Feature),
    object_frame_name(Obj, ObjFrame),
    current_object_pose(Feature, [_,FeatureFrame,Pos,Rot])
  ), StaticTransforms),
  !.

%object_state_shape_(Iri,0) :- is_a(Iri,ease_obj:'Arrow'),!.
object_state_shape_(Iri,1) :- is_a(Iri,ease_obj:'BoxShape'),!.
object_state_shape_(Iri,2) :- is_a(Iri,ease_obj:'SphereShape'),!.
object_state_shape_(Iri,3) :- is_a(Iri,ease_obj:'CylinderShape'),!.
