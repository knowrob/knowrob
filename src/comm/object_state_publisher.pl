:- module(object_state_publisher,
    [ object_state(r,-),
      object_state(r,-,+)
    ]).
/** <module> TODO
  
  @author Daniel Beßler
  @license BSD
*/

:- use_module(library('semweb/rdf_db'),
    [ rdf_equal/2,
      rdf_split_url/3
    ]).
:- use_module(library('model/DUL/Object'),
    [ has_object_type/2
    ]).
:- use_module(library('model/EASE/OBJ'),
    [ object_feature/2,
      object_color_rgb/2,
      object_mesh_path/2,
      object_dimensions/4,
      object_shape_type/2
    ]).
:- use_module(library('lang/scopes/temporal'),
    [ time_scope/3
    ]).
:- use_module(library('lang/terms/is_at'),
    [ is_at/2
    ]).

:- use_foreign_library('libobject_state_publisher.so').

%%
%
notify:notify_hook(object_changed(Object)) :-
  notify(objects_changed([Object])).

notify:notify_hook(objects_changed([])) :-
  !.

notify:notify_hook(objects_changed(Objects)) :-
  findall(ObjState, (
    member(Obj,Objects),
    object_state(Obj,ObjState)
  ), ObjStates),
  object_state_add_cpp(ObjStates).

%%%

%% 
object_state(Obj, State) :-
	object_state(Obj, State, _{}).

object_state(Obj, State, Properties_list) :-
	is_list(Properties_list),!,
	% TODO: why not use builtin `option` instead?
	findall(Key-Val, (
		member(X,Properties_list),
		X=..[Key,Val]
	), Pairs),
	dict_pairs(Dict, _, Pairs),
	object_state(Obj, State, Dict).

object_state(Obj,
		[ Obj,       % object_id
		  FrameName, % frame_name
		  TypeName,  % object_type
		  Shape,     % shape
		  Mesh,      % mesh_path
		  [R,G,B,1], % color
		  [D,W,H],   % size
		  [RefFrame,FrameName,Pos,Rot],  % pose
		  StaticTransforms % static_transforms
		], Properties) :-
	%
	has_object_type(Obj,Type),
	rdf_split_url(_,TypeName,Type),
	% FIXME: I think the idea was that there could be many localizations of one object.
	%        so which one to pick? I guess best would be to refer to global coordinate system?
	%        then object state publisher would only get data within some specified system
	object_localization(Obj,Loc),
	has_region(Loc,Pose),
	tripledb_ask(Pose,knowrob:frameName,FrameName),
	% get the state scope
	( get_dict(timestamp,Properties,Time)
	-> true
	;  get_time(Time)
	),
	time_scope(=<(Time), >=(Time),TimeScope),
	Scope=[[],TimeScope],
	!, % TODO: look into why there are choicepoints above
	%%
	object_state_shape_(Obj, Shape, Properties, Scope),
	object_state_color_(Obj, [R,G,B], Properties, Scope),
	object_state_mesh_(Obj, Mesh, Properties, Scope),
	object_state_bbox_(Obj, [D,W,H], Properties, Scope),
	object_state_pose_(Obj, [RefFrame,Pos,Rot], Scope),
	%%
	findall([FrameName,FeatureFrame,Pos,Rot], (
		object_feature(Obj,Feature),
		object_state_pose_(Feature, [_,FeatureFrame,Pos,Rot], Scope)
	), StaticTransforms),
	!.

object_state_pose_(Obj,Pose,Scope) :-
	( ask(is_at(Obj,Pose),Scope->_)
	; (	print_message(warning, unlocalized(Obj)),
		fail
	  )
	),
	!.

object_state_color_(Obj,Color,Properties,Scope) :-
	( get_dict(color,Properties,Color)
	; ask(object_color_rgb(Obj,Color),Scope->_)
	; Color=[0.5,0.5,0.5]
	),
	!.

object_state_mesh_(Obj,Mesh,Properties,Scope) :-
	( get_dict(mesh,Properties,Mesh)
	; ask(object_mesh_path(Obj,Mesh),Scope->_)
	; Mesh=''
	),
	!.

object_state_bbox_(Obj,[D,W,H],Properties,Scope) :-
	( get_dict(bbox,Properties,[D,W,H])
	; ask(object_dimensions(Obj,D,W,H),Scope->_)
	; [D,W,H] = [0.05,0.05,0.05]
	),
	!.

object_state_shape_(Obj,Shape,Properties,Scope) :-
	% get the shape, default to BoxShape
	( get_dict(shape,Properties,ShapeIri)
	; ask(object_shape_type(Obj,ShapeIri),Scope->_)
	; rdf_equal(ShapeIri,ease_obj:'BoxShape')
	),
	object_state_shape_(ShapeIri,Shape).

%object_state_shape_(Iri,0) :- rdf_equal(Iri,ease_obj:'Arrow'),!.
object_state_shape_(Iri,1)  :- rdf_equal(Iri,ease_obj:'BoxShape'),!.
object_state_shape_(Iri,2)  :- rdf_equal(Iri,ease_obj:'SphereShape'),!.
object_state_shape_(Iri,3)  :- rdf_equal(Iri,ease_obj:'CylinderShape'),!.
object_state_shape_(Iri,10) :- rdf_equal(Iri,ease_obj:'MeshShape'),!.
