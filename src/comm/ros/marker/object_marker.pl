:- module(object_marker,
	[ object_marker/3
	]).

:- use_module(library('lang/terms/is_at'),
    [ is_at/2
    ]).
:- use_module(library('model/EASE/OBJ'),
    [ object_mesh_path/2,
      object_color_rgb/2,
      object_dimensions/4
    ]).

%% object_marker(+Obj,-MarkerData) is semidet.
%
% Maps an object entity to its marker parameters.
%
% @param Obj object IRI
% @param MarkerData marker parameters
%
object_marker(Obj,Scope,
	[ pose(ObjPose)
	| MarkerData
	]) :-
	ask(is_at(Obj,ObjPose),Scope),
	object_marker1(Obj,MarkerData),
	!.

%% prefer mesh markers
object_marker1(Obj,
	[ type(mesh_resource),
	  mesh(MeshPath),
	  scale([1,1,1]),
	  color(RGBA)
	]) :-
	object_mesh_path(Obj,MeshPath),
	( file_name_extension(_, stl, MeshPath)
	-> RGBA=[0,0,0,0]
	;  object_marker_rgba(RGBA)
	).

%% fallback to cube marker
object_marker1(Obj,
	[ type(cube),
	  color(RGBA),
	  scale(Scale)
	]) :-
	object_marker_rgba(Obj,RGBA),
	object_marker_scale(Obj,Scale).

%%
object_marker_rgba(Object,[R,G,B,1]) :-
	object_color_rgb(Object,[R,G,B]),
	!.

object_marker_rgba(_,[1,1,1,1]).

%%
object_marker_scale(Object,[X,Y,Z]) :-
	object_dimensions(Object,X,Y,Z),
	!.

object_marker_scale(_,[0.1,0.1,0.1]).
