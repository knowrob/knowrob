:- module(object_marker,
	[ object_marker/4
	]).

:- use_module(library('model/EASE/OBJ'),
    [ object_shape/3,
      object_color_rgb/2
    ]).

%% object_marker(+Obj,-MarkerData) is semidet.
%
% Maps an object entity to its marker parameters.
%
% @param Obj object IRI
% @param MarkerData marker parameters
%
object_marker(Obj,Scope,
	MarkerID,
	[ pose(ShapeOrigin)
	| MarkerData
	]) :-
	catch((
		ask(object_shape(Obj,Shape,ShapeOrigin),Scope),
		ShapeOrigin=[MarkerID,_,_],
		object_marker1(Shape,Obj,MarkerData)),
		Exc,
		(log_error(Exc),fail)
	).

object_marker1(
	mesh(MeshPath,Scale), Obj,
	[ type(mesh_resource),
	  mesh(MeshPath),
	  scale(Scale),
	  color(RGBA)
	]) :-
	(  file_name_extension(_, stl, MeshPath)
	-> object_marker_rgba(Obj,RGBA)
	;  RGBA=[0,0,0,0]
	).

object_marker1(
	box(X,Y,Z), Obj,
	[ type(cube),
	  scale([X,Y,Z]),
	  color(RGBA)
	]) :-
	object_marker_rgba(Obj,RGBA).

object_marker1(
	sphere(Radius), Obj,
	[ type(sphere),
	  scale([Radius,Radius,Radius]),
	  color(RGBA)
	]) :-
	object_marker_rgba(Obj,RGBA).

object_marker1(
	cylinder(Radius,Length), Obj,
	[ type(sphere),
	  scale([Radius,Radius,Length]),
	  color(RGBA)
	]) :-
	object_marker_rgba(Obj,RGBA).

%%
object_marker_rgba(Object,[R,G,B,1]) :-
	object_color_rgb(Object,[R,G,B]),
	!.

object_marker_rgba(_,[1,1,1,1]).

