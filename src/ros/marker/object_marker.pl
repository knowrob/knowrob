:- module(object_marker,
	[ object_marker/4
	]).

:- use_module(library('model/SOMA/OBJ'),
    [ object_shape/4
    ]).

%% object_marker(+Obj,-MarkerData) is semidet.
%
% Maps an object entity to its marker parameters.
%
% @param Obj object IRI
% @param MarkerData marker parameters
%
object_marker(Obj,QScope->_,
	MarkerID,
	[ pose(MarkerPose)
	| MarkerData
	]) :-
	catch((
		ask(object_shape(Obj,Shape,Origin,Material),QScope->_),
		once((
			object_marker_pose_(Obj,QScope->_,Origin,MarkerPose),
			Origin=[MarkerID,_,_],
			object_marker1(Shape,Material,MarkerData)
		))),
		Exc,
		(log_error(Exc),fail)
	).

object_marker1(
	mesh(MeshPath,Scale), Material,
	[ type(mesh_resource),
	  mesh(MeshPath),
	  scale(Scale),
	  color(RGBA)
	]) :-
	(  file_name_extension(_, stl, MeshPath)
	-> material_rgba(Material,RGBA)
	;  RGBA=[0,0,0,0]
	).

object_marker1(
	box(X,Y,Z), Material,
	[ type(cube),
	  scale([X,Y,Z]),
	  color(RGBA)
	]) :-
	material_rgba(Material,RGBA).

object_marker1(
	sphere(Radius), Material,
	[ type(sphere),
	  scale([Radius,Radius,Radius]),
	  color(RGBA)
	]) :-
	material_rgba(Material,RGBA).

object_marker1(
	cylinder(Radius,Length), Material,
	[ type(cylinder),
	  scale([Radius,Radius,Length]),
	  color(RGBA)
	]) :-
	material_rgba(Material,RGBA).

%%
object_marker_pose_(Obj,Scope,Origin,MarkerPose) :-
	setting(marker_plugin:reference_frame,Frame),
	(	Frame=''
	->	MarkerPose=Origin
	;	(	Origin=[ObjFrame,Pos_shape,Rot_shape],
			ask(is_at(Obj,[Frame,Pos_obj,Rot_obj]),Scope),
			transform_multiply(
				[foo,ObjFrame,Pos_shape,Rot_shape],
				[ObjFrame,Frame,Pos_obj,Rot_obj],
				[foo,Frame,Pos_frame,Rot_frame]),
			MarkerPose=[Frame,Pos_frame,Rot_frame]
		)
	).

%%
material_rgba(material(Material),[R,G,B,A]) :-
	member(rgba([R,G,B,A]),Material),
	ground([R,G,B,A]),
	!.

material_rgba(material(Material),[R,G,B,1]) :-
	member(rgb([R,G,B]),Material),
	ground([R,G,B]),
	!.

material_rgba(_,[1,1,1,1]).

