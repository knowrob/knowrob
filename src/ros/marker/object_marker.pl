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
object_marker(Obj,QScope->_,MarkerID,MarkerData) :-
	catch(
		object_marker0(Obj,QScope,MarkerID,MarkerData),
		Exc,
		(log_error(Exc),fail)
	).

object_marker0(Obj,QScope,MarkerID,
		[ pose(Pose) | MarkerData ]) :-
	bagof([Shape0,Origin0,Material0],
		ask(object_shape(Obj,Shape0,Origin0,Material0),QScope->_),
		Shapes
	),
	get_shape_(Obj,QScope,Shapes,Shape,Material,MarkerID,Pose),
	object_marker1(Shape,Material,MarkerData).

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
get_shape_(Obj,QScope,Shapes,
		Shape,Material,ObjFrame,Pose) :-
	setting(marker_plugin:reference_frame,Frame),
	!,
	% Get the pose of Obj in given reference frame
	once(ask(is_at(Obj,[Frame,Pos_obj,Rot_obj]),QScope->_)),
	% Get an object shape
	member([Shape,Origin,Material],Shapes),
	% Compute the marker pose in given frame
	% FIXME: might be faster to have special handling for identity transform
	Origin=[ObjFrame,Pos_shape,Rot_shape],
	transform_multiply(
		[Frame,ObjFrame,Pos_obj,Rot_obj],
		[ObjFrame,foo,Pos_shape,Rot_shape],
		[Frame,foo,Pos_frame,Rot_frame]),
	Pose=[Frame,Pos_frame,Rot_frame].

get_shape_(_Obj,_QScope,Shapes,
		Shape,Material,ObjFrame,Pose) :-
	% Get an object shape
	member([Shape,Pose,Material],Shapes),
	Pose=[ObjFrame,_,_].

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

