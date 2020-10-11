:- module(object_marker,
	[ object_marker/4
	]).

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('model/SOMA/OBJ'),
    [ object_shape/4
    ]).
:- use_module(library('ros/tf/tf_tree')).
	
%% object_marker
%
% Maps an object entity to its marker parameters.
%
% @param Obj object IRI
% @param MarkerData marker parameters
%
object_marker(Obj,_,_,_) :-
	is_urdf_joint(Obj), !,
	fail.

object_marker(Obj,QScope->_,MarkerID,MarkerData) :-
	catch(
		object_marker0(Obj,QScope,MarkerID,MarkerData),
		Exc,
		(log_error(Exc),fail)
	).

object_marker0(Obj,QScope,MarkerID,
		[ pose(Pose) | MarkerData ]) :-
	ask(object_shape(Obj,Shape,Origin,Material),QScope->_),
	QScope=[_,QS],
	get_shape_pose_(QS,Origin,MarkerID,Pose),
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
get_shape_pose_(QS,Origin,ObjFrame,Pose) :-
	setting(marker_plugin:reference_frame,Frame),
	Frame \= '',
	!,
	time_scope_data(QS,[_QSince,QUntil]),
	tf_plugin:strip_operator_(QUntil,Stamp),
	Origin=[ObjFrame,Pos_shape,Rot_shape],
	% Get the pose of Part in given reference frame
	%rdf_split_url(IRIPrefix,_,Obj),
	%rdf_split_url(IRIPrefix,ObjFrame,Part),
	%once(ask(is_at(Part,[Frame,Pos_obj,Rot_obj]),QScope->_)),
	tf_tree_get(Stamp,Tree),
	tf_tree_lookup(Tree,ObjFrame,[Frame,Pos_obj,Rot_obj]),
	% Compute the marker pose in given frame
	% FIXME: might be faster to have special handling for identity transform
	transform_multiply(
		[Frame,ObjFrame,Pos_obj,Rot_obj],
		[ObjFrame,shape,Pos_shape,Rot_shape],
		[Frame,shape,Pos_frame,Rot_frame]),
	Pose=[Frame,Pos_frame,Rot_frame].

get_shape_pose_(_,Pose,ObjFrame,Pose) :-
	!,
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
