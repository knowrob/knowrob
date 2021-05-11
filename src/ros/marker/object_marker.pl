:- module(object_marker,
	[ object_marker/3
	]).

:- use_module(library('semweb/rdf_db'),
	[ rdf_split_url/3 ]).
:- use_module(library('model/SOMA'),
    [ object_shape/5 ]).
	
%% object_marker
%
% Maps an object entity to its marker parameters.
%
% @param Obj object IRI
% @param MarkerData marker parameters
%
object_marker(Obj,_,_) :-
	atom(Obj),
	is_urdf_joint(Obj), !,
	fail.

object_marker(Obj,MarkerID,MarkerData) :-
	catch(
		object_marker0(Obj,MarkerID,MarkerData),
		Exc,
		(log_error(Exc),fail)
	).

object_marker0(Obj,MarkerID,
		[ pose(Origin) | MarkerData ]) :-
	object_shape(Obj,MarkerID,Shape,Origin0,Material),
	object_origin(Origin0,Origin),
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
object_origin([Frame,Pos0,Rot0],[Frame,Pos1,Rot1]) :-
	(	is_list(Pos0) -> Pos1=Pos0
	;   atom(Pos0)    -> parse_number_list(Pos0,Pos1)
	;	var(Pos0)     -> Pos1 = [0,0,0]
	;	Pos1=Pos0
	),
	(	is_list(Rot0) -> Rot1=Rot0
	;	atom(Rot0)    -> parse_number_list(Rot0,Rot1)
	;	var(Rot0)     -> Rot1 = [0,0,0,1]
	;	Rot1=Rot0
	).

%
parse_number_list(Atom,NumberList) :-
	catch(term_to_atom(NumberList,Atom), _, fail),!.
parse_number_list(Atom,NumberList) :-
	atomic_list_concat(Elems, ' ', Atom),
	maplist(atom_number, Elems, NumberList).

%%
material_rgba(material(Material),[R,G,B,A]) :-
	member(rgba([R,G,B,A]),Material),
	ground([R,G,B,A]),
	!.

material_rgba(material(Material),[R,G,B,A]) :-
	member(rgba([R,G,B],A),Material),
	ground([R,G,B,A]),
	!.

material_rgba(material(Material),[R,G,B,1]) :-
	member(rgb([R,G,B]),Material),
	ground([R,G,B]),
	!.

material_rgba(_,[1,1,1,1]).
