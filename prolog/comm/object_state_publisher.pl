/*
  Copyright (C) 2017 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

:- module(object_state_publisher,
    [
      mark_dirty_objects/1
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/model/Object')).
:- use_module(library('knowrob/memory/object_pose'), [
    object_pose_data/3,
    current_object_pose/2
]).
:- use_module(library('knowrob/lang/ask'), [
    kb_type_of/2,
    kb_subclass_of/2
]).

:-  rdf_meta
    mark_dirty_objects(t).

:- use_foreign_library('libobject_state_publisher.so').

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
  kb_type_of(Obj,Type),
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

%object_state_shape_(Iri,0) :- kb_subclass_of(Iri,ease_obj:'Arrow'),!.
object_state_shape_(Iri,1) :- kb_subclass_of(Iri,ease_obj:'BoxShape'),!.
object_state_shape_(Iri,2) :- kb_subclass_of(Iri,ease_obj:'SphereShape'),!.
object_state_shape_(Iri,3) :- kb_subclass_of(Iri,ease_obj:'CylinderShape'),!.
