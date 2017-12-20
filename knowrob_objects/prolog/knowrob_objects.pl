/** <module> Utilities for reasoning about objects

  Copyright (C) 2011-2014 Moritz Tenorth
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

  @author Moritz Tenorth
  @license BSD
*/

:- module(knowrob_objects,
    [
      current_object_pose/2,
      object_pose_at_time/3,
      object_pose_at_time/4,
      object_trajectory/4,
      object_distance/3,
      object_color/2,
      object_dimensions/4,
      object_mesh_path/2,
      object_assert_dimensions/4,
      object_assert_color/2,
      storagePlaceFor/2,
      storagePlaceForBecause/3,
      object_query/4,
      object_queries/2,
      comp_tf_pose/2,
      comp_depthOfObject/2,
      comp_widthOfObject/2,
      comp_heightOfObject/2
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_math')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('owl_parser')).

:- owl_parser:owl_parse('package://knowrob_objects/owl/knowrob_objects.owl').

:-  rdf_meta
    current_object_pose(r,-),
    object_pose_at_time(r,r,?),
    object_pose_at_time(r,r,?,?),
    object_trajectory(r,t,+,-),
    object_distance(r,r,-),
    object_dimensions(r, ?, ?, ?),
    object_color(r, ?),
    object_mesh_path(r, ?),
    object_assert_dimensions(r, +, +, +),
    object_assert_color(r, +),
    storagePlaceFor(r,r),
    storagePlaceForBecause(r,r,r),
    object_query(r,?,?,?),
    object_queries(r,?),
    comp_depthOfObject(r,t),
    comp_widthOfObject(r,t),
    comp_heightOfObject(r,t).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).

%% current_object_pose(+Obj:iri, -PoseTerm:list) is semidet
%
% Get the current pose of an object.
%
% @param Obj       Instance of SpatialThing
% @param PoseTerm  float[7] (translation, quaternion) or float[16] (row-based matrix elements)
% 
current_object_pose(Obj, [TX,TY,TZ,QX,QY,QZ,QW]) :- !,
  current_time(T),
  object_pose_at_time(Obj, T, pose([TX, TY, TZ], [QX,QY,QZ,QW])),!.

current_object_pose(Obj, PoseTerm) :-
  current_time(T),
  object_pose_at_time(Obj, T, mat(PoseTerm)),!.

%% object_pose_at_time(+Obj:iri, +Instant:float, ?PoseTerm:term) is semidet
%% object_pose_at_time(+Obj:iri, +Instant:float, ?PoseTerm:term, -Interval:list) is semidet
%
% Get the pose of an object at the specified time instant.
% PoseTerm is either
%    pose([float tx, ty, tz], [float qx, qy, qz, qw]]), or
%    mat([float m11 ... m44])
%
% @param Obj         Instance of SpatialThing
% @param Instant     The time instant (float or Timepoint:iri)
% @param PoseTerm    The pose term pose(..) or mat(..)
% @param Interval    Interval during which the pose holds in the form of [float start, end] or [float start]
% 
object_pose_at_time(Obj, Instant, PoseTerm) :-
  atom(Obj),
  object_pose_at_time(Obj, Instant, PoseTerm, _).

object_pose_at_time(Obj, _Instant, PoseTerm, [0.0]) :-
  atom(Obj),
  ( rdfs_individual_of(Obj, knowrob:'Pose') ;
    rdfs_individual_of(Obj, knowrob:'Matrix') ), !,
  pose_term(Obj, PoseTerm).

object_pose_at_time(Obj, Instant, PoseTerm, Interval) :-
  atom(Obj),
  holds(Obj, knowrob:pose, Pose, Interval),
  interval_during(Instant, Interval), !,
  pose_term(Pose, PoseTerm).

%% object_trajectory(+Obj, +Interval, +Dt, -Trajectory) is semidet
%
object_trajectory(Obj, Interval, num_samples(Count), Trajectory) :-
  interval(Interval, [Begin,End]),
  Dt is (End - Begin) / Count,
  object_trajectory(Obj, Interval, dt(Dt), Trajectory).

object_trajectory(_, [Begin,End], _, []) :- Begin > End, !.
object_trajectory(Obj, [Begin,End], dt(Dt), [X|Xs]) :-
  Begin =< End, Next is Begin + Dt,
  % TODO(daniel): this could yield many mongo queries. find more efficient way of sampling trajectories.
  object_pose_at_time(Obj, Begin, X),
  object_trajectory(Obj, [Next,End], dt(Dt), Xs).

%% object_distance(+A:iri, +B:iri, ?Distance:float) is semidet
% 
% Computes eucledean distance between A and B.
%
% @param A         Instance of SpatialThing
% @param B         Instance of SpatialThing
% @param Distance  The current distance between A and B
%
object_distance(A,B,D):-
  current_object_pose(A, [AX,AY,AZ,_,_,_,_]),
  current_object_pose(B, [BX,BY,BZ,_,_,_,_]),
  DX is AX - BX,
  DY is AY - BY,
  DZ is AZ - BZ,
  D is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)), !.

%% object_color(?Obj:iri, ?Col:list) is det
%
% Get the main color of the object.
% The color is returned as [float red, green, blue, alpha], on a scale of 0-1.
% If there is no color given for an object in the knowledge base,
% then a default [0.5, 0.5, 0.5, 1] is returned.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color(Obj, Col) :-
  holds(Obj, knowrob:mainColorOfObject, literal(type(xsd:string, ColAtom))),
  parse_vector(ColAtom, Col), !.
object_color(_Obj, [0.5, 0.5, 0.5, 1.0]).

%% object_assert_color(+Obj:iri, +Col:list) is det
%
% Assert object main color property.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_assert_color(Obj, [R,G,B]) :-
  object_assert_color(Obj, [R,G,B,1.0]), !.
object_assert_color(Obj, [R,G,B,A]) :-
  atomic_list_concat([R,G,B,A], ' ', ColRGBA),
  object_assert_color(Obj, ColRGBA), !.
object_assert_color(Obj, Col) :-
  atom(Col),
  rdf_assert(Obj, knowrob:mainColorOfObject, literal(type(xsd:string, Col))), !.

%% object_dimensions(?Obj:iri, ?Depth:float, ?Width:float, ?Height:float) is semidet
%
% True if Depth x Width x Height are (exactly) the extends of the bounding box of Obj.
% NOTE that we use ROS conventions here: Coordinate systems in ROS are
% always right-handed, with X forward, Y left, and Z up. 
%
% @param Obj    Instance of SpatialThing
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_dimensions(Obj, Depth, Width, Height) :-
  object_boundingBox(Obj, Depth, Width, Height),!.
  
object_dimensions(Obj, Depth, Width, Height) :-
  holds(Obj, knowrob:depthOfObject,  literal(type(_, Depth_))),  atom_number(Depth_,  Depth),
  holds(Obj, knowrob:widthOfObject,  literal(type(_, Width_))),  atom_number(Width_,  Width),
  holds(Obj, knowrob:heightOfObject, literal(type(_, Height_))), atom_number(Height_, Height), !.

object_dimensions(Obj, Depth, Width, Height) :-
  % The depth of a knob defaults to 3cm here. This information
  % should either be asserted somewhere else or be set as a property
  % when importing the semantic map.
  holds( Obj, knowrob:radius, literal(type(_, Radius_)) ),
  atom_number(Radius_, Radius),
  Width is 2 * Radius,
  Height is Width,
  Depth is Height.

object_boundingBox(Obj, Depth, Width, Height) :-
  holds(Obj, knowrob:boundingBoxSize, literal(type(xsd:string, ScaleVector))),
  parse_vector(ScaleVector, [Depth, Width, Height]),!.

%% comp_depthOfObject(+Obj:iri, ?Depth:term) is semidet
%% comp_widthOfObject(+Obj:iri, ?Depth:term) is semidet
%% comp_heightOfObject(+Obj:iri, ?Depth:term) is semidet
%
% Computes dimension components from boundingBox properties of an object.
%
comp_depthOfObject(Obj, literal(type('http://www.w3.org/2001/XMLSchema#float', Depth))) :-
  object_boundingBox(Obj, Val, _, _), atom_number(Depth, Val).
comp_widthOfObject(Obj, literal(type('http://www.w3.org/2001/XMLSchema#float', Width))) :-
  object_boundingBox(Obj, _, Val, _), atom_number(Width, Val).
comp_heightOfObject(Obj, literal(type('http://www.w3.org/2001/XMLSchema#float', Height))) :-
  object_boundingBox(Obj, _, _, Val), atom_number(Height, Val).

%% object_assert_dimensions(+Obj:iri, +Depth:float, +Width:float, +Height:float) is det
%
% Assert object dimension properties.
%
% @param Obj    Instance of a subclass of EnduringThing-Localized
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_assert_dimensions(Obj, Depth, Width, Height) :-
  atomic_list_concat([Depth, Width, Height], ' ', V),
  rdf_assert(Obj, knowrob:boundingBoxSize, literal(type(xsd:string, V))).

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det.
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) :-
  holds(Obj, knowrob:pathToCadModel, literal(type(xsd:string, FilePath))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Pose from TF

%% comp_tf_pose
comp_tf_pose(Obj, Pose) :-
  current_time(Instant),
  comp_tf_pose_at_time(Obj, Pose, Instant).

%% comp_tf_pose_at_time
comp_tf_pose_at_time(Obj, Pose, Instant) :-
  rdf_has(Obj, knowrob:frameName, ObjFrame),
  current_time(Now),
  ( var(Instant) -> Instant = Now ;   20 > abs(Now - Instant) ),
  map_frame_name(MapFrameName),
  tf_lookup_transform(MapFrameName, ObjFrame, PoseTerm),
  create_pose(PoseTerm, Pose), !.

knowrob_temporal:holds(Obj, 'http://knowrob.org/kb/knowrob.owl#pose', Pose, [Begin,_]) :-
  comp_tf_pose_at_time(Obj, Pose, Begin).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Reasoning about function and storage location of objects
% TODO: add utility rules resoning about object function (i.e., how to use it for action?)

%% storagePlaceFor(St, ObjT) is nondet.
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% 
storagePlaceFor(St, ObjT) :-
  storagePlaceForBecause(St, ObjT, _).

%% storagePlaceForBecause(St, ObjType, ObjT) is nondet.
%
% Computes the nominal storage location of an object based on assertions for
% typePrimaryFunction-containerFor for any of its superclasses. For example,
% a Refrigerator is asserted as ...-containerFor perishable items, so
% instances of Refrigerator will therefore be returned for e.g. dairy products
% or meat products.
%
% In addition to the storage place, this predicate further returns the superclass
% of Obj for which this information is asserted (e.g. Perishable)
%
% @param St       Instance of a knowrob:'StorageConstruct'
% @param Obj      Object class or instance
% @param ObjType  Class for which information about the storage place has been asserted
%

% two instances
storagePlaceForBecause(St, Obj, ObjT) :-
  owl_subclass_of(StT, knowrob:'StorageConstruct'),
  owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-containerFor', some_values_from(ObjT))),
  owl_individual_of(Obj, ObjT),
  owl_individual_of(St, StT).

% obj type, storage instance
storagePlaceForBecause(St, ObjType, ObjT) :-
  owl_subclass_of(StT, knowrob:'StorageConstruct'),
  owl_restriction_on(StT, restriction(knowrob:'typePrimaryFunction-containerFor', some_values_from(ObjT))),
  owl_individual_of(St, StT),
  owl_subclass_of(ObjType, ObjT).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Asking queries about objects.

%% object_queries(+Obj:iri, -Queries:list) is det
%
% gather facts about queries that can be asked about an object.
% queries are represented as [string category, string title, string query].
% Category and title are primary used for displaying possible queries
% to the user.
%
% @param Obj the object name
% @param Queries list of queries that can be asked about Obj
%
object_queries(Obj, Queries) :-
  findall([Category,Title,Query],
          object_query(Obj,Category,Title,Query),
          QueriesUnsorted),
  sort(QueriesUnsorted, Queries).

%% object_query(+Obj:iri, ?QueryGroup:atom, ?QueryTitle:atom, ?Query:atom) is det
%
% True for objects Obj for which a query exists belonging to the group QueryGroup
% and labeled with QueryTitle.
%
% @param Obj the object name
% @param QueryGroup category of query
% @param QueryTitle name of the query
% @param Query the Prolog-encoded query string
%
object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific individuals
  rdf_has(QueryIndividual, knowrob:'queryAbout', Obj),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).

object_query(Obj, QueryGroup, QueryTitle, Query) :-
  atom(Obj),
  % queries about specific types
  rdfs_individual_of(Obj, IndividualClass),
  % FIXME: queryAbout some Class is non OWL! use restrictions instead!
  rdf_has(QueryIndividual, knowrob:'queryAbout', IndividualClass),
  rdf_has(QueryIndividual, knowrob:'groupName', literal(type(_,QueryGroup))),
  rdf_has(QueryIndividual, knowrob:'queryName', literal(type(_,QueryTitle))),
  rdf_has(QueryIndividual, knowrob:'queryString', literal(type(_,QueryTail))),
  atomic_list_concat(['Individual=''', Obj, ''''], '', QueryHead),
  atomic_list_concat([QueryHead,QueryTail], ', ', Query).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Utility predicates

% quaternion and position
pose_term(Pose, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  position_to_list(Pose, [X,Y,Z]),
  quaternion_to_list(Pose, [QX,QY,QZ,QW]), !.
pose_term(Pose, mat(Mat)) :-
  position_to_list(Pose, [X,Y,Z]),
  quaternion_to_list(Pose, [QX,QY,QZ,QW]),
  matrix([X,Y,Z], [QX,QY,QZ,QW], Mat), !.

% list of transfoamtion matrix elements
pose_term(Matrix, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  is_list(Matrix), !,
  matrix_rotation(Matrix, [QX,QY,QZ,QW]),
  matrix_translation(Matrix, [X,Y,Z]).
pose_term(Matrix, mat(Matrix)) :- is_list(Matrix), !.

% transformation matrix individual
pose_term(Pose, pose([X,Y,Z], [QX,QY,QZ,QW])) :-
  rdfs_individual_of(Pose, knowrob:'Matrix'),
  rotmat_to_list(Pose, Matrix),
  matrix_rotation(Matrix, [QX,QY,QZ,QW]),
  matrix_translation(Matrix, [X,Y,Z]), !.
pose_term(Pose, mat(Matrix)) :-
  rdfs_individual_of(Pose, knowrob:'Matrix'),
  rotmat_to_list(Pose, Matrix), !.
