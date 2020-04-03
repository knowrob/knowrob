/*
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
*/

:- module('knowrob/model/Object',
    [
      kb_is_object/1,
      object_assert/3,
      object_lifetime/2,
      object_frame_name/2,
      object_color/2,
      object_quality/3,
      object_feature/2,
      object_feature/3,
      object_disposition/2,
      object_disposition/3,
      object_localization/2,
      object_dimensions/4,
      object_mesh_path/2,
      object_assert_dimensions/4,
      object_assert_color/2,
      object_assert_frame_name/1,
      object_set_lifetime_begin/2,
      object_set_lifetime_end/2,
      object_is_alive/1,
      object_shape_type/2,
      %%
      disposition_trigger_type/2
    ]).
/** <module> Interface to RDF model of (physical) objects.

*PhysicalObject* is defined as any physical, social, or mental object, or a substance. Following DOLCE Full, objects are always participating in some event (at least their own life), and are spatially located.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/lang/ask')).
:- use_module(library('knowrob/lang/tell')).

:-  rdf_meta
    kb_is_object(r),
    object_lifetime(r,?),
    object_assert(r,r,+),
    object_dimensions(r, ?, ?, ?),
    object_color(r, ?),
    object_localization(r,r),
    object_frame_name(r,?),
    object_mesh_path(r, ?),
    object_assert_dimensions(r, +, +, +),
    object_assert_color(r, +),
    object_assert_frame_name(r),
    object_feature(r,r),
    object_feature(r,r,r),
    object_quality(r,r,r),
    object_disposition(r,r),
    object_disposition(r,r,r),
    object_is_alive(r),
    object_set_lifetime_begin(r,+),
    object_set_lifetime_end(r,+),
    object_aspect_(r,r,r,r),
    disposition_trigger_type(r,r).

%% kb_is_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Object'.
%
% @param Entity An entity IRI.
%
kb_is_object(Entity) :-
  kb_type_of(Entity,dul:'Object'),!.

object_assert_frame_name(Obj) :-
  rdf_split_url(_, ObjName, Obj),
  ( rdf_has(Obj, knowrob:'frameName', _) ;
    kb_assert(Obj, knowrob:'frameName', literal(ObjName)) ), !.

%% object_assert(+ObjType:iri, -Obj:iri, +Graph) is det.
%
% Asserts a new object to the named RDF graph.
%
% @param ObjType the type of the new object
% @param Obj the object asserted
%
object_assert(ObjType, Obj, Graph) :-
  kb_create(ObjType, Obj, _{graph:Graph}),
  % set TF frame to object name
  rdf_split_url(_, ObjName, Obj),
  kb_assert(Obj, knowrob:'frameName', literal(ObjName)).

%%
object_is_alive(Obj) :-
  object_lifetime_(Obj,LT),
  \+ rdf_has(LT,ease:hasIntervalEnd,_).

%%
object_lifetime(Obj,Interval) :-
  object_lifetime_(Obj,LT),
  inteval(LT,Interval).

object_lifetime_(Obj,LT) :-
  rdf_has(Obj,dul:hasTimeInterval,LT),!.

object_lifetime_(Obj,LT) :-
  once(rdf(Obj,rdf:type,_,G)),
  kb_create(dul:'TimeInterval',LT,_{graph:G}),
  rdf_assert(Obj,dul:hasTimeInterval,LT,G).

%%
object_set_lifetime_begin(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalBegin,Stamp,_{graph:G}).

%%
object_set_lifetime_end(Obj,Stamp) :-
  once(rdf(Obj,rdf:type,_,G)),
  object_lifetime_(Obj,LT),
  kb_assert(LT,ease:hasIntervalEnd,Stamp,_{graph:G}).

%%
object_localization(Obj,Obj) :-
  atom(Obj),
  rdfs_individual_of(Obj,ease_obj:'Localization'),!.
object_localization(Obj,Obj) :-
  atom(Obj),
  kb_triple(Obj, ease_obj:hasSpaceRegion, _),!.
object_localization(Obj,Loc) :-
  object_localization_(Obj,Loc).

%% object_frame_name(+Obj:iri, ?FrameName:atom) is det
%
% True if FrameName is the name of origin frame of the object.
% Fallback is to use the IRI suffix of Obj.
% 
% @param Obj Instance of SpatialThing
% @param FrameName The frame name
%
object_frame_name(Obj,FrameName) :-
  kb_triple(Obj,knowrob:frameName,FrameName), !.

object_frame_name(Localization,FrameName) :-
  atom(Localization),
  rdf_has(Obj,ease_obj:hasLocalization,Localization),
  object_frame_name(Obj,FrameName), !.

object_frame_name(Obj,FrameName) :-
  atom(Obj),
  rdf_split_url(_,FrameName,Obj).

%% object_color(?Obj:iri, ?Col:list) is det
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue, alpha], on a scale of 0-1.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color(Obj, [R,G,B,A]) :-
  kb_triple(Obj,ease_obj:hasColor,Color),
  kb_triple(Color,dul:hasRegion,ColorRegion),
  kb_triple(ColorRegion, ease_obj:hasRGBValue, [R,G,B|Rest]),
  ( Rest=[A] ; A is 1.0 ),!.

object_color(Obj, [R,G,B,A]) :-
  kb_triple(Obj, ease_obj:hasRGBValue, [R,G,B|Rest]),
  ( Rest=[A] ; A is 1.0 ),!.

%% object_assert_color(+Obj:iri, +Col:list) is det
%
% Assert object main color property.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_assert_color(Obj,ColorValue) :-
  once(rdf(Obj,_,_,G)),
  object_color_(Obj,Color),
  %%
  kb_create(ease_obj:'ColorRegion',ColorRegion,_{graph: G}),
  kb_assert(ColorRegion,ease_obj:hasRGBValue,ColorValue),
  %%
  kb_retract(Color,dul:hasRegion,_),
  kb_assert(Color,dul:hasRegion,ColorRegion).

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
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_triple(Shape,dul:hasRegion,ShapeRegion),
  shape_bbox(ShapeRegion, Depth, Width, Height),!.

object_dimensions(Obj, Depth, Width, Height) :-
  shape_bbox(Obj, Depth, Width, Height),!.

%% object_shape_type(?Obj:iri, ?Shape:iri) is semidet
%
object_shape_type(Obj,ShapeType) :-
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_type_of(Shape,ShapeType).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) :-
  kb_triple(ShapeRegion, ease_obj:hasDepth, Depth),
  kb_triple(ShapeRegion, ease_obj:hasWidth, Width),
  kb_triple(ShapeRegion, ease_obj:hasHeight, Height), !.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) :-
  kb_type_of(ShapeRegion,ease_obj:'SphereShape'),
  kb_triple(ShapeRegion, ease_obj:hasRadius, Radius), !,
  number(Radius),
  Diameter is 2 * Radius.

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
  once(rdf(Obj,_,_,G)),
  object_shape_(Obj, Shape),
  % create a new region
  kb_create(ease_obj:'BoxShape',ShapeRegion,_{graph: G}),
  kb_assert(ShapeRegion, ease_obj:hasDepth, Depth),
  kb_assert(ShapeRegion, ease_obj:hasWidth, Width),
  kb_assert(ShapeRegion, ease_obj:hasHeight, Height),
  % unlink previous box shapes
  forall((
    kb_triple(Shape,dul:hasRegion,X),
    kb_type_of(X,ease_obj:'BoxShape')),
    kb_retract(Shape,dul:hasRegion,X)
  ),
  kb_assert(Shape,dul:hasRegion,ShapeRegion).

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) :-
  kb_triple(Obj,ease_obj:hasShape,Shape),
  kb_triple(Shape,dul:hasRegion,ShapeRegion),
  kb_triple(ShapeRegion,ease_obj:hasFilePath,FilePath),!.

object_mesh_path(Obj, FilePath) :-
  kb_triple(Obj,ease_obj:hasFilePath,FilePath),!.
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object qualities

%%
object_aspect_(Object,Relation,AspectType,Aspect) :-
  atom(Object),
  rdfs_individual_of(Object,dul:'Object'),
  (( 
    kb_triple(Object,Relation,Aspect),
    kb_type_of(Aspect,AspectType)
  ) ; (
    once(rdf(Object,_,_,G)),
    kb_create(AspectType,Aspect,_{graph: G}),
    kb_assert(Object,Relation,Aspect),
    % FIXME: below is a bit hacked, shouldn't e.g. owl_individual_of
    %         draw restrictions from related things?
    %       - this is problematic when AspectType is something general and
    %         there are multiple aspects with that type!
    %         probably best to take max cardinality and disjointness into account.
    forall((
      property_cardinality(Object,Relation,X,Min,_),
      Min>0,
      once(owl_subclass_of(X,AspectType))
    ), (
      kb_assert(Aspect,rdf:type,X)
    ))
  )), !.

%%
object_color_(Obj,Color) :-
  object_aspect_(Obj, ease_obj:hasColor, ease_obj:'Color', Color).
%%
object_shape_(Obj,Shape) :-
  object_aspect_(Obj, ease_obj:hasShape, ease_obj:'Shape', Shape).
%%
object_localization_(Obj,Localization) :-
  object_aspect_(Obj, ease_obj:hasLocalization, ease_obj:'Localization', Localization).

object_quality(Obj, QualityType, Quality) :-
  object_aspect_(Obj, dul:hasQuality, QualityType, Quality).
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object dispositions

%% object_disposition(?Obj:iri, ?Disposition:iri) is nondet.
%
% Relates an object to one of its dispositions.
%
% @param Obj           An individual of type dul:'Object'.
% @param Disposition   An individual of type ease_obj:'Disposition'.
%
object_disposition(Obj, Disposition) :-
  atom(Disposition),!,
  rdf_has(Obj,ease_obj:hasDisposition,Disposition).
object_disposition(Obj, Disposition) :-
  kb_some(Obj,ease_obj:hasDisposition,DispositionType),
  object_aspect_(Obj,ease_obj:hasDisposition,DispositionType,Disposition).

%% object_disposition(?Obj:iri, ?Disposition:iri, +DispositionType:iri) is nondet.
%
% Relates an object to one of its dispositions that is an instance
% of given disposition type.
%
% @param Obj               An individual of type dul:'Object'.
% @param Disposition       An individual of type ease_obj:'Disposition'.
% @param DispositionType   A sub-class of ease_obj:'Disposition'.
%
object_disposition(Obj, Disposition, DispositionType) :-
  ground(DispositionType),!,
  object_aspect_(Obj,ease_obj:hasDisposition,DispositionType,Disposition).
object_disposition(Obj, Disposition, DispositionType) :-
  object_disposition(Obj, Disposition),
  % get the Disposition type
  once((
    kb_type_of(Disposition,DispositionType),
    owl_subclass_of(DispositionType,ease_obj:'Disposition')
  )).

%% disposition_trigger_type(?Disposition:iri, ?TriggerType:iri) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition   An individual of type ease_obj:'Disposition'.
% @param TriggerType   A sub-class of dul:'Object'.
%
disposition_trigger_type(Disposition,TriggerType) :-
  property_range(Disposition,ease_obj:affordsTrigger,TriggerRole),!,
  property_range(TriggerRole,dul:classifies,ClassifiedType),
  ( var(TriggerType) -> TriggerType=ClassifiedType ;
    once(owl_subclass_of(TriggerType,ClassifiedType))
  ),
  rdfs_subclass_of(TriggerType,dul:'Object'),!.
  
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % Object features

%%
object_feature(Obj, Feature) :-
  atom(Feature),!,
  rdf_has(Obj,ease_obj:hasFeature,Feature).
object_feature(Obj, Feature) :-
  kb_some(Obj,ease_obj:hasFeature,FeatureType),
  object_aspect_(Obj,ease_obj:hasFeature,FeatureType,Feature).

%%
object_feature(Obj, Feature, FeatureType) :-
  ground(FeatureType),!,
  object_aspect_(Obj,ease_obj:hasFeature,FeatureType,Feature).
object_feature(Obj, Feature, FeatureType) :-
  object_feature(Obj, Feature),
  % get the feature type
  once((
    kb_type_of(Feature,FeatureType),
    owl_subclass_of(FeatureType,ease_obj:'Feature')
  )).
