:- module(model_EASE_OBJ,
    [ %% Life Time
      %is_alive(r),
      %% Qualities
      is_physical_quality(r),
      is_social_quality(r),
      is_intrinsic(r),
      is_extrinsic(r),
      object_localization(r,r),
      object_color_rgb(r,?),
      object_dimensions(r,?,?,?),
      object_mesh_path(r,?),
      object_shape_type(r,r),
      %% Features
      is_feature(r),
      object_feature(r,r),
      object_feature_type(r,r,r),
      %% Affordances
      is_affordance(r),
      is_disposition(r),
      has_disposition(r,r),
      has_disposition_type(r,r,r),
      disposition_trigger_type(r,r),
      %% Roles & Parameters
      is_patient(r),
      is_instrument(r),
      is_location(r),
      is_destination(r),
      is_origin(r)
    ]).
/** <module> Interface predicates for EASE-OBJ model.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).
:- use_module(library('model/OWL'),
    [ is_individual/1 ]).
:- use_module(library('model/DUL/Object'),
    [ has_role/2,
      has_object_type/2,
      has_quality_type/2
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).
:- use_module(library('db/scope'),
    [ universal_scope/1 ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

:- tripledb_load('http://www.ease-crc.org/ont/EASE-OBJ.owl',
    [ graph(tbox),
      namespace(ease_obj)
    ]).

		 /*******************************
		 *	    LIFE TIME		*
		 *******************************/

%% is_alive(+Obj) is det.
%
%is_alive(Obj) ?>
  %% FIXME: knowrob namespace
  %holds(Obj,knowrob:hasLifetime,Evt),
  %occurs(Evt).

%is_alive(Obj) +>
  %{ holds(Obj,knowrob:hasLifetime,Evt) },
  %occurs(Evt).

		 /*******************************
		 *	    QUALITIES		*
		 *******************************/

%% is_physical_quality(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'PhysicalQuality'.
%
% @param Entity An entity IRI.
%
is_physical_quality(Entity) ?+>
  has_type(Entity, ease_obj:'PhysicalQuality').

%% is_social_quality(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'SocialQuality'.
%
% @param Entity An entity IRI.
%
is_social_quality(Entity) ?+>
  has_type(Entity, ease_obj:'SocialQuality').

%% is_intrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Intrinsic'.
%
% @param Entity An entity IRI.
%
is_intrinsic(Entity) ?+>
  has_type(Entity, ease_obj:'Intrinsic').

%% is_extrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Extrinsic'.
%
% @param Entity An entity IRI.
%
is_extrinsic(Entity) ?+>
  has_type(Entity, ease_obj:'Extrinsic').

%% object_localization(?Obj, ?Loc) is nondet.
%
% Relates an object to its localization quality.
%
% @param Obj object resource
% @param Loc localization quality
%
object_localization(Obj,Loc) ?+>
  holds(Obj,ease_obj:hasLocalization,Loc).

%% object_color_rgb(?Obj, ?Col) is nondet.
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue], on a scale of 0-1.
%
% @param Obj object resource
% @param Col rgb color data
% 
object_color_rgb(Obj,[R,G,B]) ?>
  holds(Obj,ease_obj:hasColor,Color),
  holds(Color,dul:hasRegion,Region),
  holds(Region,ease_obj:hasRGBValue,[R,G,B]),
  { ! }.

object_color_rgb(Obj, [R,G,B]) ?>
  holds(Obj,ease_obj:hasRGBValue,[R,G,B]),
  { ! }.
  
object_color_rgb(Obj,[R,G,B]) +>
  % get the color quality
  { holds(Obj,ease_obj:hasColor,Color) },
  % create a new region
  { universal_scope(US),
    tell([ has_type(Region,ease_obj:'ColorRegion'),
           holds(Region,ease_obj:hasRGBValue,[R,G,B])
         ],US)
  },
  % update the region of the color quality
  holds(Color,dul:hasRegion,update(Region)),
  notify(object_changed(Obj)).

%% object_shape_type(?Obj, ?ShapeType) is nondet.
%
% Relates an object to the type of its shape(s).
% An object may have multiple shapes associated that
% provide different level of detail.
%
% @param Obj Object resource
% @param ShapeType IRI of shape type
%
object_shape_type(Obj, ShapeType) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  has_type(ShapeRegion,ShapeType).

%% object_dimensions(?Obj, ?Depth, ?Width, ?Height) is nondet.
%
% True if Depth x Width x Height are (exactly) the extends of the bounding box of Obj.
% NOTE that we use ROS conventions here: Coordinate systems in ROS are
% always right-handed, with X forward, Y left, and Z up. 
%
% @param Obj    Object resource
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_dimensions(Obj, Depth, Width, Height) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  shape_bbox(ShapeRegion,Depth,Width,Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) ?>
  shape_bbox(Obj, Depth, Width, Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) +>
  % get the shape quality
  { holds(Obj,ease_obj:hasShape,Shape) },
  is_individual(ShapeRegion),
  % create a new region
  % TODO: replace any other BoxShape region
  { universal_scope(US),
    tell([ has_type(ShapeRegion,ease_obj:'BoxShape'),
           holds(ShapeRegion, ease_obj:hasDepth,  Depth),
           holds(ShapeRegion, ease_obj:hasWidth,  Width),
           holds(ShapeRegion, ease_obj:hasHeight, Height)
         ],US)
  },
  holds(Shape,dul:hasRegion,ShapeRegion),
  notify(object_changed(Obj)).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) ?>
  holds(ShapeRegion, ease_obj:hasDepth, Depth),
  holds(ShapeRegion, ease_obj:hasWidth, Width),
  holds(ShapeRegion, ease_obj:hasHeight, Height),
  { ! }.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) ?>
  %holds(ShapeRegion,rdf:type,ease_obj:'SphereShape' ),
  holds(ShapeRegion, ease_obj:hasRadius, Radius),
  { Diameter is 2 * Radius },
  { ! }.

%% object_mesh_path(?Obj, -FilePath) is nondet.
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj      Object resource
% @param FilePath the file path
%
object_mesh_path(Obj, FilePath) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  holds(ShapeRegion,ease_obj:hasFilePath,FilePath),
  { ! }.

object_mesh_path(Obj, FilePath) ?>
  holds(Obj,ease_obj:hasFilePath,FilePath ),
  { ! }.

object_mesh_path(Obj, FilePath) +>
  { holds(Obj,ease_obj:hasShape,Shape) },
  % create a new region
  { universal_scope(US),
    tell([ has_type(ShapeRegion,ease_obj:'MeshShape'),
           holds(ShapeRegion,ease_obj:hasFilePath,FilePath)
         ],US)
  },
  % assign the region to the shape quality
  holds(Shape,dul:hasRegion,ShapeRegion),
  notify(object_changed(Obj)).

		 /*******************************
		 *	    FEATURES		*
		 *******************************/

%% is_feature(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Feature'.
%
% @param Entity An entity IRI.
%
is_feature(Entity) ?+>
  has_type(Entity,ease_obj:'Feature').

%% object_feature(+Obj, ?Feature) is nondet.
%
% Associates an object resource to features it hosts.
%
% @param Obj      Object resource
% @param Feature  Feature resource
%
object_feature(Obj, Feature) ?+>
  holds(Obj,ease_obj:hasFeature,Feature).

%% object_feature(?Obj, ?Feature, ?FeatureType) is nondet.
%
% Same as object_feature/2 but additionally unifies
% the feature type.
%
% @param Obj      Object resource
% @param Feature  Feature resource
% @param FeatureType  Class resource
%
object_feature_type(Obj, Feature, FeatureType) ?>
  object_feature(Obj,Feature),
  has_object_type(Feature,FeatureType).

		 /*******************************
		 *	    AFFORDANCES		*
		 *******************************/

%% is_affordance(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Affordance'.
%
% @param Entity An entity IRI.
%
is_affordance(Entity) ?+>
  has_type(Entity,ease_obj:'Affordance').

%% is_disposition(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Disposition'.
%
% @param Entity An entity IRI.
%
is_disposition(Entity) ?+>
  has_type(Entity,ease_obj:'Disposition').

%% has_disposition(?Obj, ?Disposition) is nondet.
%
% Relates an object to its dispositions.
%
% @param Obj          Object resource
% @param Disposition  Disposition resource
%
has_disposition(Obj, Disposition) ?+>
  holds(Obj,ease_obj:hasDisposition,Disposition).

%% has_disposition(?Obj:iri, ?Disposition:iri, +DispositionType:iri) is nondet.
%
% Relates an object to one of its dispositions that is an instance
% of given disposition type.
%
% @param Obj               Object resource
% @param Disposition       Disposition resource
% @param DispositionType   Class resource
%
has_disposition_type(Obj, Disposition, DispositionType) ?>
  holds(Obj,ease_obj:hasDisposition,Disposition),
  has_quality_type(Disposition,DispositionType).

%% disposition_trigger_type(?Disposition, ?TriggerType) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition  Disposition resource
% @param TriggerType  Class resource
%
disposition_trigger_type(Disposition,TriggerType) ?>
  holds(Disposition, ease_obj:affordsTrigger, only(TriggerRole)),
  subclass_of(TriggerRole, only(dul:classifies,TriggerType)).

		 /*******************************
		 *	    ROLES & PARAMETERS		*
		 *******************************/

%% is_patient(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Patient'.
%
% @param Entity An entity IRI.
%
is_patient(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Patient').

%% is_instrument(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Instrument'.
%
% @param Entity An entity IRI.
%
is_instrument(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Instrument').

%% is_location(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Location'.
%
% @param Entity An entity IRI.
%
is_location(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Location').

%% is_destination(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Destination'.
%
% @param Entity An entity IRI.
%
is_destination(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Destination').

%% is_origin(?Entity) is nondet.
%
% True iff Entity is an instance of ease_obj:'Origin'.
%
% @param Entity An entity IRI.
%
is_origin(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Origin').
