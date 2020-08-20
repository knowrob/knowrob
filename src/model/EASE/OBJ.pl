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
      object_shape(r,-,-),
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
:- use_module(library('db/scope'),
    [ universal_scope/1 ]).
:- use_module(library('comm/notify'),
    [ notify/1 ]).

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
% True iff Entity is an instance of soma:'PhysicalQuality'.
%
% @param Entity An entity IRI.
%
is_physical_quality(Entity) ?+>
  has_type(Entity, soma:'PhysicalQuality').

%% is_social_quality(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'SocialQuality'.
%
% @param Entity An entity IRI.
%
is_social_quality(Entity) ?+>
  has_type(Entity, soma:'SocialQuality').

%% is_intrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Intrinsic'.
%
% @param Entity An entity IRI.
%
is_intrinsic(Entity) ?+>
  has_type(Entity, soma:'Intrinsic').

%% is_extrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Extrinsic'.
%
% @param Entity An entity IRI.
%
is_extrinsic(Entity) ?+>
  has_type(Entity, soma:'Extrinsic').

%% object_localization(?Obj, ?Loc) is nondet.
%
% Relates an object to its localization quality.
%
% @param Obj object resource
% @param Loc localization quality
%
object_localization(Obj,Loc) ?+>
  holds(Obj,soma:hasLocalization,Loc).

%% object_color_rgb(?Obj, ?Col) is nondet.
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue], on a scale of 0-1.
%
% @param Obj object resource
% @param Col rgb color data
% 
object_color_rgb(Obj,[R,G,B]) ?>
  holds(Obj,soma:hasColor,Color),
  holds(Color,dul:hasRegion,Region),
  holds(Region,soma:hasRGBValue,[R,G,B]),
  { ! }.

object_color_rgb(Obj, [R,G,B]) ?>
  holds(Obj,soma:hasRGBValue,[R,G,B]),
  { ! }.
  
object_color_rgb(Obj,[R,G,B]) +>
  % get the color quality
  { holds(Obj,soma:hasColor,Color) },
  % create a new region
  { universal_scope(US),
    tell([ has_type(Region,soma:'ColorRegion'),
           holds(Region,soma:hasRGBValue,[R,G,B])
         ],US)
  },
  % update the region of the color quality
  update(holds(Color,dul:hasRegion,Region)),
  notify(object_changed(Obj)).

%% object_shape(?Obj,?ShapeTerm,?ShapeOrigin) is nondet.
%
% Relates objects to shapes and their origin (usually a pose relative to the object).
% The shape is represented as a Prolog term that encodes the shape type
% and its geometrical properties.
%
% ShapeTerm may be one of:
% - mesh(File,Scale)
% - box(X,Y,Z)
% - cylinder(Radius,Length)
% - sphere(Radius)
%
% ShapeOrigin is a list of frame-position-quaternion.
%
% @Obj IRI atom
% @ShapeTerm A shape term
% @ShapeOrigin The origin of the shape
%
object_shape(Obj,ShapeTerm,[Frame,Pos,Rot]) ?>
	triple(Obj,soma:hasShape,Shape),
	triple(Shape,dul:hasRegion,ShapeRegion),
	rdf_split_url(_,Frame,Obj),
	{ shape_data(ShapeRegion,ShapeData),
	  shape_origin(ShapeRegion,[Pos,Rot])
	}.

%%
shape_data(ShapeRegion,mesh(File,Scale)) :-
	holds(ShapeRegion,soma:hasFilePath,File),
	shape_scale(ShapeRegion,Scale),
	!.

shape_data(ShapeRegion,box(X,Y,Z)) :-
	triple(ShapeRegion, soma:hasWidth,  X),
	triple(ShapeRegion, soma:hasHeight, Y),
	triple(ShapeRegion, soma:hasDepth,  Z),
	!.

shape_data(ShapeRegion,cylinder(Radius,Length)) :-
	triple(ShapeRegion, soma:hasLength, Length),
	triple(ShapeRegion, soma:hasRadius, Radius),
	!.

shape_data(ShapeRegion,sphere(Radius)) :-
	triple(ShapeRegion, soma:hasRadius, Radius),
	!.

%%
shape_scale(ShapeRegion,[X,Y,Z]) :-
	% TODO: knowrob namespace should not be used here
	triple(ShapeRegion, 'http://knowrob.org/kb/knowrob.owl#hasXScale', X),
	triple(ShapeRegion, 'http://knowrob.org/kb/knowrob.owl#hasYScale', Y),
	triple(ShapeRegion, 'http://knowrob.org/kb/knowrob.owl#hasZScale', Z).
shape_scale(_,[1,1,1]).

%%
shape_origin(ShapeRegion,[Pos,Rot]) :-
	% TODO: urdf namespace should not be used here
	triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin),
	triple(Origin, soma:hasPositionVector, term(Pos)),
	triple(Origin, soma:hasOrientationVector, term(Rot)),
	!.
get_origin_(_,[[0,0,0],[0,0,0,1]]).

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
  holds(Obj,soma:hasShape,Shape),
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
  holds(Obj,soma:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  shape_bbox(ShapeRegion,Depth,Width,Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) ?>
  shape_bbox(Obj, Depth, Width, Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) +>
  % get the shape quality
  { holds(Obj,soma:hasShape,Shape) },
  is_individual(ShapeRegion),
  % create a new region
  % TODO: replace any other BoxShape region
  { universal_scope(US),
    tell([ has_type(ShapeRegion,soma:'BoxShape'),
           holds(ShapeRegion, soma:hasDepth,  Depth),
           holds(ShapeRegion, soma:hasWidth,  Width),
           holds(ShapeRegion, soma:hasHeight, Height)
         ],US)
  },
  holds(Shape,dul:hasRegion,ShapeRegion),
  notify(object_changed(Obj)).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) ?>
  holds(ShapeRegion, soma:hasDepth, Depth),
  holds(ShapeRegion, soma:hasWidth, Width),
  holds(ShapeRegion, soma:hasHeight, Height),
  { ! }.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) ?>
  %holds(ShapeRegion,rdf:type,soma:'SphereShape' ),
  holds(ShapeRegion, soma:hasRadius, Radius),
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
  holds(Obj,soma:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  holds(ShapeRegion,soma:hasFilePath,FilePath),
  { ! }.

object_mesh_path(Obj, FilePath) ?>
  holds(Obj,soma:hasFilePath,FilePath ),
  { ! }.

object_mesh_path(Obj, FilePath) +>
  { holds(Obj,soma:hasShape,Shape) },
  % create a new region
  { universal_scope(US),
    tell([ has_type(ShapeRegion,soma:'MeshShape'),
           holds(ShapeRegion,soma:hasFilePath,FilePath)
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
% True iff Entity is an instance of soma:'Feature'.
%
% @param Entity An entity IRI.
%
is_feature(Entity) ?+>
  has_type(Entity,soma:'Feature').

%% object_feature(+Obj, ?Feature) is nondet.
%
% Associates an object resource to features it hosts.
%
% @param Obj      Object resource
% @param Feature  Feature resource
%
object_feature(Obj, Feature) ?+>
  holds(Obj,soma:hasFeature,Feature).

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
% True iff Entity is an instance of soma:'Affordance'.
%
% @param Entity An entity IRI.
%
is_affordance(Entity) ?+>
  has_type(Entity,soma:'Affordance').

%% is_disposition(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Disposition'.
%
% @param Entity An entity IRI.
%
is_disposition(Entity) ?+>
  has_type(Entity,soma:'Disposition').

%% has_disposition(?Obj, ?Disposition) is nondet.
%
% Relates an object to its dispositions.
%
% @param Obj          Object resource
% @param Disposition  Disposition resource
%
has_disposition(Obj, Disposition) ?+>
  holds(Obj,soma:hasDisposition,Disposition).

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
  holds(Obj,soma:hasDisposition,Disposition),
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
  holds(Disposition, soma:affordsTrigger, only(TriggerRole)),
  subclass_of(TriggerRole, only(dul:classifies,TriggerType)).

		 /*******************************
		 *	    ROLES & PARAMETERS		*
		 *******************************/

%% is_patient(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Patient'.
%
% @param Entity An entity IRI.
%
is_patient(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,soma:'Patient').

%% is_instrument(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Instrument'.
%
% @param Entity An entity IRI.
%
is_instrument(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,soma:'Instrument').

%% is_location(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Location'.
%
% @param Entity An entity IRI.
%
is_location(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,soma:'Location').

%% is_destination(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Destination'.
%
% @param Entity An entity IRI.
%
is_destination(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,soma:'Destination').

%% is_origin(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Origin'.
%
% @param Entity An entity IRI.
%
is_origin(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,soma:'Origin').
