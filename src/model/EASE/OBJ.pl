:- module(model_EASE_OBJ,
    [ object_frame_name(r,?),
      %% Life Time
      is_alive(r),
      %% Qualities
      is_physical_quality(r),
      is_social_quality(r),
      is_intrinsic(r),
      is_extrinsic(r),
      object_localization(r,r),
      object_color_rgb(r,?),
      object_dimensions(r,?,?,?),
      object_mesh_path(r,?),
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

:- use_module(library('model/RDFS')
        [ has_type/2 ]).
:- use_module(library('model/OWL')
        [ is_individual/2 ]).
:- use_module(library('model/DUL/Object')
        [ has_role/2, has_object_type/2, has_quality_type/2 ]).
:- use_module(library('db/tripledb')
        [ tripledb_load/2 ]).

:- tripledb_load(
        'http://www.ease-crc.org/ont/EASE-OBJ.owl',
        [ graph(static),
          namespace(ease_obj)
        ]).

%% object_frame_name(+Obj:iri, ?FrameName:atom) is det
%
% True if FrameName is the name of origin frame of the object.
% Fallback is to use the IRI suffix of Obj.
% 
% @param Obj Instance of SpatialThing
% @param FrameName The frame name
%
object_frame_name(Obj,FrameName) :-
  % FIXME: use of knowrob here
  holds(Obj,knowrob:frameName,FrameName), !.

% TODO: this clause really needed?
%object_frame_name(Localization,FrameName) :-
  %atom(Localization),
  %holds(Obj,ease_obj:hasLocalization,Localization),
  %object_frame_name(Obj,FrameName), !.

object_frame_name(Obj,FrameName) :-
  % FIXME: this does not work for var(Obj)
  %           so generally knowrob:frameName is required :/
  atom(Obj),
  rdf_split_url(_,FrameName,Obj).

		 /*******************************
		 *	    LIFE TIME		*
		 *******************************/

%% is_alive(+Obj) is det.
%
is_alive(Obj) ?>
  holds(Obj,ease:hasLifetime,Evt),
  occurs(Evt).

is_alive(Obj) +>
  { ask(holds(Obj,ease:hasLifetime,Evt)) },
  occurs(Evt).

		 /*******************************
		 *	    QUALITIES		*
		 *******************************/

%% is_physical_quality(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'PhysicalQuality'.
%
% @param Entity An entity IRI.
%
is_physical_quality(Entity) ?+>
  has_type(Entity, ease_obj:'PhysicalQuality').

%% is_social_quality(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'SocialQuality'.
%
% @param Entity An entity IRI.
%
is_social_quality(Entity) ?+>
  has_type(Entity, ease_obj:'SocialQuality').

%% is_intrinsic(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Intrinsic'.
%
% @param Entity An entity IRI.
%
is_intrinsic(Entity) ?+>
  has_type(Entity, ease_obj:'Intrinsic').

%% is_extrinsic(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Extrinsic'.
%
% @param Entity An entity IRI.
%
is_extrinsic(Entity) ?+>
  has_type(Entity, ease_obj:'Extrinsic').

%% object_localization(?Obj, ?Loc) is det
%
%
object_localization(Obj,Loc) ?+>
  holds(Obj,ease_obj:hasLocalization,Loc).

%% object_color_rgb(?Obj:iri, ?Col:list) is det
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue, alpha], on a scale of 0-1.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color_rgb(Obj,[R,G,B]) ?+>
  holds(Obj,ease_obj:hasColor,Color),
  holds(Color,dul:hasRegion,Region),
  % TODO: what happens to old values here on tell?
  holds(Region,ease_obj:hasRGBValue,[R,G,B]),
  { ! }.

% TODO: really allow data values everywhere?
%           seems hacky... and slows down
object_color_rgb(Obj, [R,G,B]) ?>
  holds(Obj,ease_obj:hasColor,Color),
  holds(Color,ease_obj:hasRGBValue,[R,G,B]),
  { ! }.

object_color_rgb(Obj, [R,G,B]) ?>
  holds(Obj,ease_obj:hasRGBValue,[R,G,B]),
  { ! }.

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
object_dimensions(Obj, Depth, Width, Height) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  shape_bbox(ShapeRegion,Depth,Width,Height),
  { ! }.

% TODO: really allow data values everywhere?
%           seems hacky... and slows down
object_dimensions(Obj, Depth, Width, Height) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  shape_bbox(Shape,Depth,Width,Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) ?>
  shape_bbox(Obj, Depth, Width, Height),
  { ! }.

object_dimensions(Obj, Depth, Width, Height) +>
  { ask(holds(Obj,ease_obj:hasShape,Shape)) },
  is_individual(ShapeRegion),
  has_type(ShapeRegion,ease_obj:'BoxShape'),
  % TODO: use array term instead?
  % TODO: what happens to old values here on tell?
  holds(ShapeRegion, ease_obj:hasDepth,  Depth),
  holds(ShapeRegion, ease_obj:hasWidth,  Width),
  holds(ShapeRegion, ease_obj:hasHeight, Height),
  holds(Shape,       dul:hasRegion,      ShapeRegion).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) ?>
  holds(ShapeRegion, ease_obj:hasDepth, Depth),
  holds(ShapeRegion, ease_obj:hasWidth, Width),
  holds(ShapeRegion, ease_obj:hasHeight, Height),
  { ! }.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) ?>
  %holds(ShapeRegion,rdf:type,ease_obj:'SphereShape' ),
  holds(ShapeRegion ease_obj:hasRadius Radius),
  { Diameter is 2 * Radius },
  { ! }.

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,dul:hasRegion,ShapeRegion),
  holds(ShapeRegion,ease_obj:hasFilePath,FilePath),
  { ! }.

% TODO: really allow data values everywhere?
%           seems hacky... and slows down
object_mesh_path(Obj, FilePath) ?>
  holds(Obj,ease_obj:hasShape,Shape),
  holds(Shape,ease_obj:hasFilePath,FilePath),
  { ! }.

object_mesh_path(Obj, FilePath) ?>
  holds(Obj ease_obj:hasFilePath FilePath ),
  { ! }.

object_mesh_path(Obj, FilePath) +>
  { ask(holds(Obj,ease_obj:hasShape,Shape)) },
  is_individual(ShapeRegion),
  % TODO: what happens to old values here on tell?
  has_type(ShapeRegion,ease_obj:'MeshShape'),
  holds(Shape,dul:hasRegion,ShapeRegion),
  holds(ShapeRegion,ease_obj:hasFilePath,FilePath).

		 /*******************************
		 *	    FEATURES		*
		 *******************************/

%% is_feature(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Feature'.
%
% @param Entity An entity IRI.
%
is_feature(Entity) ?+>
  has_type(Entity,ease_obj:'Feature').

%% object_feature(+Obj, ?Feature) is semidet.
%
%
object_feature(Obj, Feature) ?+>
  holds(Obj,ease_obj:hasFeature,Feature).

%% object_feature(+Obj, ?Feature, ?FeatureType) is semidet.
%
%
object_feature_type(Obj, Feature, FeatureType) ?>
  object_feature(Obj,Feature),
  has_object_type(Feature,FeatureType).

		 /*******************************
		 *	    AFFORDANCES		*
		 *******************************/

%% is_affordance(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Affordance'.
%
% @param Entity An entity IRI.
%
is_affordance(Entity) ?+>
  has_type(Entity,ease_obj:'Affordance').

%% is_disposition(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Disposition'.
%
% @param Entity An entity IRI.
%
is_disposition(Entity) ?+>
  has_type(Entity,ease_obj:'Disposition').

%% has_disposition(?Obj:iri, ?Disposition:iri) is nondet.
%
% Relates an object to one of its dispositions.
%
% @param Obj           An individual of type dul:'Object'.
% @param Disposition   An individual of type ease_obj:'Disposition'.
%
has_disposition(Obj, Disposition) ?+>
  holds(Obj,ease_obj:hasDisposition,Disposition).

%% has_disposition(?Obj:iri, ?Disposition:iri, +DispositionType:iri) is nondet.
%
% Relates an object to one of its dispositions that is an instance
% of given disposition type.
%
% @param Obj               An individual of type dul:'Object'.
% @param Disposition       An individual of type ease_obj:'Disposition'.
% @param DispositionType   A sub-class of ease_obj:'Disposition'.
%
has_disposition_type(Obj, Disposition, DispositionType) ?>
  holds(Obj,ease_obj:hasDisposition,Disposition),
  has_quality_type(Disposition,DispositionType).

%% disposition_trigger_type(?Disposition:iri, ?TriggerType:iri) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition   An individual of type ease_obj:'Disposition'.
% @param TriggerType   A sub-class of dul:'Object'.
%
disposition_trigger_type(Disposition,TriggerType) ?>
  holds(Disposition, ease_obj:affordsTrigger, only(TriggerRole)),
  holds(TriggerRole, dul:classifies,          only(TriggerType)).

		 /*******************************
		 *	    ROLES & PARAMETERS		*
		 *******************************/

%% is_patient(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Patient'.
%
% @param Entity An entity IRI.
%
is_patient(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Patient').

% TODO: not sure about just creating a role symbol here.
%         this needs some more consideration.
%         e.g. what about role symbols defined by tasks?
%            **is_patient(X) during Event**
%is_patient(Entity) +>
  %is_role(Role),
  %has_type(Role,ease_obj:'Patient'),
  %has_role(Entity,Role)

%% is_instrument(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Instrument'.
%
% @param Entity An entity IRI.
%
is_instrument(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Instrument').

%% is_location(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Location'.
%
% @param Entity An entity IRI.
%
is_location(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Location').

%% is_destination(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Destination'.
%
% @param Entity An entity IRI.
%
is_destination(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Destination').

%% is_origin(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Origin'.
%
% @param Entity An entity IRI.
%
is_origin(Entity) ?>
  has_role(Entity,Role),
  has_object_type(Role,ease_obj:'Origin').
