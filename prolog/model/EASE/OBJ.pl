
:- module(model_EASE_OBJ,
    [
      object_frame_name(r,?),
      %% Life Time
      object_is_alive(r),
      object_lifetime(r,r),
      %% Qualities
      is_physical_quality(r),
      is_social_quality(r),
      is_intrinsic(r),
      is_extrinsic(r),
      object_localization(r,r),
      object_color(r,?),
      object_assert_color(r,+),
      object_dimensions(r,?,?,?),
      object_mesh_path(r,?),
      object_assert_dimensions(r,+,+,+),
      object_shape_type(r,r),
      %% Features
      is_feature(r),
      object_feature(r,r),
      object_feature(r,r,r),
      %% Affordances
      is_affordance(r),
      is_disposition(r),
      object_disposition(r,r),
      object_disposition(r,r,r),
      disposition_trigger_type(r,r),
      %% Roles & Parameters
      is_patient(r),
      is_instrument(r),
      is_location(r),
      is_destination(r),
      is_origin(r)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-OBJ model.

@author Daniel Beßler
@license BSD
*/

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
  ask( Obj knowrob:frameName FrameName ), !.

object_frame_name(Localization,FrameName) :-
  atom(Localization),
  ask( Obj ease_obj:hasLocalization Localization ),
  object_frame_name(Obj,FrameName), !.

object_frame_name(Obj,FrameName) :-
  atom(Obj),
  rdf_split_url(_,FrameName,Obj).

		 /*******************************
		 *	    LIFE TIME		*
		 *******************************/

%% object_is_alive(+Obj) is det.
%
%
object_is_alive(Obj) :-
  object_lifetime(Obj,LT),
  event_is_active(LT).

%% object_lifetime(+Obj,?LT) is det.
%
%
object_lifetime(Obj,LT) :-
  ask( Obj ease:hasLifetime LT ).

		 /*******************************
		 *	    QUALITIES		*
		 *******************************/

%% is_physical_quality(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'PhysicalQuality'.
%
% @param Entity An entity IRI.
%
is_physical_quality(Entity) :-
  ask( Entity rdf:type ease_obj:'PhysicalQuality' ).

%% is_social_quality(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'SocialQuality'.
%
% @param Entity An entity IRI.
%
is_social_quality(Entity) :-
  ask( Entity rdf:type ease_obj:'SocialQuality' ).

%% is_intrinsic(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Intrinsic'.
%
% @param Entity An entity IRI.
%
is_intrinsic(Entity) :-
  ask( Entity rdf:type ease_obj:'Intrinsic' ).

%% is_extrinsic(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Extrinsic'.
%
% @param Entity An entity IRI.
%
is_extrinsic(Entity) :-
  ask( Entity rdf:type ease_obj:'Extrinsic' ).

%% object_localization(?Obj, ?Loc) is det
%
%
object_localization(Obj,Loc) :-
  ask( Obj ease_obj:hasLocalization Loc ).

object_localization(Loc,Loc) :-
  atom(Obj),
  ask( Loc rdf:type ease_obj:'Localization' ),!.

%% object_color(?Obj:iri, ?Col:list) is det
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue, alpha], on a scale of 0-1.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_color(Obj, [R,G,B,A]) :-
  ask( Obj         ease_obj:hasColor     Color ),
  ask( Color       dul:hasRegion         ColorRegion ),
  ask( ColorRegion ease_obj:hasRGBValue  [R,G,B|Rest] ),
  ( Rest=[A] ; A is 1.0 ),!.

object_color(Obj, [R,G,B,A]) :-
  ask( Obj ease_obj:hasRGBValue [R,G,B|Rest] ),
  ( Rest=[A] ; A is 1.0 ),!.

%% object_assert_color(+Obj:iri, +Col:list) is det
%
% Assert object main color property.
%
% @param Obj  Instance of a subclass of EnduringThing-Localized
% @param Col  Main color of the object
% 
object_assert_color(Obj,ColorValue) :-
  new( ease_obj:'ColorRegion', ColorRegion ),
  ask(  Obj         ease_obj:hasColor    Color ),
  tell( ColorRegion ease_obj:hasRGBValue ColorValue ),
  tell( Color       dul:hasRegion        ColorRegion ).

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
  ask( Obj   ease_obj:hasShape Shape),
  ask( Shape dul:hasRegion     ShapeRegion),
  shape_bbox(ShapeRegion, Depth, Width, Height),!.

object_dimensions(Obj, Depth, Width, Height) :-
  shape_bbox(Obj, Depth, Width, Height),!.

%% object_shape_type(?Obj:iri, ?Shape:iri) is semidet
%
object_shape_type(Obj,ShapeType) :-
  ask( Obj   ease_obj:hasShape Shape),
  ask( Shape rdf:type          ShapeType ).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) :-
  ask( ShapeRegion ease_obj:hasDepth Depth),
  ask( ShapeRegion ease_obj:hasWidth Width),
  ask( ShapeRegion ease_obj:hasHeight Height), !.

shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) :-
  ask( ShapeRegion rdf:type           ease_obj:'SphereShape' ),
  ask( ShapeRegion ease_obj:hasRadius Radius ), !,
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
object_assert_dimensions(Obj, ShapeRegion, Width, Height) :-
  new( ease_obj:'BoxShape', ColorRegion ),
  ask( Obj          ease_obj:hasShape   Shape ),
  tell(ShapeRegion, ease_obj:hasDepth,  Depth),
  tell(ShapeRegion, ease_obj:hasWidth,  Width),
  tell(ShapeRegion, ease_obj:hasHeight, Height),
  tell(Shape        dul:hasRegion       ShapeRegion ).

%% object_mesh_path(+Obj:iri, -FilePath:atom) is det
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj        Instance of a subclass of EnduringThing-Localized
% @param FilePath   the path (usually a package:// path)
%
object_mesh_path(Obj, FilePath) :-
  ask( Obj         ease_obj:hasShape    Shape ),
  ask( Shape       dul:hasRegion        ShapeRegion ),
  ask( ShapeRegion ease_obj:hasFilePath FilePath ),!.

object_mesh_path(Obj, FilePath) :-
  ask( Obj ease_obj:hasFilePath FilePath ),!.

		 /*******************************
		 *	    FEATURES		*
		 *******************************/

%% is_feature(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Feature'.
%
% @param Entity An entity IRI.
%
is_feature(Entity) :-
  ask( Entity rdf:type ease_obj:'Feature' ).

%% object_feature(+Obj, ?Feature) is semidet.
%
%
object_feature(Obj, Feature) :-
  ask( Obj ease_obj:hasFeature Feature ).

%% object_feature(+Obj, ?Feature, ?FeatureType) is semidet.
%
%
object_feature(Obj, Feature, FeatureType) :-
  object_feature(Obj, Feature),
  ask( Feature rdf:type FeatureType ).

		 /*******************************
		 *	    AFFORDANCES		*
		 *******************************/

%% is_affordance(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Affordance'.
%
% @param Entity An entity IRI.
%
is_affordance(Entity) :-
  ask( Entity rdf:type ease_obj:'Affordance' ).

%% is_disposition(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Disposition'.
%
% @param Entity An entity IRI.
%
is_disposition(Entity) :-
  ask( Entity rdf:type ease_obj:'Disposition' ).

%% object_disposition(?Obj:iri, ?Disposition:iri) is nondet.
%
% Relates an object to one of its dispositions.
%
% @param Obj           An individual of type dul:'Object'.
% @param Disposition   An individual of type ease_obj:'Disposition'.
%
object_disposition(Obj, Disposition) :-
  ask( Obj ease_obj:hasDisposition Disposition ).

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
  ask( Obj         ease_obj:hasDisposition Disposition ),
  ask( Disposition rdf:type                DispositionType ).

%% disposition_trigger_type(?Disposition:iri, ?TriggerType:iri) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition   An individual of type ease_obj:'Disposition'.
% @param TriggerType   A sub-class of dul:'Object'.
%
disposition_trigger_type(Disposition,TriggerType) :-
  ask( Disposition ease_obj:affordsTrigger only(TriggerRole) ),!,
  ask( TriggerRole dul:classifies          only(ClassifiedType) ),
  ( var(TriggerType) -> TriggerType=ClassifiedType ;
    once( ask(TriggerType rdf:type ClassifiedType) )
  ),!.

		 /*******************************
		 *	    ROLES & PARAMETERS		*
		 *******************************/

%% is_patient(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Patient'.
%
% @param Entity An entity IRI.
%
is_patient(Entity) :-
  ask( Entity rdf:type ease_obj:'Patient' ).

%% is_instrument(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Instrument'.
%
% @param Entity An entity IRI.
%
is_instrument(Entity) :-
  ask( Entity rdf:type ease_obj:'Instrument' ).

%% is_location(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Location'.
%
% @param Entity An entity IRI.
%
is_location(Entity) :-
  ask( Entity rdf:type ease_obj:'Location' ).

%% is_destination(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Destination'.
%
% @param Entity An entity IRI.
%
is_destination(Entity) :-
  ask( Entity rdf:type ease_obj:'Destination' ).

%% is_origin(+Entity) is semidet.
%
% True iff Entity is an instance of ease_obj:'Origin'.
%
% @param Entity An entity IRI.
%
is_origin(Entity) :-
  ask( Entity rdf:type ease_obj:'Origin' ).
