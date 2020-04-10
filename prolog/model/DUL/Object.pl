
:- module(model_DUL_Object,
    [
      is_object(r),
      is_agent(r),
      is_physical_object(r),
      is_social_object(r),
      is_composite_object(r),
      object_location(r,r)
    ]).
:- rdf_module.
/** <module> DUL notion of Object.

In DUL, Object is defined as:
  "Any physical, social, or mental object, or a substance. Following DOLCE Full, objects are always participating in some event (at least their own life), and are spatially located."

@author Daniel Beßler
@license BSD
*/

%% is_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Object'.
%
% @param Entity An entity IRI.
%
is_object(Entity) :-
  ask( Entity rdf:type dul:'Object' ).

%% is_agent(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Agent'.
%
% @param Entity An entity IRI.
%
is_agent(Entity) :-
  ask( Entity rdf:type dul:'Agent' ).

%% is_physical_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PhysicalObject'.
%
% @param Entity An entity IRI.
%
is_physical_object(Entity) :-
  ask( Entity rdf:type dul:'PhysicalObject' ).

%% is_social_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SocialObject'.
%
% @param Entity An entity IRI.
%
is_social_object(Entity) :-
  ask( Entity rdf:type dul:'SocialObject' ).

%
is_composite_object(Entity) :-
  is_object( Entity ),
  ask( Entity dul:hasPart SubRegion ), !.

%%
%
object_location(Object, Location) :-
  ask( Object dul:hasLocation Location ).
