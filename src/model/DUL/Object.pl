:- module(model_DUL_Object,
    [ is_object(r),
      is_quality(r),  % ?Quality
      is_role(r),   % ?Role
      is_agent(r),
      is_physical_object(r),
      is_social_object(r),
      has_object_type(r,r),
      has_quality_type(r,r),
      has_location(r,r),
      has_role(r,r) % ?Object, ?Role
    ]).
/** <module> DUL notion of Object.

In DUL, Object is defined as:
  "Any physical, social, or mental object, or a substance. Following DOLCE Full, objects are always participating in some event (at least their own life), and are spatially located."

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2
    ]).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Object'.
%
% @param Entity An entity IRI.
%
is_object(Entity) ?+>
  has_type(Entity, dul:'Object').

%% is_quality(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Quality'.
%
% @param Entity An entity IRI.
%
is_quality(Entity) ?+>
  has_type(Entity, dul:'Quality').

%% is_role(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Role'.
%
% @param Entity An entity IRI.
%
is_role(Entity) ?+>
  has_type(Entity, dul:'Role').

%% is_agent(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Agent'.
%
% @param Entity An entity IRI.
%
is_agent(Entity) ?+>
  has_type(Entity, dul:'Agent').

%% is_physical_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PhysicalObject'.
%
% @param Entity An entity IRI.
%
is_physical_object(Entity) ?+>
  has_type(Entity, dul:'PhysicalObject').

%% is_social_object(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'SocialObject'.
%
% @param Entity An entity IRI.
%
is_social_object(Entity) ?+>
  has_type(Entity, dul:'SocialObject').

%%
%
has_quality_type(Entity,Type) ?>
  has_type(Entity,Type),
  triple(Type,rdfs:subClassOf,dul:'Quality'),
  % only yield one type
  { ! }.

%%
%
has_object_type(Entity,Type) ?>
  has_type(Entity,Type),
  triple(Type,rdfs:subClassOf,dul:'Object'),
  % only yield one type
  { ! }.

%%
%
has_location(Object, Location) ?+>
  holds(Object, dul:hasLocation, Location).

%%
%
has_role(Entity,Role) ?+>
  holds(Role, dul:classifies, Entity).
