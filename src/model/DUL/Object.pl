:- module(model_DUL_Object,
    [ is_object(r),
      is_quality(r),  % ?Quality
      is_concept(r),
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
    [ has_type/2 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).

% setup tabled ask calls (the "g_" is prepended in expand_term)
:- table(g_is_object/1).
:- table(g_is_quality/1).
:- table(g_is_concept/1).
:- table(g_is_role/1).
:- table(g_is_agent/1).
:- table(g_is_physical_object/1).
:- table(g_is_social_object/1).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Object'.
%
% @param Entity An entity IRI.
%
is_object(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Object').

%% is_quality(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Quality'.
%
% @param Entity An entity IRI.
%
is_quality(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Quality').

%% is_concept(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Concept'.
%
% @param Entity An entity IRI.
%
is_concept(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Concept').

%% is_role(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Role'.
%
% @param Entity An entity IRI.
%
is_role(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Role').

%% is_agent(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Agent'.
%
% @param Entity An entity IRI.
%
is_agent(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Agent').

%% is_physical_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PhysicalObject'.
%
% @param Entity An entity IRI.
%
is_physical_object(Entity), [table(?)] ?+>
  has_type(Entity, dul:'PhysicalObject').

%% is_social_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SocialObject'.
%
% @param Entity An entity IRI.
%
is_social_object(Entity), [table(?)] ?+>
  has_type(Entity, dul:'SocialObject').

%% has_quality_type(+Entity,?Type) is nondet.
%
% Relates an entity to its types that are sub-classes
% of the Quality concept.
%
% @param Entity named individual
% @param Type class resource
%
has_quality_type(Entity,Type) ?>
  has_type(Entity,Type),
  triple(Type,rdfs:subClassOf,dul:'Quality'),
  % only yield one type
  { ! }.

%% has_object_type(+Entity,?Type) is nondet.
%
% Relates an entity to its types that are sub-classes
% of the Object concept.
%
% @param Entity named individual
% @param Type class resource
%
has_object_type(Entity,Type) ?>
  has_type(Entity,Type),
  triple(Type,rdfs:subClassOf,dul:'Object'),
  % only yield one type
  { ! }.

%% has_location(?Object, ?Location) is nondet.
%
% A generic, relative spatial location, holding between any entities. E.g.
% - 'the cat is on the mat',
% - 'Omar is in Samarcanda',
% - 'the wound is close to the femural artery'.
%
% @param Object named individual
% @param Location named individual
%
has_location(Object, Location) ?+>
  holds(Object, dul:hasLocation, Location).

%% has_role(?Entity,?Role) is nondet.
%
% Relates an object to its roles.
%
% @param Entity named individual
% @param Role named individual
%
has_role(Entity,Role) ?+>
  holds(Role, dul:classifies, Entity).
