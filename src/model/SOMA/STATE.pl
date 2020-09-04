:- module(model_SOMA_STATE,
    [ is_state(r),
      is_physical_state(r),
      is_social_state(r),
      is_configuration(r),
      is_state_type(r)
    ]).
/** <module> Interface predicates for EASE-STATE model.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).

%% is_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'State'.
%
% @param Entity An entity IRI.
%
is_state(Entity) ?+>
  has_type(Entity, soma:'State').

%% is_physical_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalState'.
%
% @param Entity An entity IRI.
%
is_physical_state(Entity) ?+>
  has_type(Entity, soma:'PhysicalState').

%% is_social_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'SocialState'.
%
% @param Entity An entity IRI.
%
is_social_state(Entity) ?+>
  has_type(Entity, soma:'SocialState').

%% is_configuration(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Configuration'.
%
% @param Entity An entity IRI.
%
is_configuration(Entity) ?+>
  has_type(Entity, soma:'Configuration').

%% is_state_type(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'StateType'.
%
% @param Entity An entity IRI.
%
is_state_type(Entity) ?+>
  has_type(Entity, soma:'StateType').
