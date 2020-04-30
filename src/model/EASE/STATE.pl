:- module(model_EASE_STATE,
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

:- use_module(library('model/RDFS')
        [ has_type/2 ]).
:- use_module(library('db/tripledb')
        [ tripledb_load/2 ]).

:- tripledb_load(
        'http://www.ease-crc.org/ont/EASE-STATE.owl',
        [ graph(static),
          namespace(ease_state)
        ]).

%% is_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'State'.
%
% @param Entity An entity IRI.
%
is_state(Entity) ?+>
  has_type(Entity, ease_state:'State').

%% is_physical_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'PhysicalState'.
%
% @param Entity An entity IRI.
%
is_physical_state(Entity) ?+>
  has_type(Entity, ease_state:'PhysicalState').

%% is_social_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'SocialState'.
%
% @param Entity An entity IRI.
%
is_social_state(Entity) ?+>
  has_type(Entity, ease_state:'SocialState').

%% is_configuration(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'Configuration'.
%
% @param Entity An entity IRI.
%
is_configuration(Entity) ?+>
  has_type(Entity, ease_state:'Configuration').

%% is_state_type(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'StateType'.
%
% @param Entity An entity IRI.
%
is_state_type(Entity) ?+>
  has_type(Entity, ease_state:'StateType').
