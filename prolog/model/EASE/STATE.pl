
:- module(model_EASE_STATE,
    [
      is_state(r),
      is_physical_state(r),
      is_social_state(r),
      is_configuration(r),
      is_state_type(r)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-STATE model.

@author Daniel Beßler
@license BSD
*/

%% is_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'State'.
%
% @param Entity An entity IRI.
%
is_state(Entity) :-
  ask( Entity rdf:type ease_state:'State' ).

%% is_physical_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'PhysicalState'.
%
% @param Entity An entity IRI.
%
is_physical_state(Entity) :-
  ask( Entity rdf:type ease_state:'PhysicalState' ).

%% is_social_state(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'SocialState'.
%
% @param Entity An entity IRI.
%
is_social_state(Entity) :-
  ask( Entity rdf:type ease_state:'SocialState' ).

%% is_configuration(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'Configuration'.
%
% @param Entity An entity IRI.
%
is_configuration(Entity) :-
  ask( Entity rdf:type ease_state:'Configuration' ).

%% is_state_type(+Entity) is semidet.
%
% True iff Entity is an instance of ease_state:'StateType'.
%
% @param Entity An entity IRI.
%
is_state_type(Entity) :-
  ask( Entity rdf:type ease_state:'StateType' ).
