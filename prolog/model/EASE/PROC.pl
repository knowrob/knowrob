
:- module(model_EASE_PROC,
    [
      is_chemical_process(r),
      is_physical_process(r),
      is_force_interaction(r),
      is_motion(r),
      is_process_flow(r),
      is_process_type(r),
      is_progression(r)
    ]).
:- rdf_module.
/** <module> Interface predicates for EASE-PROC model.

@author Daniel Beßler
@license BSD
*/

%% is_chemical_process(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ChemicalProcess'.
%
% @param Entity An entity IRI.
%
is_chemical_process(Entity) :-
  ask( Entity rdf:type ease_proc:'ChemicalProcess' ).

%% is_physical_process(+Entity) is semidet.
%
% True iff Entity is an instance of ease:'PhysicalProcess'.
%
% @param Entity An entity IRI.
%
is_physical_process(Entity) :-
  ask( Entity rdf:type ease:'PhysicalProcess' ).

%% is_process_flow(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ProcessFlow'.
%
% @param Entity An entity IRI.
%
is_process_flow(Entity) :-
  ask( Entity rdf:type ease_proc:'ProcessFlow' ).

%% is_process_type(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ProcessType'.
%
% @param Entity An entity IRI.
%
is_process_type(Entity) :-
  ask( Entity rdf:type ease_proc:'ProcessType' ).

%% is_motion(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'Motion'.
%
% @param Entity An entity IRI.
%
is_motion(Entity) :-
  ask( Entity rdf:type ease_proc:'Motion' ).

%% is_force_interaction(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ForceInteraction'.
%
% @param Entity An entity IRI.
%
is_force_interaction(Entity) :-
  ask( Entity rdf:type ease_proc:'ForceInteraction' ).

%% is_progression(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'Progression'.
%
% @param Entity An entity IRI.
%
is_progression(Entity) :-
  ask( Entity rdf:type ease_proc:'Progression' ).
