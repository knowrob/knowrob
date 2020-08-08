:- module(model_EASE_PROC,
    [ is_chemical_process(r),
      is_physical_process(r),
      is_force_interaction(r),
      is_motion(r),
      is_process_flow(r),
      is_process_type(r),
      is_progression(r)
    ]).
/** <module> Interface predicates for EASE-PROC model.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).

%% is_chemical_process(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ChemicalProcess'.
%
% @param Entity An entity IRI.
%
is_chemical_process(Entity) ?+>
  has_type(Entity, soma:'ChemicalProcess').

%% is_physical_process(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalProcess'.
%
% @param Entity An entity IRI.
%
is_physical_process(Entity) ?+>
  has_type(Entity, soma:'PhysicalProcess').

%% is_process_flow(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ProcessFlow'.
%
% @param Entity An entity IRI.
%
is_process_flow(Entity) ?+>
  has_type(Entity, soma:'ProcessFlow').

%% is_process_type(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ProcessType'.
%
% @param Entity An entity IRI.
%
is_process_type(Entity) ?+>
  has_type(Entity, soma:'ProcessType').

%% is_motion(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Motion'.
%
% @param Entity An entity IRI.
%
is_motion(Entity) ?+>
  has_type(Entity, soma:'Motion').

%% is_force_interaction(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ForceInteraction'.
%
% @param Entity An entity IRI.
%
is_force_interaction(Entity) ?+>
  has_type(Entity, soma:'ForceInteraction').

%% is_progression(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Progression'.
%
% @param Entity An entity IRI.
%
is_progression(Entity) ?+>
  has_type(Entity, soma:'Progression').
