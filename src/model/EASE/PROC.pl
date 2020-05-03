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

:- use_module(library('model/RDFS')
    [ has_type/2
    ]).
:- use_module(library('db/tripledb')
    [ tripledb_load/2
    ]).

:- tripledb_load('http://www.ease-crc.org/ont/EASE-PROC.owl',
    [ graph(tbox),
      namespace(ease_proc)
    ]).

%% is_chemical_process(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ChemicalProcess'.
%
% @param Entity An entity IRI.
%
is_chemical_process(Entity) ?+>
  has_type(Entity, ease_proc:'ChemicalProcess').

%% is_physical_process(+Entity) is semidet.
%
% True iff Entity is an instance of ease:'PhysicalProcess'.
%
% @param Entity An entity IRI.
%
is_physical_process(Entity) ?+>
  has_type(Entity, ease:'PhysicalProcess').

%% is_process_flow(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ProcessFlow'.
%
% @param Entity An entity IRI.
%
is_process_flow(Entity) ?+>
  has_type(Entity, ease_proc:'ProcessFlow').

%% is_process_type(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ProcessType'.
%
% @param Entity An entity IRI.
%
is_process_type(Entity) ?+>
  has_type(Entity, ease_proc:'ProcessType').

%% is_motion(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'Motion'.
%
% @param Entity An entity IRI.
%
is_motion(Entity) ?+>
  has_type(Entity, ease_proc:'Motion').

%% is_force_interaction(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'ForceInteraction'.
%
% @param Entity An entity IRI.
%
is_force_interaction(Entity) ?+>
  has_type(Entity, ease_proc:'ForceInteraction').

%% is_progression(+Entity) is semidet.
%
% True iff Entity is an instance of ease_proc:'Progression'.
%
% @param Entity An entity IRI.
%
is_progression(Entity) ?+>
  has_type(Entity, ease_proc:'Progression').
