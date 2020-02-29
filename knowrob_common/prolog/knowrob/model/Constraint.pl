
:- module(knowrob_model_Constraint,
    [
      constraint_term/2,
      has_constrained_concept/3,
      has_dependent_concept/3,
      set_constrained_concept/2,
      set_dependent_concept/2
    ]).
/** <module> Interface to RDF model of constraints.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      constraint_term(r,t),
      has_constrained_concept(r,r,r),
      has_dependent_concept(r,r,r),
      set_constrained_concept(r,r),
      set_dependent_concept(r,r).

%%
constraint_term(Constraint,Term) :-
  kb_type_of(Constraint,C_type),
  rdfs_label(C_type,C_label),!,
  %%
  has_constrained_concept(Constraint,R0,_),
  has_dependent_concept(Constraint,R1,_),
  Term =.. [C_label,R0,R1].

%%
has_constrained_concept(Constraint,Role0,Role) :-
  kb_triple(Constraint,knowrob:constrains,Role0),
  kb_type_of(Role0,Role).

%%
has_dependent_concept(Constraint,Role0,Role) :-
  kb_triple(Constraint,knowrob:dependsOnConcept,Role0),
  kb_type_of(Role0,Role).
  
%%
set_constrained_concept(Concept0,Concept1) :-
  kb_assert(Concept0,knowrob:constrains,Concept1).

%%
set_dependent_concept(Concept0,Concept1) :-
  kb_assert(Concept0,knowrob:dependsOnConcept,Concept1).
