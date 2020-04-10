
:- module(model_EASE_EXT,
    [
      kb_reification(r,r),
      %%
      constraint_term(r,?),
      has_constrained_concept(r,r,r),
      has_dependent_concept(r,r,r),
      set_constrained_concept(r,r),
      set_dependent_concept(r,r),
      %%
      building_number_of_levels(r,?),
      building_number_of_stories(r,?), 
      building_has_room(r,r)
     ]).
:- rdf_module.
/** <module> Interface predicates for proposed extensions of the EASE model.

@author Daniel Beßler
@author Lars Kunze
@license BSD
*/

		 /*******************************
		 *	    REIFICATIONS		*
		 *******************************/

%%
%
%
kb_reification(Resource,Reification) :-
  ask( Reification ease:'isReificationOf' Resource ),!.

kb_reification(Resource,Reification) :-
  atom(Resource),
  new( ease:'Reification', Reification ),
  tell( Reification ease:'isReificationOf' Resource ).

		 /*******************************
		 *	    CONSTRAINTS		*
		 *******************************/

%% constraint_term(+Constraint,?Term) is semidet
%
%
constraint_term(Constraint,Term) :-
  ask( Constraint rdf:type   C_type ),
  ask( C_type     rdfs:label C_label ),!,
  %%
  has_constrained_concept(Constraint,R0,_),
  has_dependent_concept(Constraint,R1,_),
  Term =.. [C_label,R0,R1].

%% has_constrained_concept(+Constraint,?Role0,?Role) is semidet
%
%
has_constrained_concept(Constraint,Role0,Role) :-
  ask( Constraint knowrob:constrains Role0 ),
  ask( Role0      rdf:type           Role ).

%% has_dependent_concept(+Constraint,?Role0,?Role) is semidet
%
%
has_dependent_concept(Constraint,Role0,Role) :-
  ask( Constraint knowrob:dependsOnConcept Role0 ),
  ask( Role0      rdf:type                 Role ).

%% set_constrained_concept(+Concept0,+Concept1) is semidet
%
%
set_constrained_concept(Concept0,Concept1) :-
  tell( Concept0 knowrob:constrains Concept1 ).

%% set_dependent_concept(+Concept0,+Concept1) is semidet
%
%
set_dependent_concept(Concept0,Concept1) :-
  tell( Concept0 knowrob:dependsOnConcept Concept1 ).

		 /*******************************
		 *	    BUILDING		*
		 *******************************/

%% building_number_of_levels(?Map:iri, ?N:int) is semidet
%
% True if N is the number of levels in Map.
%
% @param Map The semantic map 
% @param N The number of levels
%
building_number_of_levels(Map, N):-
  setof(L, ask( Map knowrob:'hasLevels' L ), Ls),
  length(Ls, N).

%% building_number_of_stories(?Map:iri, ?N:int) is semidet
%
% True if N is the number of stories in Map.
%
% @param Map The semantic map 
% @param N The number of stories
%
building_number_of_stories(Map, N) :-
  setof(L, (
    ask( Map knowrob:'hasLevels', L ),
    ask( L   rdf:type             knowrob:'AboveGroundLevelInAConstruction' )
  ), Ls),
  length(Ls, N).

%% building_has_room(+Map:iri, ?Room:iri) is semidet
%
% True if Room is a Room of some level in Map.
%
% @param Map The semantic map 
% @param Room The room in Map
%
building_has_room(Map, Room) :-
  ask( Map   knowrob:'hasLevels' Level ),
  ask( Level knowrob:'hasRooms'  Room ).
