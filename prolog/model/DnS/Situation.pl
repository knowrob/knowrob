
:- module(model_DnS_Situation,
    [
      is_situation(r),
      is_classification(r),
      is_transition(r),
      is_plan_execution(r),
      situation_satisfies(r,r),
      situation_classifies(r,r,r),
      situation_add(r,r),
      situation_add_satisfies(r,r),
      situation_add_classifies(r,r,r),
      classification_concept(r,r),
      classification_entity(r,r)
    ]).
:- rdf_module.
/** <module> DnS notion of Situation.

In DnS, Situation is defined as:
  "A view, consistent with ('satisfying') a Description, on a set of entities. It can also be seen as a 'relational context' created by an observer on the basis of a 'frame' (i.e. a Description)."

@author Daniel Beßler
@license BSD
*/

%% is_situation(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Situation'.
%
% @param Entity An entity IRI.
%
is_situation(Entity) :-
  ask( Entity rdf:type dul:'Situation' ).

%% is_classification(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Classification'.
%
% @param Entity An entity IRI.
%
is_classification(Entity) :-
  ask( Entity rdf:type dul:'Classification' ).

%% is_transition(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Transition'.
%
% @param Entity An entity IRI.
%
is_transition(Entity) :-
  ask( Entity rdf:type dul:'Transition' ).

%% is_plan_execution(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'PlanExecution'.
%
% @param Entity An entity IRI.
%
is_plan_execution(Entity) :-
  ask( Entity rdf:type dul:'PlanExecution' ).

%% situation_add(+Sit,+Entity) is det.
%
% Asserts that some entity is included in the situation.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity A named individual.
%
situation_add(Sit,Act) :-
  ask( Act rdf:type dul:'Action' ),!,
  tell( Sit dul:includesAction Act ).

situation_add(Sit,Evt) :-
  ask( Evt rdf:type dul:'Event' ),!,
  tell( Sit dul:includesEvent Evt ).

situation_add(Sit,Agent) :-
  ask( Agent rdf:type dul:'Agent' ),!,
  tell( Sit dul:includesAgent Agent ).

situation_add(Sit,SubSituation) :-
  ask( SubSituation rdf:type dul:'Situation' ),!,
  tell( Sit dul:hasPart SubSituation ).

situation_add(Sit,Region) :-
  ask(Region rdf:type dul:'Region'),!,
  % TODO: use more specific relation
  tell( Sit dul:isSettingFor Region ).

situation_add(Sit,Object) :-
  ask( Object rdf:type dul:'Object' ),!,
  tell( Sit dul:includesObject Object ).

%% situation_satisfies(?Sit,?Descr) is nondet.
%
% Associates a situation to a description that is
% satisfied by the situation.
% An example is that the execution of a plan (a situation)
% satisfies the plan (a description).
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
situation_satisfies(Sit,Descr) :-
  ask( Sit dul:satisfies Descr ).

%% situation_add_satisfies(+Sit,+Descr) is det.
%
% Asserts that a situation satisfies some description
% (such as a plan).
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
situation_add_satisfies(Sit,Descr) :-
  tell( Sit dul:satisfies Descr ).

%% situation_classifies(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_classifies(Sit,Entity,Concept) :-
  ask( Sit            dul:hasPart Classification ),
  ask( Classification rdf:type    dul:'Classification'),
  classification_concept(Classification,Concept),
  classification_entity(Classification,Entity).

%% situation_add_classifies(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_add_classifies(Sit,Entity,Concept) :-
  new( dul:'Classification', Classification ),
  situation_add(Classification,Entity),
  situation_add(Classification,Concept),
  situation_add(Sit,Classification).

		 /*******************************
		 *	classifications		*
		 *******************************/

%% 
classification_concept(Classification,Concept) :-
  ask( Classification dul:includesObject Concept ),
  ask( Concept        rdf:type           dul:'Concept' ).

%% 
classification_entity(Classification,Entity) :-
  ask( Classification dul:includesObject Entity ),
  \+ ask( Concept     rdf:type           dul:'Concept' ).
