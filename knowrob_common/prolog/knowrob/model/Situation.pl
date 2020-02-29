
:- module(knowrob_model_Situation,
    [
      situation_satisfies/2,
      situation_create/3,
      situation_add/2,
      situation_add_satisfies/2,
      %%
      situation_includes_classification/3,
      situation_add_classification/3,
      classification_concept/2,
      classification_entity/2,
      set_classification_entity/2,
      set_classification_concept/2,
      %%
      assignment_argument/2,
      assignment_value/2,
      set_assignment_argument/2,
      set_assignment_value/2,
      situation_includes_assignment/3,
      situation_add_assignment/3
    ]).
/** <module> Interface to RDF model of situations.

*Situation* is defined as a view, consistent with ('satisfying') a *Description*, on a set of entities. 
It can also be seen as a 'relational context' created by an observer on the basis of a 'frame' (i.e. a *Description*). 
For example, a *PlanExecution* is a context including some actions executed by agents according to certain parameters and expected tasks to be achieved from a *Plan*; a *DiagnosedSituation* is a context of observed entities that is interpreted on the basis of a *Diagnosis*, etc.
*Situation* is also able to represent reified n-ary relations, where *isSettingFor* is the top-level relation for all binary projections of the n-ary relation. If used in a transformation pattern for n-ary relations, the designer should take care of creating only one subclass of *Situation* for each n-ary relation, otherwise the 'identification constraint' (Calvanese et al., IJCAI 2001) could be violated.

@author Daniel Beßler
@license BSD
*/
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).

:- rdf_meta
      situation_satisfies(r,r),
      situation_includes_classification(r,r,r),
      situation_includes_assignment(r,r,r),
      situation_create(r,-,+),
      situation_add(r,r),
      situation_add_satisfies(r,r),
      situation_add_classification(r,r,r),
      situation_add_assignment(r,r,r),
      classification_concept(r,r),
      classification_entity(r,r),
      assignment_argument(r,r),
      assignment_value(r,r),
      set_assignment_argument(r,r),
      set_assignment_value(r,r),
      set_classification_entity(r,r),
      set_classification_concept(r,r).

%% situation_create(+SitType,-Sit,+Graph) is semidet.
%
% Creates a situation individual.
%
% @param SitType A sub-class of dul:'Situation'.
% @param Sit An individual of type SitType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
situation_create(SitType,Sit,Graph) :-
  kb_create(SitType,Sit,_{graph:Graph}).

%% situation_add(+Sit,+Entity) is det.
%
% Asserts that some entity is included in the situation.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity A named individual.
%
situation_add(Sit,Act) :-
  kb_type_of(Act,dul:'Action'),!,
  kb_assert(Sit,dul:includesAction,Act).

situation_add(Sit,Evt) :-
  kb_type_of(Evt,dul:'Event'),!,
  kb_assert(Sit,dul:includesEvent,Evt).

situation_add(Sit,Agent) :-
  kb_type_of(Agent,dul:'Agent'),!,
  kb_assert(Sit,dul:includesAgent,Agent).

situation_add(Sit,SubSituation) :-
  kb_type_of(SubSituation,dul:'Situation'),!,
  kb_assert(Sit,ease:includesSituation,SubSituation).

situation_add(Sit,Concept) :-
  kb_type_of(Concept,dul:'Concept'),!,
  kb_assert(Sit,ease:includesConcept,Concept).

situation_add(Sit,Region) :-
  kb_type_of(Region,dul:'Region'),!,
  % TODO: use more specific relation
  kb_assert(Sit,dul:isSettingFor,Region).

situation_add(Sit,Object) :-
  kb_type_of(Object,dul:'Object'),!,
  kb_assert(Sit,dul:includesObject,Object).

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
  kb_triple(Sit,dul:satisfies,Descr).

%% situation_add_satisfies(+Sit,+Descr) is det.
%
% Asserts that a situation satisfies some description
% (such as a plan).
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
situation_add_satisfies(Sit,Descr) :-
  kb_assert(Sit,dul:satisfies,Descr).

		 /*******************************
		 *	classifications		*
		 *******************************/

%% 
classification_concept(Classification,Concept) :-
  kb_triple(Classification,ease:includesConcept,Concept).

%% 
classification_entity(Classification,Entity) :-
  % TODO: better use more specific property
  classification_concept(Classification,Concept),
  kb_triple(Classification,dul:isSettingFor,Entity),
  Concept \= Entity.

%% 
set_classification_entity(Classification,Entity) :-
  situation_add(Classification,Entity).

%% 
set_classification_concept(Sit,Concept) :-
  kb_is_class(Concept),!,
  once(rdf(Sit,rdf:type,_,Graph)),
  kb_create(Concept,Concept0,_{graph:Graph}),
  set_classification_concept(Sit,Concept0).

set_classification_concept(Classification,Concept) :-
  situation_add(Classification,Concept).

%% situation_includes_classification(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_includes_classification(Sit,Entity,Concept) :-
  kb_triple(Sit,ease:includesSituation,Classification),
  rdfs_individual_of(Classification,dul:'Classification'),
  classification_concept(Classification,Concept),
  classification_entity(Classification,Entity).

%% situation_add_classification(?Sit,?Entity,?Concept) is nondet.
%
% Associates a situation to a classification that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity The classified entity.
% @param Concept The dul:'Concept' that classifies the entity.
%
situation_add_classification(Sit,Entity,Concept) :-
  once(rdf(Sit,rdf:type,_,Graph)),
  situation_create(dul:'Classification',Classification,Graph),
  set_classification_concept(Classification,Concept),
  set_classification_entity(Classification,Entity),
  situation_add(Sit,Classification).

		 /*******************************
		 *	assignments		*
		 *******************************/

%% 
assignment_argument(Assignment,Argument) :-
  kb_triple(Assignment,dul:includesObject,Argument),
  kb_type_of(Argument,ease_io:'DigitalObject').

%% 
assignment_value(Assignment,Value) :-
  kb_triple(Assignment,dul:isSettingFor,Value),
  % TODO: better use more specific property
  assignment_argument(Assignment,X),
  Value \= X.

%% 
set_assignment_argument(Assignment,Argument) :-
  situation_add(Assignment,Argument).

%% 
set_assignment_value(Assignment,Value) :-
  situation_add(Assignment,Value).

%% situation_includes_assignment(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_includes_assignment(Sit,Argument,Value) :-
  ground(Argument) ->
  (situation_includes_assignment_(Sit,Argument,Value),!);
  (situation_includes_assignment_(Sit,Argument,Value)).

situation_includes_assignment_(Sit,Argument,Value) :-
  kb_triple(Sit,ease:includesSituation,Assignment),
  rdfs_individual_of(Assignment,knowrob:'Assignment'),
  assignment_argument(Assignment,Argument),
  assignment_value(Assignment,Value).

%% situation_add_assignment(?Sit,?Argument,?Value) is nondet.
%
% Associates a situation to an argument assignment that holds
% within the situational context.
%
% @param Sit An individual of type dul:'Situation'.
% @param Argument An individual of type knowrob:'ProcedureArgument'.
% @param Value The RDF value of the argument.
%
situation_add_assignment(Sit,Argument,Value) :-
  once(rdf(Sit,rdf:type,_,Graph)),
  situation_create(knowrob:'Assignment',Assignment,Graph),
  set_assignment_argument(Assignment,Argument),
  set_assignment_value(Assignment,Value),
  situation_add(Sit,Assignment).
