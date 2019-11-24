
:- module(knowrob_action_effects,
    [
      action_effects_apply/2,
      action_effect/2
    ]).
/** <module> Reasoning about action effects.

The effects of actions on objects are derived from the roles the objects play during the action. For example, whenever an object plays the role of being the *AlteredObject* one of its qualities is being altered to some level. Hence the effect of the action is an alteration of the region of the quality which may be automatically updated by KnowRob after the action has been executed. Other distinct cases are objects being destroyed or created in which case KnowRob updates the lifetime of them, and objects being combined, included, or excluded in which case KnowRob may update the compositional relations between these objects.

@author Moritz Tenorth
@author Daniel Beßler
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_model')).
:- use_module(library('knowrob/objects')).

:-  rdf_meta
    action_effect(r,t),
    action_effects_apply(r,t),
    action_grounding_(r,r,r).

		 /*******************************
		 *	Action post-conditions	*
		 *******************************/

%% action_effect(?Act:iri, ?Effect:term) is nondet
%
% Relates an action or task to roles that imply chang
% and to their range.
% Effect is one of the following terms:
%
%	| created(Type)			| An object has been created.	|
%	| destroyed(Type)		| An object has been destroyed.	|
%	| altered(Type,QualityType)	| An object has been changed.	|
%	| linked(Type)			| An object has been linked. 	|
%	| deposited(Type)		| An object has been deposited. |
%	| commited(Type)		| An object has been commited. 	|
%	| included(Type)		| An object has been included.	|
%	| extracted(Type)		| An object has been extracted.	|
%
action_effect(Act,Effect) :-
  action_has_task(Act,Tsk),
  action_effect_(Tsk,Effect).

action_effect(Tsk,Effect) :-
  action_effect_(Tsk,Effect).

action_effect_(Tsk, created(Type)) :-
  task_role_range(Tsk,ease_obj:'CreatedObject',Type).
action_effect_(Tsk, destroyed(Type)) :-
  task_role_range(Tsk,ease_obj:'DestroyedObject',Type).
action_effect_(Tsk, altered(Type,QualityType)) :-
  task_role_range(Tsk,ease_obj:'AlteredObject',Type),
  get_altered_quality_type_(Type,QualityType).
action_effect_(Tsk, linked(Type)) :-
  task_role_range(Tsk,ease_obj:'LinkedObject',Type).
action_effect_(Tsk, commited(Type)) :-
  task_role_range(Tsk,ease_obj:'CommitedObject',Type).
action_effect_(Tsk, deposited(Type)) :-
  task_role_range(Tsk,ease_obj:'DepositedObject',Type).
action_effect_(Tsk, extracted(Type)) :-
  task_role_range(Tsk,ease_obj:'ExtractedObject',Type).
action_effect_(Tsk, included(Type)) :-
  task_role_range(Tsk,ease_obj:'IncludedObject',Type).

%% action_effects_apply(+Act,+Grounding) is det.
%
% @param Act An individual of type dul:'Action'
%
action_effects_apply(Act,Pairs) :-
  is_list(Pairs), !,
  dict_pairs(Grounding,_,Pairs),
  action_effects_apply(Act,Grounding).

action_effects_apply(Act,Grounding0) :-
  action_has_task(Act,Tsk),
  dict_pairs(Grounding0,_,Pairs0),
  % make sure groundings are participants in the action
  forall(
    member(_-X0,Pairs0),
    action_add_filler(Act,X0)
  ),
  %% instantiate created objects that are not created yet
  action_create_objects_(Act,Tsk,Grounding0,Pairs1),
  append(Pairs0,Pairs1,Pairs),
  dict_pairs(Grounding1,_,Pairs),
  %% apply effects based on roles of objects
  forall(
    get_dict(Concept,Grounding1,Filler),
    ( object_effects_apply_(Act,Grounding1,Concept,Filler) -> true ; (
      print_message(warning,
        runtime_error(cannot_apply_effects_of(Act,Concept)))
    ))
  ).

%%
object_effects_apply_(_Act,_Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'DestroyedObject'),!,
  current_time(Now),
  object_set_lifetime_end(Filler,Now).

object_effects_apply_(_Act,_Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'CreatedObject'),!,
  current_time(Now),
  object_set_lifetime_begin(Filler,Now).

object_effects_apply_(Act,_Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'AlteredObject'),!,
  get_altered_quality_(Concept,Filler,Quality), % TODO: better use parameter here!?!
  %action_grounding_(Grounding,ease_obj:'AchievedValue',Region),
  kb_triple(Act,ease_obj:hasAlterationResult,Region),
  quality_set_region_(Quality,Region).

object_effects_apply_(_Act,Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'CommitedObject'),!,
  action_grounding_(Grounding,ease_obj:'AlteredObject',Parent),
  object_add_part_(Parent,Filler).

object_effects_apply_(_Act,Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'IncludedObject'),!,
  action_grounding_(Grounding,ease_obj:'Container',Parent),
  object_add_content_(Parent,Filler).

object_effects_apply_(_Act,Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'ExtractedObject'),!,
  ( action_grounding_(Grounding,ease_obj:'AlteredObject',Parent);
    action_grounding_(Grounding,ease_obj:'Container',Parent);
    action_grounding_(Grounding,ease_obj:'Deposit',Parent)
  ),
  object_remove_deposit_(Parent,Filler),
  object_remove_link_(Parent,Filler),
  object_remove_part_(Parent,Filler),
  object_remove_content_(Parent,Filler).

object_effects_apply_(_Act,Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'LinkedObject'),!,
  forall(
    action_grounding_(Grounding,ease_obj:'LinkedObject',Linked),
    object_create_link_(Filler,Linked)
  ),
  forall(
    action_grounding_(Grounding,ease_obj:'CreatedObject',Parent),
    object_add_part_(Parent,Filler)
  ).

object_effects_apply_(_Act,Grounding,Concept,Filler) :-
  rdfs_individual_of(Concept,ease_obj:'DepositedObject'),!,
  action_grounding_(Grounding,ease_obj:'Deposit',Deposit),
  object_add_deposit_(Deposit,Filler).


		 /*******************************
		 *	helper		*
		 *******************************/

%%
get_altered_quality_type_(Concept,Quality_type) :-
  property_range(Concept,ease_obj:isTriggerDefinedIn,Affordance),
  property_range(Affordance,ease_obj:describesQuality,Quality_type),
  rdfs_subclass_of(Quality_type,dul:'Quality'),
  \+ rdf_equal(Quality_type,dul:'Quality'),!.

get_altered_quality_(Concept,Filler,Quality) :-
  get_altered_quality_type_(Concept,Quality_type),
  object_quality(Filler,Quality_type,Quality).

%%
action_grounding_(Grounding,Concept,Filler) :-
  get_dict(C,Grounding,Filler),
  rdfs_individual_of(C,Concept),!.

%%
action_create_objects_(Act,Tsk,Grounding,Pairs) :-
  findall(Role-Obj, (
    kb_triple(Tsk,dul:isTaskOf,Role),
    once((
      kb_type_of(Role,ease_obj:'CreatedObject'),
      property_range(Role,dul:classifies,Type),
      action_create_object_(Act,Grounding,Type,Obj)
    ))
  ), Pairs).

%%
action_create_object_(Act,Grounding,Type,Obj) :-
  action_participant(Act,Obj),
  kb_type_of(Obj,Type),
  get(Concept,Grounding,Obj),
  rdfs_individual_of(Concept,ease_obj:'CreatedObject'),!,
  fail.
action_create_object_(Act,_Grounding,Type,Obj) :-
  kb_create(Type,Obj),
  action_add_filler(Act,Obj).

%%
quality_set_region_(Quality,Region) :-
  kb_retract(Quality,dul:hasRegion,_),
  kb_assert(Quality,dul:hasRegion,Region).

%%
object_add_part_(Parent,Filler) :-
  kb_assert(Parent,dul:hasPart,Filler).
object_remove_part_(Parent,Filler) :-
  kb_retract(Parent,dul:hasPart,Filler).

%%
object_add_content_(Parent,Filler) :-
  kb_assert(Parent,ease_obj:containsObject,Filler).
object_remove_content_(Parent,Filler) :-
  kb_retract(Parent,ease_obj:containsObject,Filler).

%%
object_add_deposit_(Parent,Filler) :-
  kb_assert(Filler,ease_obj:isOntopOf,Parent).
object_remove_deposit_(Parent,Filler) :-
  kb_retract(Filler,ease_obj:isOntopOf,Parent).

%%
object_create_link_(L,L) :- !.
object_create_link_(L0,L1) :-
  kb_assert(L0,ease_obj:isLinkedTo,L1).
object_remove_link_(L,L) :- !.
object_remove_link_(L0,L1) :-
  kb_retract(L0,ease_obj:isLinkedTo,L1).
