% NOTE: this is currently disabled because the role taxonomy
%        is not yet fixed in SOMA, the module should be re-implemented
%        once this is done.
:- module(reasoning_roles,
    [
    ]).

%:- module(reasoning_roles,
    %[
      %action_effects_apply/2
    %]).
%/** <module> Reasoning about action effects.

%The effects of actions on objects are derived from the roles the objects play during the action. For example, whenever an object plays the role of being the *AlteredObject* one of its qualities is being altered to some level. Hence the effect of the action is an alteration of the region of the quality which may be automatically updated by KnowRob after the action has been executed. Other distinct cases are objects being destroyed or created in which case KnowRob updates the lifetime of them, and objects being combined, included, or excluded in which case KnowRob may update the compositional relations between these objects.

%@author Moritz Tenorth
%@author Daniel Beßler
%*/

%:- use_module(library('semweb/rdfs')).
%:- use_module(library('semweb/rdf_db')).

%:- use_module(library('knowrob/model/Event'),  [ event_participant/3 ]).
%:- use_module(library('knowrob/model/Action'), [ action_has_task/2,
                                                 %action_add_filler/2 ]).
%:- use_module(library('knowrob/model/Task'),   [ task_role_range/3,
                                                 %task_parameter_range/3 ]).
%:- use_module(library('knowrob/model/Object'), [ object_quality/3,
                                                 %object_set_lifetime_begin/2,
                                                 %object_set_lifetime_end/2 ]).

%:-  rdf_meta
    %action_effects_apply(r,t),
    %action_grounding_(r,r,r,r).



%% task_effect(?EventType, ?Effect) is nondet
%
% Relates an action or task to roles that imply change,
% and that need to be taken by some object when the task is executed.
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
% @param ActOrTsk An individual of type dul:'Action' or dul:'Task', or a subclass of dul:'Task'.
%
/*
task_effect(Tsk,Effect) :-
	ground(Effect)
	->	once(task_effect_(Tsk,Effect))
	;	task_effect_(Tsk,Effect).

task_effect_(Tsk, created(Type))   :- task_role_range(Tsk,soma:'CreatedObject',Type).
task_effect_(Tsk, destroyed(Type)) :- task_role_range(Tsk,soma:'DestroyedObject',Type).
task_effect_(Tsk, linked(Type))    :- task_role_range(Tsk,soma:'LinkedObject',Type).
task_effect_(Tsk, commited(Type))  :- task_role_range(Tsk,soma:'CommitedObject',Type).
task_effect_(Tsk, deposited(Type)) :- task_role_range(Tsk,soma:'DepositedObject',Type).
task_effect_(Tsk, extracted(Type)) :- task_role_range(Tsk,soma:'ExtractedObject',Type).
task_effect_(Tsk, included(Type))  :- task_role_range(Tsk,soma:'IncludedObject',Type).

task_effect_(Tsk, altered(Type,QualityType)) :-
	task_role_range(Tsk,soma:'AlteredObject',Type),
	has_parameter_range(Tsk,soma:'Setpoint',Region),
	get_altered_quality_type_(Type,Region,QualityType).

%%
get_altered_quality_type_(Concept,Region,Quality_type) :-
	get_altered_quality_type__(Concept,Region,Quality_type),
	\+ rdf_equal(Quality_type,dul:'Quality'),!.

get_altered_quality_type__(_Concept,Region,Quality_type) :-
	holds(Region, dul:isRegionFor, only(Quality_type)).

get_altered_quality_type__(Concept,_Region,Quality_type) :-
	holds(Concept, soma:isTriggerDefinedIn, only(Affordance)),
	subclass_of_expr(Affordance,only(soma:describesQuality,Quality_type)).
*/


%%% action_effects_apply(+ActOrTsk::iri,+Grounding::dict) is det.
%%
%% Some action effects may imply new relations, for example, that
%% one object is part of another after the action has been performed.
%% Consequently, *action_effects_apply* maintains these relations automatically
%% based on the roles objects play during the action.
%% Note that this predicate should be used carefully as it may create new entity
%% symbols in case objects were created during the action.
%%
%%    action_effects_apply(pancake:'CrackingAnEgg',
%%      _{'DestroyedObject': 'Egg_0'})
%%
%% @param ActOrTsk An individual of type dul:'Action' or dul:'Task', or a subclass of dul:'Task'.
%% @param Grounding A dict that encodes a mapping from roles and parameters (key) to objects and regions (value) that are classified by the role or parameter during the action.
%%
%action_effects_apply(Act,Pairs) :-
  %is_list(Pairs), !,
  %dict_pairs(Grounding,_,Pairs),
  %action_effects_apply(Act,Grounding).

%action_effects_apply(Act,Grounding0) :-
  %action_has_task(Act,Tsk),!,
  %dict_pairs(Grounding0,_,Pairs0),
  %% make sure groundings are participants in the action
  %forall(
    %member(_-X0,Pairs0),
    %action_add_filler(Act,X0)
  %),
  %%% instantiate created objects that are not created yet
  %action_create_objects_(Act,Tsk,Grounding0,Pairs1),
  %append(Pairs0,Pairs1,Pairs),
  %dict_pairs(Grounding1,_,Pairs),
  %%% apply effects based on roles of objects
  %get_time(Now),
  %forall(
    %get_dict(Concept,Grounding1,Filler),
    %( object_effects_apply_(Tsk,Grounding1,Concept,Filler,Now) -> true ; (
      %print_message(warning,
        %runtime_error(cannot_apply_effects_of(Act,Concept)))
    %))
  %).

%%%
%object_effects_apply_(_Tsk,_Grounding,Concept,Filler,Now) :-
  %rdfs_individual_of(Concept,soma:'DestroyedObject'),!,
  %object_set_lifetime_end(Filler,Now).

%object_effects_apply_(_Tsk,_Grounding,Concept,Filler,Now) :-
  %rdfs_individual_of(Concept,soma:'CreatedObject'),!,
  %object_set_lifetime_begin(Filler,Now).

%object_effects_apply_(Tsk,Grounding,Concept,Filler,_) :-
  %rdfs_individual_of(Concept,soma:'AlteredObject'),!,
  %action_grounding_(Tsk,Grounding,soma:'Setpoint',Region),
  %get_altered_quality_(Concept,Filler,Quality,Region),
  %quality_set_region_(Quality,Region).

%object_effects_apply_(Tsk,Grounding,Concept,Filler,Now) :-
  %rdfs_individual_of(Concept,soma:'CommitedObject'),!,
  %( action_grounding_(Tsk,Grounding,soma:'AlteredObject',Parent) ;
    %action_grounding_(Tsk,Grounding,soma:'CreatedObject',Parent)
  %),
  %object_add_part_(Parent,Filler),
  %object_set_lifetime_end(Filler,Now).

%%object_effects_apply_(Tsk,Grounding,Concept,Filler,Now) :-
  %%rdfs_individual_of(Concept,soma:'TransformedObject'),!,
  %%fail.

%object_effects_apply_(Tsk,Grounding,Concept,Filler,_) :-
  %rdfs_individual_of(Concept,soma:'IncludedObject'),!,
  %action_grounding_(Tsk,Grounding,soma:'Container',Parent),
  %object_add_content_(Parent,Filler).

%object_effects_apply_(Tsk,Grounding,Concept,Filler,_) :-
  %rdfs_individual_of(Concept,soma:'ExtractedObject'),!,
  %( action_grounding_(Tsk,Grounding,soma:'AlteredObject',Parent);
    %action_grounding_(Tsk,Grounding,soma:'Container',Parent);
    %action_grounding_(Tsk,Grounding,soma:'Deposit',Parent)
  %),
  %object_remove_deposit_(Parent,Filler),
  %object_remove_link_(Parent,Filler),
  %object_remove_part_(Parent,Filler),
  %object_remove_content_(Parent,Filler).

%object_effects_apply_(Tsk,Grounding,Concept,Filler,_) :-
  %rdfs_individual_of(Concept,soma:'LinkedObject'),!,
  %forall(
    %action_grounding_(Tsk,Grounding,soma:'LinkedObject',Linked),
    %object_create_link_(Filler,Linked)
  %),
  %forall(
    %action_grounding_(Tsk,Grounding,soma:'CreatedObject',Parent),
    %object_add_part_(Parent,Filler)
  %).

%object_effects_apply_(Tsk,Grounding,Concept,Filler,_) :-
  %rdfs_individual_of(Concept,soma:'DepositedObject'),!,
  %action_grounding_(Tsk,Grounding,soma:'Deposit',Deposit),
  %object_add_deposit_(Deposit,Filler).

%object_effects_apply_(_,_,_,_,_).


		 %/*******************************
		 %*	helper		*
		 %*******************************/

%%%
%get_altered_quality_type_(Concept,Region,Quality_type) :-
  %get_altered_quality_type__(Concept,Region,Quality_type),
  %rdfs_subclass_of(Quality_type,dul:'Quality'),
  %\+ rdf_equal(Quality_type,dul:'Quality'),!.

%get_altered_quality_type__(_Concept,Region,Quality_type) :-
  %property_range(Region,dul:isRegionFor,Quality_type).
%get_altered_quality_type__(Concept,_Region,Quality_type) :-
  %property_range(Concept,soma:isTriggerDefinedIn,Affordance),
  %property_range(Affordance,soma:describesQuality,Quality_type).

%get_altered_quality_(Concept,Object,Quality,Region) :-
  %get_altered_quality_type_(Concept,Region,Quality_type),
  %object_quality(Object,Quality_type,Quality).

%%%
%action_grounding_(_Tsk,Grounding,Concept,Filler) :-
  %get_dict(C,Grounding,Filler),
  %rdfs_individual_of(C,Concept),!.
%action_grounding_(Tsk,_Grounding,Concept,Filler) :-
  %( kb_triple(Tsk,dul:isTaskOf,X);
    %kb_triple(Tsk,dul:hasParameter,X) ),
  %kb_type_of(X,Concept),
  %kb_triple(X,dul:classifies,Filler).

%%%
%action_create_objects_(Act,Tsk,Grounding,Pairs) :-
  %findall(Role-Obj, (
    %kb_triple(Tsk,dul:isTaskOf,Role),
    %once((
      %kb_type_of(Role,soma:'CreatedObject'),
      %property_range(Role,dul:classifies,Type),
      %action_create_object_(Act,Grounding,Type,Obj)
    %))
  %), Pairs).

%%%
%action_create_object_(Act,Grounding,Type,Obj) :-
  %event_participant(Act,Obj,Type),
  %get_dict(Concept,Grounding,Obj),
  %rdfs_individual_of(Concept,soma:'CreatedObject'),!,
  %fail.
%action_create_object_(Act,_Grounding,Type,Obj) :-
  %kb_create(Type,Obj),
  %action_add_filler(Act,Obj).

%%%
%quality_set_region_(Quality,Region) :-
  %kb_retract(Quality,dul:hasRegion,_),
  %kb_assert(Quality,dul:hasRegion,Region).

%%%
%object_add_part_(Parent,Filler) :-
  %kb_assert(Parent,dul:hasPart,Filler).
%object_remove_part_(Parent,Filler) :-
  %kb_retract(Parent,dul:hasPart,Filler).

%%%
%object_add_content_(Parent,Filler) :-
  %kb_assert(Parent,soma:containsObject,Filler).
%object_remove_content_(Parent,Filler) :-
  %kb_retract(Parent,soma:containsObject,Filler).

%%%
%object_add_deposit_(Parent,Filler) :-
  %kb_assert(Filler,soma:isOntopOf,Parent).
%object_remove_deposit_(Parent,Filler) :-
  %kb_retract(Filler,soma:isOntopOf,Parent).

%%%
%object_create_link_(L,L) :- !.
%object_create_link_(L0,L1) :-
  %kb_assert(L0,soma:isLinkedTo,L1).
%object_remove_link_(L,L) :- !.
%object_remove_link_(L0,L1) :-
  %kb_retract(L0,soma:isLinkedTo,L1).
