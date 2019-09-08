
:- module(knowrob_action_effects,
    [
      action_effects_apply/2,
      action_effect/2
    ]).
/** <module> Reasoning about action effects.

KnowRob mainly uses OWL as its knowledge representation language.
While being very expressive OWL still has some limitations.
First, action effects may establish new relations between objects.
We use SWRL to express this (knowrob_common:swrl.pl).
Second, actions may create/destroy objects. This can further not 
be expressed in SWRL.
Here, we use the computable mechanism of KnowRob and a set of Prolog 
rules that compute processStared, outputCreated, ... relations.
This has the nice side-effect that we can use computed relations
in SWRL rules (i.e., relate what was created to something else).

@author Moritz Tenorth
@author Daniel Beßler
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('swrl')).
:- use_module(library('swrl_parser')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/action_model')).
:- use_module(library('knowrob/objects')).

:-  rdf_meta
    action_effect(r,t),
    action_effects_apply(r,r),
    comp_processStarted(r,r),
    comp_outputsCreated(r,r),
    task_role_range(r,r,r),
    assign_role_(r,r,r,r).

		 /*******************************
		 *	Action post-conditions	*
		 *******************************/

%% action_effect(?Tsk:iri, ?EffectTerm:term) is nondet
%
% Reasoning about which Task has a desired effect.
% EffectTerm is a Prolog term describing the effect:
%
%	| commited(Type)	| An object has been commited. 	|
%	| moved(Type)		| An object has been moved.	|
%	| operated(Type)	| An object has been operated.	|
%	| removed(Type)		| An object has been removed from a whole.	|
%	| transformed(Type)	| An object has been transformed.	|
%	| created(Type)		| An object has been created.	|
%	| destroyed(Type)	| An object has been destroyed.	|
%	| started(Type)		| A process has been started.	|
%	| stopped(Type)		| A process has been stopped.	|
%
action_effect(Act,Effect) :-
  action_has_task(Act,Tsk),
  action_effect_(Tsk,Effect).

action_effect(Tsk,Effect) :-
  action_effect_(Tsk,Effect).

action_effect_(Tsk, commited(Type)) :-
  task_role_range(Tsk,ease_obj:'CommitedObject',Type).
action_effect_(Tsk, moved(Type)) :-
  task_role_range(Tsk,ease_obj:'MovedObject',Type).
action_effect_(Tsk, operated(Type)) :-
  task_role_range(Tsk,ease_obj:'OperatedObject',Type).
action_effect_(Tsk, removed(Type)) :-
  task_role_range(Tsk,ease_obj:'RemovedObject',Type).
action_effect_(Tsk, transformed(Type)) :-
  task_role_range(Tsk,ease_obj:'TransformedObject',Type).
action_effect_(Tsk, created(Type)) :-
  task_role_range(Tsk,ease_obj:'CreatedObject',Type).
action_effect_(Tsk, destroyed(Type)) :-
  task_role_range(Tsk,ease_obj:'DestroyedObject',Type).

action_effect_(Tsk, started(ProcType)) :-
  property_cardinality(Tsk,ease:starts,ProcType,C,_), C>0,
  kb_type_of(ProcType,dul:'Process').
action_effect_(Tsk, stopped(ProcType)) :-
  property_cardinality(Tsk,ease:finishes,ProcType,C,_), C>0,
  kb_type_of(ProcType,dul:'Process').

%% action_effects_apply(+Act:iri)
%
% Apply all the effects that are known to apply if Act is performed successfully.
%
% @param Act Instance of knowrob:'Action'
%
action_effects_apply(Act,Tsk) :-
  event_end_time(Act,Stamp),
  % make sure cached computables are inferred
  forall(rdfs_computable_triple(knowrob:processStarted, Act, _), true),
  forall(rdfs_computable_triple(knowrob:outputsCreated, Act, _), true),
  % assign roles
  forall(rdf_has(Tsk,dul:isTaskOf,Role), assign_role(Act,Role)),
  % use swrl for projection of action effects
  forall(
    ( action_has_task(Act,Tsk), task_effect_rule(Tsk,Rule,Args) ),
    ( action_effect_rule_project(Act,Rule,Args) ; (
      print_message(warning, action_effect_failed(Act,Tsk,Args))
    ))
  ),
  % start/stop lifetime of objects
  forall(
    rdf(Act, knowrob:outputsCreated, Started),
    object_set_lifetime_begin(Started,Stamp)),
  forall(
    rdf(Act, knowrob:inputsDestroyed, Started),
    object_set_lifetime_end(Started,Stamp)),
  % start/stop processes
  forall(
    rdf(Act, knowrob:processStarted, Started),
    event_set_begin_time(Started,Stamp)),
  forall(
    rdf(Act, knowrob:processStopped, Stopped),
    event_set_end_time(Stopped,Stamp)),!.

%%
action_effect_rule_project(Act,Rule,Args) :-
  swrl_rule_arg_(Rule,Args,action_var,Var),
  swrl_project(Rule,[var(Var,Act)]).

%% 
task_effect_rule(Tsk,Rule,Args) :-
  rdfs_individual_of(Tsk,Tsk_type),
  rdf_split_url(_,TskName,Tsk_type),
  swrl_rule_arg_(Rule,Args,task,TskName).

swrl_rule_arg_(Rule,Args,Key,Value) :-
  % HACK
  swrl_parser:swrl_file_store(_,Rule,Args),
  get_dict(Key,Args,Value).

%%
%assign_role(Act,Role) :-
  %assign_role_(Act, Role, ease_obj:'CommitedObject', knowrob:?),!.
assign_role(Act,Role) :-
  assign_role_(Act,Role, ease_obj:'MovedObject', knowrob:objectTransported),!.
%assign_role(Act,Role) :-
  %assign_role_(Act, Role, ease_obj:'OperatedObject', knowrob:?),!.
assign_role(Act,Role) :-
  assign_role_(Act, Role, ease_obj:'RemovedObject', knowrob:objectRemoved),!.
%assign_role(Act,Role) :-
  %assign_role_(Act, Role, ease_obj:'TransformedObject', knowrob:?),!.
assign_role(Act,Role) :-
  assign_role_(Act, Role, ease_obj:'DestroyedObject', knowrob:inputsDestroyed),!.
assign_role(Act,Role) :-
  assign_role_(Act, Role, ease_obj:'Tool', knowrob:toolUsed),!.
assign_role(Act,Role) :-
  assign_role_(Act, Role, ease_obj:'Destination', knowrob:goalLocation),!.

assign_role_(Act,Role,RoleConcept,RoleRelation) :-
  kb_type_of(Role,RoleConcept),!,
  forall(kb_triple(Role,dul:classifies,Obj),(
    kb_retract(Act,dul:hasParticipant,Obj),
    kb_assert(Act,RoleRelation,Obj)
  )).

		 /*******************************
		 *	Computables		*
		 *******************************/

%% 
comp_processStarted(Act,Proc) :-
  action_effect(Act,started(ProcType)),
  kb_create(ProcType,Proc).

%% 
comp_outputsCreated(Act,Obj) :-
  action_effect(Act,created(ObjType)),
  kb_create(ObjType,Obj).
