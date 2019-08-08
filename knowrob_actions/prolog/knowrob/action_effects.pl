/*
  Copyright (C) 2011 Moritz Tenorth
  Copyright (C) 2018 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
:- module(action_effects,
    [
      action_effects_apply/1,
      action_effect/2,
      action_precondition_check/1,
      action_precondition_check/2
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
@license BSD
*/

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/swrl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/actions')).
:- use_module(library('knowrob/objects')).
:- use_module(library('knowrob/triple_memory')).

:-  rdf_meta
    action_effect(r,t),
    action_effects_apply(r),
    action_precondition_check(r),
    action_precondition_check(r,r),
    comp_processStarted(r,r),
    comp_outputsCreated(r,r).

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
  rdf_has(Act,dul:isClassifiedBy,Tsk),
  action_effect_(Tsk,Effect).

action_effect_(Tsk, commited(Type)) :-
  task_role(Tsk,ease_obj:'CommitedObject',Type).
action_effect_(Tsk, moved(Type)) :-
  task_role(Tsk,ease_obj:'MovedObject',Type).
action_effect_(Tsk, operated(Type)) :-
  task_role(Tsk,ease_obj:'OperatedObject',Type).
action_effect_(Tsk, removed(Type)) :-
  task_role(Tsk,ease_obj:'RemovedObject',Type).
action_effect_(Tsk, transformed(Type)) :-
  task_role(Tsk,ease_obj:'TransformedObject',Type).
action_effect_(Tsk, created(Type)) :-
  task_role(Tsk,ease_obj:'CreatedObject',Type).
action_effect_(Tsk, destroyed(Type)) :-
  task_role(Tsk,ease_obj:'DestroyedObject',Type).
action_effect_(Tsk, started(ProcType)) :-
  rdf_has(Tsk,ease:starts,X),
  rdfs_type_of(X,dul:'Process',ProcType).
action_effect_(Tsk, stopped(ProcType)) :-
  rdf_has(Tsk,ease:finishes,X),
  rdfs_type_of(X,dul:'Process',ProcType).

%% action_effects_apply(+Act:iri)
%
% Apply all the effects that are known to apply if Act is performed successfully.
%
% @param Act Instance of knowrob:'Action'
%
action_effects_apply(Act) :-
  event_end_time(Act,Stamp),
  % make sure cached computables are inferred
  forall(rdfs_computable_has(Act, knowrob:processStarted, _), true),
  forall(rdfs_computable_has(Act, knowrob:outputsCreated, _), true),
  % assign roles
  forall(
    ( rdf_has(Act,dul:isClassifiedBy,Tsk), rdf_has(Tsk,dul:isTaskOf,Role) ),
    assign_role(Act,Role)),
  % use swrl for projection of action effects
  forall(
    ( event_type(Act,Tsk), task_effect_rule(Tsk,Rule,Args) ),
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
  swrl_rule_arg(Rule,Args,action_var,Var),
  swrl_project(Rule,[var(Var,Act)]).

%% 
task_effect_rule(Tsk,Rule,Args) :-
  rdf_split_url(_,TskName,Tsk),
  swrl_rule_arg(Rule,Args,task,TskName).

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
  rdfs_individual_of(Role,RoleConcept),!,
  once(rdf(Act,rdf:type,_,G)),
  forall(rdf_has(Role,dul:classifies,Obj),(
    rdf_retractall(Act,dul:hasParticipant,Obj),
    rdf_assert(Act,RoleRelation,Obj,G)
  )).

		 /*******************************
		 *	Action pre-conditions	*
		 *******************************/

%% action_precondition_check(+Act:iri).
%% action_precondition_check(+Act:iri,+Effect:iri).
%
% True if Act is an action without unsatisfied preconditions, or
% if Effect is a satisfied precondition of Act.
%
% @param Act Instance of knowrob:'Action'
% @param Effect RDF description of SWRL action effect rule
%
% TODO: this seems strange. revise it
action_precondition_check(Act) :-
  action_precondition_check(Act, _), !.

action_precondition_check(Act, Effect) :-
  event_type(Act,Tsk),
  task_effect_rule(Tsk,Rule,Args),
  swrl_rule_arg(Rule,Args,task,TskName)
  swrl_satisfied(Rule,[var(Var,Act)]).

		 /*******************************
		 *	Computables		*
		 *******************************/

%% 
comp_processStarted(Act,Proc) :-
  action_effect(Act,started(ProcType)),
  rdfs_instance_from_class(ProcType,Proc).

%% 
comp_outputsCreated(Act,Obj) :-
  action_effect(Act,created(ObjType)),
  rdfs_instance_from_class(ObjType,Obj).
