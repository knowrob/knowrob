/** <module> Prediction of action effects

  Copyright (C) 2011 Moritz Tenorth, 2016 Daniel Beßler
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

@author Moritz Tenorth, Daniel Beßler
@license BSD

*/
:- module(action_effects,
    [
      action_effects_apply/1,
      action_effect_apply/2,
      action_precondition_check/1,
      action_precondition_check/2,
      comp_hasEffect/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).
:- use_module(library('knowrob_temporal')).
:- use_module(library('owl')).


:- rdf_db:rdf_register_ns(knowrob,       'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(action_effects,'http://knowrob.org/kb/action-effects.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(swrl, 'http://www.w3.org/2003/11/swrl#', [keep(true)]).

:-  rdf_meta
    action_effects_apply(r),
    action_effects_apply(r,r),
    action_precondition_check(r),
    action_precondition_check(r,r),
    comp_hasEffect(r,r).

comp_hasEffect(Act, Effect) :-
  rdf_has(Effect, knowrob:swrlActionConcept, Cls),
  strip_literal_type(Cls,Cls_),
  once(owl_individual_of(Act, Cls_)).

knowrob_temporal:holds(Act, 'http://knowrob.org/kb/knowrob.owl#hasEffect', Effect, _) :-
  comp_hasEffect(Act, Effect).

action_effects_apply(Act) :-
  forall( comp_hasEffect(Act, Descr),
          action_effect_apply(Act, Descr) ).

action_effect_apply(Act,Descr) :-
  rdf_has(Descr, knowrob:swrlActionVariable, VarLiteral),
  strip_literal_type(VarLiteral, Var),
  rdf_swrl_project(Descr, [var(Var,Act)]).


action_precondition_check(Act) :-
  action_precondition_check(Act, _), !.

action_precondition_check(Act, Descr) :-
  rdf_has(Descr, knowrob:swrlActionVariable, VarLiteral),
  strip_literal_type(VarLiteral, Var),
  rdf_swrl_satisfied(Descr, [var(Var,Act)]).


% % % % % % % % % % % % % % % %
% Cutting off a piece
% @deprecated
%project_action_effects(Action) :-

  %owl_individual_of(Action, knowrob:'CuttingOffAPiece'),
  %\+ owl_has(Action, knowrob:outputsCreated, _),


  % TODO(DB): this is not accurate, pieces of things may have a slightly different class 
  %           then the input object. For example cutting a bread yields a slice of bread, not a new "bread".
  %           Also seems to be hard to express in SWRL. Will need to have annotations that map concepts
  %           to their slices concept so that this can generically be handled in rules.
  %owl_has(Action, knowrob:objectActedOn, Obj),
  %rdf_has(Obj, rdf:type, ObjType),
  %ObjType \= 'http://www.w3.org/2002/07/owl#NamedIndividual',!,

  %% new objects
  %rdf_instance_from_class(ObjType, knowrob_projection, Slice),

  %% new relations
  %rdf_assert(Action, knowrob:outputsRemaining, Obj, knowrob_projection),
  %rdf_assert(Action, knowrob:outputsCreated, Slice, knowrob_projection),

  %print(Obj),print(' -> '), print(Slice), print('\n').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Moving something to a location
%%

%% % % % % % % % % % % % % % % %
%% Putting something to some location
%% @deprecated
%project_action_effects(Action) :-

  %owl_individual_of(Action, knowrob:'PuttingSomethingSomewhere'),

  %owl_has(Action, knowrob:objectActedOn, Obj),
  %owl_has(Action, knowrob:toLocation, To),!,

  %% predict the object to be at the toLocation of the action
  %rdf_instance_from_class(knowrob:'ThoughtExperimenting', knowrob_projection, Pred),
  %rdf_assert(Pred, knowrob:'objectActedOn', Obj, knowrob_projection),

  %get_timepoint(NOW), % TODO: add predicted action duration
  %rdf_assert(Pred, knowrob:'startTime', NOW, knowrob_projection),

  % TODO(DB): this seems to be hacky. In general cool to project the pose of objects
  %           (also for grasping something!). Still object pose may be published in TF tree
  %           and is not explicitely modelled.
  %           Also note that this would effect nearly all manipulation actions (since they change pose of objects).
  %           Need a concept for combining pose info from different sources (e.g., overwrite last percept using effect projection)
  %((owl_individual_of(To, knowrob:'RotationMatrix'),!) -> (
    %% if toLocation is given as pose matrix, use this one
    %rdf_assert(Pred, knowrob:'eventOccursAt', To, knowrob_projection)
   %) ; (
    %% otherwise use the knowrob:orientation of the toLocation
    %rdf_triple(knowrob:orientation, To, ToPose),
    %rdf_assert(Pred, knowrob:'eventOccursAt', ToPose, knowrob_projection)
  %)),

  %print(Obj),print(' at location '), print(To), print('\n').

