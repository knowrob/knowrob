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
      action_check_preconditions/1,
      action_project_effects/1,
      action_project_effects/2,
      action_pddl_description/2
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

:-  rdf_meta
    action_check_preconditions(r),
    action_project_effects(r),
    action_project_effects(r,?).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Action precondition checking based on OWL descriptions
%

action_check_preconditions(_) :-
  true. % TODO: implement

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Action effect projection based on OWL description of effects
%

%% action_project_effects(+Action)
%% action_project_effects(+Action, -Effects)
%
% Project the effects of @Action into the KB.
% This boils down to manipulation of the rdf triple store.
% Since we want to reason about past states we generally use
% fluent properties and never retract from the KB.
% The projection method uses the action-effect and fluent
% ontologies which are part of the KnowRob core KB.
% Preconditions are not checked.
%
% @Action The action of which effects should be projected in the KB
% @Effects Descriptions of projected effects
%
%action_project_effects(Action) :- fail.
%action_project_effects(Action, Effects) :- fail.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% OWL to PDDL
%

action_pddl_description(Action_OWL, Action_PDDL) :-
  fail.


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

