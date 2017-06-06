/** <module> Methods for reasoning about object changes caused by actions
  
  Copyright (C) 2011 Moritz Tenorth
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

@author Moritz Tenorth
@license BSD

*/

%% TODO DB: looks like code graveyard here. no useful predicate exported.

:- module(object_change,
    [
      action_effects/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('rdfs_computable')).
:- use_module(library('owl_parser')).
:- use_module(library('knowrob_owl')).

:- use_module(library('action_effects')).
:- use_module(library('process_effects')).


:- owl_parse('package://knowrob_actions/owl/object-change.owl').
% :- owl_parse('package://knowrob_actions/owl/pancake-making.owl').

:- rdf_db:rdf_register_ns(knowrob,      'http://knowrob.org/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://knowrob.org/kb/object-change.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake, 'http://knowrob.org/kb/pancake-making.owl#', [keep(true)]).

:-  rdf_meta
        project_and_debug(r,r,r),
        project_action_class(r,r,r),
        add_subactions_for_action(r, ?),
        action_inputs(r,r),
        action_missing_inputs(r,r),
        action_outputs(r,r),
        resource_provided_by_actionseq(r, ?),
        resource_available(r),
        action_effects(r,r).


%% project_and_debug(+Plan, -OrigActionSeq, -ResultActSeq)
%
% Project a plan, infer actions that are missing in the original
% action sequence, and add these actions to the plan.
%
% @param Plan           Plan specification as action class with subAction descriptions
% @param OrigActionSeq  Sequence of subActions of the Plan
% @param ResultActSeq   Debugged action sequence, including actions that have been added
%                       in order to match the input specifications of all actions
%
project_and_debug(Plan, OrigActionSeq, ResultActSeq) :-

  % read action seq
  knowrob_actions:plan_subevents(Plan, OrigActionSeq),

  integrate_additional_actions(OrigActionSeq, ResultActSeq).
  % TODO: check if end result is ok
    % if yes, finish
    % if not: - check if inputs can be provided by another action
    %         - add this action at the earliest stage where its inputs are ok
    %         - remove projection results, start projection again




%% integrate_additional_actions(+ActSeq, -ResultActSeq)
%
% add additional actions (required to make an intermediate
% action possible) just before this action)
%
% @param ActSeq         Sequence of action classes (possibly incomplete)
% @param ResultActSeq   Resulting sequence, with additional actions integrated into ActSeq,
%                       so that all inputs of an action are filled with either existing objects
%                       or by the output of another action
%
integrate_additional_actions([],[]).
integrate_additional_actions([A|ActSeq], ResultActSeq) :-

  add_subactions_for_action(A, AddActions),
  project_action_class(A, _, _),!,
  integrate_additional_actions(ActSeq, RestActSeq),
  append(AddActions, [A], ResultActSeq1),
  append(ResultActSeq1, RestActSeq, ResultActSeq).



%% project_action_class(+Action, -Inst, -PostActors)
%
% create instances for the plan
%
% @param Action       Action class that is to be projected
% @param Inst         Generated action instance
% @param PostActors   Created outputs
%
project_action_class(Action, Inst, PostActors) :-

  % create instance of the action
  rdf_instance_from_class(Action, knowrob_projection, Inst),

  % startTime: now
  get_timepoint(NOW),
  rdf_assert(Inst, knowrob:'startTime', NOW, knowrob_projection),

  % bind the action properties from the class description to object instances
  findall([P,OT], ( class_properties(Action, P, OT)), PrObjTs),

  findall(ObjInst, (member([P, OT], PrObjTs),
                    obj_inst(OT, ObjInst),
                    rdf_assert(Inst, P, ObjInst, knowrob_projection)), _),

  % project action effects
  (action_effects(Inst, PostActors);true).


%% action_effects(+Action, -PostActors)
%
% Perform projection of action effects and those of processes that have
% been triggered as indirect effects
%
% @param Action     Action instance
% @param PostActors Effects of the action
%
action_effects(Action, PostActors) :-

  % project what is happening when performing the action
  project_action_effects(Action), % FIXME: use new action projection predicate

  % check for processes that got triggered
  (project_process_effects;true),

  rdf_has(Action, knowrob:postActors, PostActors).


obj_inst(OT, ObjInst) :-
  owl_individual_of(ObjInst, OT),!. % just take the first instance of the resp. kind



%% add_subactions_for_action(+Action, -SubActions)
%
% An action is possible if all prerequisites are fulfilled
% or if all of the missing ones can be provides by possible actions
%
% @param Action     Action whose availability is to be checked
% @param SubActions List of additional actions that need to be performed before
%                   Action to generate the missing inputs (possibly empty)
%
add_subactions_for_action(Action, []) :-
  action_missing_inputs(Action, []),!.

add_subactions_for_action(Action, SubActions) :-

  action_missing_inputs(Action, Ms),
  setof(Sub, ((member(M, Ms), resource_provided_by_actionseq(M, Sub)) ; fail), Subs),
  flatten(Subs, SubActions).




%% action_inputs(+Action, -Input)
%
% Required inputs for an action
%
% @param Action   Action class
% @param Input    Input linked via a preActors restriction
%
action_inputs(Action, Input) :-
  class_properties(Action, knowrob:'preActors', Input).



%% action_missing_inputs(+Action, -Missing)
%
% Missing inputs of an action (required, but not available)
%
% @param Action   Action class
% @param Missing  Input linked via a preActors restriction, but not available
%
action_missing_inputs(Action, Missing) :-
  findall(Pre, (action_inputs(Action, Pre), \+ resource_available(Pre)), Missing).


%% action_outputs(+Action, -Output)
%
% Outputs of an action
%
% @param Action   Action class
% @param Output   Output linked via a postActors restriction
%
action_outputs(Action, Output) :-
  class_properties(A, knowrob:'postActors', Output),
  owl_subclass_of(Action, A).
%TODO: check class subsumption (allow more complex requirements)



%% resource_available(+Resource)
%
% Resource is available (TODO: check destruction etc)
%
% @param Resource Resource whose availability is to be checked (e.g. object class, check if instance of that class exists)
%
resource_available(Resource) :-
  owl_individual_of(_, Resource).


%% resource_provided_by_actionseq(Resource, [SubActions|SubAction])
%
% Resouce can be provided by a sequence of SubActions
%
% @param Resource     Resource whose availability is to be checked (e.g. object class)
% @param SubActions   List of action classes that need to be performed in order to obtain Resource
%
resource_provided_by_actionseq(Resource, [SubActions|SubAction]) :-
  action_outputs(SubAction, Resource),
  add_subactions_for_action(SubAction, SubActions).
