%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides methods for reasoning about action descriptions
%% in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


:- module(knowrob_actions,
    [
      plan_subevents/2,
      plan_subevents_recursive/2,
      plan_objects/2,
      action_objectActedOn/2,
      action_toLocation/2,
      action_fromLocation/2,
      matching_actions/2,
      compare_actions_partial_order/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).


:- rdf_meta
      plan_subevents(r,-),
      plan_subevents_recursive(r,r),
      plan_objects(r,r),
      action_objectActedOn(r,r),
      action_toLocation(r,r),
      action_fromLocation(r,r),
      matching_actions(r,r),
      compare_actions_partial_order(-,r,r).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).


%% plan_subevents(+Plan, ?SubEvents) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% TODO: unify with plan_subevents_recursive (return list or single instances)
%
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents(Plan, SubEvents) :-
  findall(SubAction, (class_properties(Plan, knowrob:subAction, SubAction)), Sub),
  predsort(compare_actions_partial_order, Sub, SubEvents).



%% plan_subevents_recursive(+Plan, ?SubEvents) is semidet.
%
% Recursively read all sub-action classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan      Plan identifier
% @param SubEvents Sub-events of the plan
%
plan_subevents_recursive(Plan, SubAction) :-
    class_properties(Plan, knowrob:subAction, SubAction).

plan_subevents_recursive(Plan, SubAction) :-
    class_properties(Plan, knowrob:subAction, Sub),
    Sub \= Plan,
    plan_subevents_recursive(Sub, SubAction).



%% plan_objects(+Plan, -Objects) is semidet.
%
% Read all objects mentioned in sub-actions of the imported plan
%
% @param Plan Plan identifier
% @param SubEvents List of objects of the plan
plan_objects(Plan, Objects) :-
  plan_subevents(Plan, SubEvents),
  findall(Obj,
    (member(SubEvent, SubEvents),
     action_objectActedOn(SubEvent, Obj)), Objects).


%% action_objectActedOn(?Action, ?Object) is nondet.
%
% Reads the objectActedOn for a TBOX action description
%
action_objectActedOn(Action, Object) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction(knowrob:'objectActedOn', some_values_from(Object))).
action_objectActedOn(Action, Object) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction(knowrob:'objectActedOn', some_values_from(Object))).

%% action_toLocation(?Action, ?Loc) is nondet.
%
% Reads the toLocation for a TBOX action description
%
action_toLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction(knowrob:'toLocation', some_values_from(Loc))).
action_toLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction(knowrob:'toLocation', some_values_from(Loc))).

%% action_fromLocation(?Action, ?Loc) is nondet.
%
% Reads the fromLocation for a TBOX action description
%
action_fromLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_direct_subclass_of(Sup, Sup2),
        owl_restriction(Sup2,restriction(knowrob:'fromLocation', some_values_from(Loc))).
action_fromLocation(Action, Loc) :-
        owl_direct_subclass_of(Action, Sup),
        owl_restriction(Sup,restriction(knowrob:'fromLocation', some_values_from(Loc))).



%% matching_actions(?Plan, ?Act) is semidet.
%
% Search for action instances that fit the classes described in the imported plan
%
% @param Plan Plan identifier
% @param Act Matching actions
matching_actions(Plan, Act) :-
  plan_subevents(Plan, SubEvents),
  rdf_has(Act, rdf:type, knowrob:'PuttingSomethingSomewhere'),
  member(ActCl, SubEvents),
  owl_individual_of(Act, ActCl).


%% compare_actions_partial_order(-Rel, +Act1, +Act2) is semidet.
%
% Compare predicate to be used in predsort for sorting a list of actions
% based on partial-order constraints
%
% Checks if there is an ordering constraint that has these two actions as before/after
% TODO: can we check if these constraints belong to the current task?
%
compare_actions_partial_order('<', Act1, Act2) :-
  owl_has(Constraint, knowrob:occursBeforeInOrdering, Act1),
  owl_has(Constraint, knowrob:occursAfterInOrdering, Act2),!.

compare_actions_partial_order('>', Act1, Act2) :-
  owl_has(Constraint, knowrob:occursBeforeInOrdering, Act2),
  owl_has(Constraint, knowrob:occursAfterInOrdering, Act1),!.

% default: keep ordering if there are no matching ordering constraints
compare_actions_partial_order('<', _, _).
