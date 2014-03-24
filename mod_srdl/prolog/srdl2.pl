%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module is a complete re-implementation of the SRDL language
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


:- module(srdl2,
    [
        action_feasible_on_robot/2,
        missing_for_action/4,
        missing_cap_for_action/3,
        required_cap_for_action/2,
        missing_comp_for_action/3,
        required_comp_for_action/2,
        cap_available_on_robot/2,
        comp_type_available/2,
        sub_component/2
  ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).


:- owl_parser:owl_parse('../owl/srdl2-action.owl', false, false, true).
:- owl_parser:owl_parse('../owl/PR2.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://ias.cs.tum.edu/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://ias.cs.tum.edu/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://ias.cs.tum.edu/kb/srdl2-cap.owl#', [keep(true)]).


:- rdf_meta
        action_feasible_on_robot(r,r),
        missing_for_action(r,r,r,r),
        missing_cap_for_action(r,r,r),
        required_cap_for_action(r,r),
        missing_comp_for_action(r,r,r),
        required_comp_for_action(r,r),
        cap_available_on_robot(r,r),
        comp_type_available(r,r),
        sub_component(r,r).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Actions

action_feasible_on_robot(Action, Robot) :-
  \+ missing_cap_for_action(Action, Robot, _),
  \+ missing_comp_for_action(Action, Robot, _).


missing_for_action(Action, Robot, MissingCaps, MissingComps) :-
  missing_cap_for_action(Action, Robot, MissingCaps);
  missing_comp_for_action(Action, Robot, MissingComps).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Capabilities


%% missing_cap_for_action(Action, Robot, Cap) is nondet.
%
% missing capabilites are required, but not available on the robot
%
% @param Action   Action class to be checked
% @param Robot   Robot instance to be checked
% @param Cap     Capability required to perform the action
%

missing_cap_for_action(Action, Robot, Cap) :-
  required_cap_for_action(Action, Cap),
  \+ cap_available_on_robot(Cap, Robot).


%% required_cap_for_action(Action, Cap) is nondet.
%
% capabilities required by an action and all of its sub-actions
%
% @param Action   Action class to be checked
% @param Cap     Capability required to perform the action
%

required_cap_for_action(Action, Cap) :-
  class_properties(Action, srdl2cap:'dependsOnCapability', Cap).

required_cap_for_action(Action, Cap) :-
  plan_subevents_recursive(Action, SubAction),
  class_properties(SubAction, srdl2cap:'dependsOnCapability', Cap).


%% cap_available_on_robot(Cap, Robot) is nondet.
%
% Check if a capability is available on a robot. This is the case if the capability
%
% 1) is asserted for this robot class
% 2) is asserted for this robot instance
% 3) depends only on available components and sub-capabilites
%
% @param Cap   Capability class to be checked
% @param Robot Robot instance
%

% capability asserted for robot instance
cap_available_on_robot(Cap, Robot) :-
    owl_individual_of(Robot, knowrob:'Robot'),
    owl_has(Robot, srdl2cap:'hasCapability', SubCap),
    owl_subclass_of(SubCap, Cap).

% capability asserted for robot class
cap_available_on_robot(Cap, Robot) :-
    owl_subclass_of(RobotClass, knowrob:'Robot'),
    rdfs_individual_of(Robot, RobotClass),
    class_properties(RobotClass, srdl2cap:'hasCapability', SubCap),

    % If sub-properties are available, their super-capabilites are also
    % available. Make sure, however, not to scale beyond 'Capability'.
    owl_subclass_of(SubCap, Cap),
    owl_subclass_of(Cap, srdl2cap:'Capability'),
    \+ rdf_equal(Cap, srdl2cap:'Capability').

% capability depends only on available components or capabilities
cap_available_on_robot(Cap, Robot) :-

    owl_individual_of(Robot, knowrob:'Robot'),
    owl_subclass_of(Cap, srdl2cap:'Capability'),
    \+ rdf_equal(Cap, srdl2cap:'Capability'),

    forall( class_properties(Cap, srdl2comp:'dependsOnComponent', CompT),
            comp_type_available(Robot, CompT) ),

    forall( class_properties(Cap, srdl2cap:'dependsOnCapability', SubCap),
            cap_available_on_robot(SubCap, Robot) ).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Components

%% missing_comp_for_action(Action, Robot, Comp) is nondet.
%
% missing components are required, but not available on the robot
%
% @param Action  Action class to be checked
% @param Robot   Robot instance to be checked
% @param Comp    Component required to perform the action
%

missing_comp_for_action(Action, Robot, Comp) :-
  required_comp_for_action(Action, Comp),
  \+ comp_type_available(Robot, Comp).



%% required_comp_for_action(Action, Comp) is nondet.
%
% Components that are either directly required by an action and
% all of its sub-actions, or indirectly required by required
% capabilities
%
% @param Action   Action class to be checked
% @param Comp     Component required to perform the action
%

% components directly required by an action and all of its sub-actions
required_comp_for_action(Action, Comp) :-
  class_properties(Action, srdl2comp:'dependsOnComponent', Comp).

required_comp_for_action(Action, Comp) :-
  plan_subevents_recursive(Action, SubAction),
  class_properties(SubAction, srdl2comp:'dependsOnComponent', Comp).

% components indirectly required by required capabilities
required_comp_for_action(Action, Comp) :-
  required_cap_for_action(Action, Cap),
  class_properties(Cap, srdl2comp:'dependsOnComponent', Comp).


%% comp_type_available(+Super, +SubT) is nondet.
%
% Check if Super has a sub-component Sub of type SubT
% (i.e. if a component of that type is available as part of Super)
%
% @param Super  Upper component
% @param SubT   Type of a Component that is part of the Super component
%
comp_type_available(Super, SubT) :-
    sub_component(Super, Sub),
    owl_individual_of(Sub, SubT).


%% sub_component(?Super, ?Sub) is nondet.
%
% Recursively read all sub-components of a robot or a component
% 
% IMPORTANT: 
%
% @param Super  Upper component
% @param Sub    Component that is part of the Super component
%
sub_component(Super, Sub) :-  % direct or transitive subComponent
  owl_has(Super, srdl2comp:'subComponent', Sub);
  owl_has(Super, srdl2comp:'successorInKinematicChain', Sub).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% OWL/ DL Predicates


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

