/** <module> Reasoning about robot components and capabilities


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

:- module(srdl2,
    [
        action_feasible_on_robot/2,
        action_feasible_on_robot/3,
        missing_for_action/4,
        missing_cap_for_action/3,
        required_cap_for_action/2,
        missing_comp_for_action/3,
        required_comp_for_action/2,
        unsatisfied_restr_on_robot/3,
        unsatisfied_restr_on_robot/4,
        restricted_action_on_robot/3,
        robot_tf_prefix/2,
        robot_part_tf_prefix/2,
        cap_available_on_robot/2,
        comp_type_available/2,
        sub_component/2,
        succeeding_link/2
  ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('owl_parser')).
:- use_module(library('owl')).
:- use_module(library('rdfs_computable')).
:- use_module(library('knowrob_owl')).


:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).


:- rdf_meta
        action_feasible_on_robot(r,r),
        action_feasible_on_robot(r,t,r),
        missing_for_action(r,r,r,r),
        missing_cap_for_action(r,r,r),
        required_cap_for_action(r,r),
        missing_comp_for_action(r,r,r),
        required_comp_for_action(r,r),
        unsatisfied_restr_on_robot(r,r,r),
        unsatisfied_restr_on_robot(r,r,r,r),
        restricted_action_on_robot(r,r,r),
        robot_tf_prefix(r,r),
        robot_part_tf_prefix(r,r),
        cap_available_on_robot(r,r),
        comp_type_available(r,r),
        sub_component(r,r),
        succeeding_link(r,r).


        
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% TfPrefix

%% robot_part_tf_prefix(?RobotPart, ?TfPrefix).
%
% Checks if the Agent individual that belongs to this Part (URDF Link) has a tf Prefix assigned 
% TfPrefix defaults to '/'
%
% @param RobotPart   The part of the robot that should be checked(URDF Link)
% @param TfPrefix The TfPrefix
%       
        
robot_part_tf_prefix(RobotPart, TfPrefix) :-
  owl_individual_of(RobotPart,srdl2comp:'UrdfLink'),
  owl_individual_of(Robot, knowrob:'Agent-Generic'),
  sub_component(Robot, RobotPart),
  owl_has(Robot, srdl2comp:'tfPrefix', literal(TfPrefix)). 
  
robot_part_tf_prefix(_, '/').
%% robot_tf_prefix(?Robot, ?TfPrefix).
%
% Checks if an Agent individual  has a tf Prefix assigned 
% TfPrefix defaults to '/'
%
% @param RobotPart   The robot individual 
% @param TfPrefix The TfPrefix
%       
  
robot_tf_prefix(Robot, TfPrefix) :-
  owl_has(Robot, srdl2comp:'tfPrefix', literal(TfPrefix)).
  
robot_tf_prefix(_, '/').

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Actions

%% action_feasible_on_robot(?Action, ?Robot).
%% action_feasible_on_robot(?Action, ?ActionDescription, ?Robot).
%
% Verifies that an action is feasible on the given robot by making
% sure that neither capabilites nor components are missing.
%
% The 3-argument variant also checks restrictions on action parameters
% with the action instance described in ActionDescription.
%
% @param Action   Action class to be checked
% @param ActionDescription   Designator that includes some action parameters
% @param Robot   Robot instance to be checked
% 
action_feasible_on_robot(ActionConcept, Robot) :-
  \+ missing_cap_for_action(ActionConcept, Robot, _),
  \+ missing_comp_for_action(ActionConcept, Robot, _).

action_feasible_on_robot(ActionConcept, ActionDescription, Robot) :-
  action_feasible_on_robot(ActionConcept, Robot),
  unsatisfied_restr_on_robot(ActionDescription, Robot, [], ActionConcept).

%% missing_for_action(Action, Robot, MissingCaps, MissingComps).
%
% Determines all components and capabilites that are required by an action,
% but are not available on the given robot.
%
% @param Action   Action class to be checked
% @param Robot   Robot instance to be checked
% 
missing_for_action(Action, Robot, MissingCaps, MissingComps) :-
  missing_cap_for_action(Action, Robot, MissingCaps);
  missing_comp_for_action(Action, Robot, MissingComps).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Capabilities


%% missing_cap_for_action(Action, Robot, Cap) is nondet.
%
% Missing capabilites are required, but not available on the robot
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
% Capabilities required by an action and all of its sub-actions
%
% @param Action   Action class to be checked
% @param Cap     Capability required to perform the action
%

required_cap_for_action(Action, Cap) :-
  class_properties(Action, srdl2cap:'dependsOnCapability', Cap).

required_cap_for_action(Action, Cap) :-
  class_properties(Action, srdl2cap:'dependsOnCapability', Cp),
  class_properties(Cp, srdl2cap:'dependsOnCapability', Cap).

required_cap_for_action(Action, Cap) :-
  plan_subevents_recursive(Action, SubAction),
  class_properties(SubAction, srdl2cap:'dependsOnCapability', Cap).


%% cap_available_on_robot(Cap, Robot) is nondet.
%
% Check if a capability is available on a robot. This is the case if the capability
%
% 1) is asserted for this robot class
% 
% 2) is asserted for this robot instance
% 
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
% Missing components are required, but not available on the robot
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
% @param Super  Upper component
% @param Sub    Component that is part of the Super component
%

% Directly asserted sub-component (subComponent is not transitive because
% this would allow predecessor/successor loops
sub_component(Super, Sub) :-
  \+ owl_individual_of(Super, srdl2comp:'ComponentComposition'),
  owl_has(Super, srdl2comp:'subComponent', Sub).

% Transitive: successorInKinematicChain, which is transitive and allows
% to step over link/joint chains
sub_component(Super, Sub) :-
  \+ owl_individual_of(Super, srdl2comp:'ComponentComposition'),
  owl_has(Super, srdl2comp:'successorInKinematicChain', Sub).

% Handle component compositions: subcomponents are those between their
% baseLink and endLinks
%
% Note: Compositions are only considered as annotations, i.e. all sub-
%       components of this composition are supposed to already be part
%       of the main kinematic chain, thus the case distinction in the
%       beginning.
% 
sub_component(Super, Sub) :-
  owl_individual_of(Super, srdl2comp:'ComponentComposition'),
  owl_has(Super, srdl2comp:'baseLinkOfComposition', Base),
  owl_has(Super, srdl2comp:'endLinkOfComposition', End),
  sub_component(Base, Sub),
  sub_component(Sub, End).

%% succeeding_link(+BaseLink, +SucceedingLink) is nondet.
%
% Check if SucceedingLink is successor link of BaseLink
%
% @param BaseLink         The base UrdfLink
% @param SucceedingLink   The succeeding UrdfLink
%
succeeding_link(BaseLink, SucceedingLink) :-
  owl_has(BaseLink, srdl2comp:'succeedingJoint', ConnectionJoint),
  owl_has(ConnectionJoint, srdl2comp:'succeedingLink', SucceedingLink).


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


%% unsatisfied_restr_on_robot(+ActionDescription, +Robot, ?Unsatisfied) is nondet.
%% unsatisfied_restr_on_robot(+ActionDescription, +Robot, ?Unsatisfied, ?ActionConcept) is nondet.
%% unsatisfied_restr_on_robot(+ActionInstance, +Robot, ?Unsatisfied) is nondet.
%% unsatisfied_restr_on_robot(+ActionInstance, +Robot, ?Unsatisfied, ?ActionConcept) is nondet.
%
% Lists all unsatisfied restrictions on given action class or individual.
%
% @param ActionInstance A instance of knowrob:Action
% @param ActionDescription An entity description
% @param Robot An individual of knowrob:Robot
% @param Unsatisfied List of unsatidfied restrictions
% @param ActionConcept A specialization of knowrob:Action
%
unsatisfied_restr_on_robot(ActionDescription, Robot, Unsatisfied) :-
  unsatisfied_restr_on_robot(ActionDescription, Robot, Unsatisfied, _).
unsatisfied_restr_on_robot(ActionDescription, Robot, Unsatisfied, ActionConcept) :-
  % generate temporary OWL individual in order to perform standard OWL reasoning
  entity_description(ActionDescription), % FIXME: not existing!
  entity_assert(ActionInstance, ActionDescription),
  unsatisfied_restr_on_robot(ActionInstance, Robot, Unsatisfied, ActionConcept),
  entity_retract(ActionInstance), % FIXME: not existing!
  !.

unsatisfied_restr_on_robot(ActionInstance, Robot, Unsatisfied, ActionConcept) :-
  owl_individual_of(ActionInstance, knowrob:'Action'),
  % make sure the instantiation satisfies all actionability restrictions
  ground(ActionConcept)
  ->
  findall(R, (
    restricted_action_on_robot(ActionConcept, Robot, R), 
    \+ owl_individual_of(ActionInstance, R)
  ), Unsatisfied) ;
  findall(R, (
    restricted_action_on_robot(ActionInstance, Robot, R), 
    \+ owl_individual_of(ActionInstance, R)
  ), Unsatisfied).

%% restricted_action_on_robot(+ActionConcept, +Robot, ?RestrictedAction) is nondet.
%% restricted_action_on_robot(+ActionInstance, +Robot, ?RestrictedAction) is nondet.
%
% Lists all restrictions imposed on given action class or individual.
%
% @param ActionConcept A specialization of knowrob:Action
% @param ActionInstance A instance of knowrob:Action
% @param Robot An individual of knowrob:Robot
% @param RestrictedAction A restriction imposed on first ActionConcept/ActionInstance
%
restricted_action_on_robot(ActionConcept, Robot, RestrictedAction) :-
    rdfs_individual_of(ActionConcept, owl:'Class'), !,
    class_properties(Robot, knowrob:'actionable', RestrictedAction),
    owl_subclass_of(ActionConcept, RestrictedAction).

restricted_action_on_robot(ActionInstance, Robot, RestrictedAction) :-
    owl_individual_of(ActionInstance, knowrob:'Action'), !,
    class_properties(Robot, knowrob:'actionable', RestrictedAction),
    once((
      owl_individual_of(ActionInstance, ActionConcept),
      owl_subclass_of(ActionConcept, RestrictedAction)
    )).
