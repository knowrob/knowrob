/*
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
*/

:- module(srdl2,
    [
        action_feasible_on_robot/2,
        action_feasible_on_robot/3,
        action_feasible_with_components/4,
        missing_for_action/4,
        missing_cap_for_action/3,
        missing_comp_for_action/3,
        required_cap_for_action/2,
        required_comp_for_action/2,
        insufficient_comp_for_action/4,
        unsatisfied_restr_for_action/5,
        unsatisfied_restr_for_required_comp/5,
        unsatisfied_restr_for_comp/5,
        cap_available_on_robot/2,
        comp_type_available/2,
        comp_restricted_action/3,
        comp_installable_for_action/4,
        comp_installable_on_robot/3,
        comp_baselink_pose/2,
        comp_baselink_pose/3,
        sub_component/2,
        succeeding_link/2,
        robot_tf_prefix/2,
        robot_part_tf_prefix/2,
        srdl_inFieldOfView/2,
        srdl_inFieldOfView/3
  ]).
/** <module> Reasoning about robot components and capabilities

  @author Moritz Tenorth
  @license BSD
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/action_planning')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://knowrob.org/kb/srdl2.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2cap, 'http://knowrob.org/kb/srdl2-cap.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2act, 'http://knowrob.org/kb/srdl2-action.owl#', [keep(true)]).

:- rdf_meta
        action_feasible_on_robot(r,r),
        action_feasible_on_robot(r,r,r),
        action_feasible_with_components(r,r,r,-),
        missing_for_action(r,r,r,r),
        missing_cap_for_action(r,r,r),
        required_cap_for_action(r,r),
        missing_comp_for_action(r,r,r),
        insufficient_comp_for_action(r,r,r,r),
        required_comp_for_action(r,r),
        unsatisfied_restr_for_action(r,r,r,r,-),
        unsatisfied_restr_for_required_comp(r,r,r,r,-),
        unsatisfied_restr_for_comp(r,r,r,r,-),
        restricted_act_on_robot(r,r,r),
        robot_tf_prefix(r,r),
        robot_part_tf_prefix(r,r),
        cap_available_on_robot(r,r),
        comp_type_available(r,r),
        comp_restricted_action(r,r,r),
        comp_installable_for_action(r,r,r,-),
        comp_installable_on_robot(r,r,r),
        comp_baselink_pose(r,-),
        comp_baselink_pose_at_time(r,-,+),
        srdl_inFieldOfView(r,-),
        srdl_inFieldOfView_at_time(r,-,+),
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

%% with_action_description(+ActionD, ?ActionI, +Robot, +Goal) is nondet.
%
% Ensures entity description is asserted and binds the name to
% Individual before goal is called.
% Also the performedBy relation will automatically be specified.
% Temporary assertions are retracted in a cleanup step.
%
% @param Description Entity description or individual
% @param Individual Entity individual
% Goal The goal with OWL entity asserted
%
with_action_description(ActionD, ActionI, Robot, Goal) :-
  with_owl_description(ActionD, ActionI, (
    once((
      rdf_has(ActionI, 'http://knowrob.org/kb/knowrob.owl#performedBy', _);
      rdf_assert(ActionI, 'http://knowrob.org/kb/knowrob.owl#performedBy', Robot)
    )),
    call(Goal)
  )).

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

action_feasible_on_robot(ActionC, ActionD, Robot) :-
  action_feasible_on_robot(ActionC, Robot),
  \+ unsatisfied_restr_for_action(ActionC, ActionD, Robot, _, _).


%% action_feasible_with_components(+ActionC, +ActionD, +Robot, -Components).
%
% True for feasible actions of type ActionC for which a
% Components list exists such that each element is one of the required components
% without any unsatisfied restrictions imposed on the action described by ActionD.
%
% @param ActionC   Action class to be checked
% @param ActionD   Action individual or description to be checked
% @param Robot   Robot instance to be checked
% @param Components   List of components feasible for the action
% 
action_feasible_with_components(ActionC, ActionD, Robot, Components) :-
  with_action_description(ActionD, ActionI, Robot, (
    action_feasible_on_robot(ActionC, ActionI, Robot),
    findall(Cs, (
      unsatisfied_restr_for_required_comp(ActionC, ActionI, Robot, _, RestrictedComponents),
      findall(Comp, member((Comp,[]), RestrictedComponents), Cs),
      \+ Cs=[]
    ), Components)
  )).
  


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
% Restricted actions

%% unsatisfied_restr_for_action(ActionC, ActionD, Robot, CompC, Unsatisfied).
%
% Checks if any action restriction imposed on robot or component level
% is unsatisfied.
%
% @param ActionC   Action class to be checked
% @param ActionD   Action description or instance to be checked
% @param Robot   Robot instance to be checked
% @param CompC   Required component class with unsatisfied restrictions
% @param Unsatisfied   Tuples of component individuals and unsatisfied restrictions
% 
unsatisfied_restr_for_action(ActionC, ActionD, Robot, CompC, Unsatisfied) :-
  with_action_description(ActionD, ActionI, Robot, (
    findall((CompC, Unsatisfied), (
        ((
          % check restrictions on robot level
          unsatisfied_restr_for_comp(ActionC, ActionI, Robot, Robot, Rs),
          CompC='http://knowrob.org/kb/knowrob.owl#Robot',
          Unsatisfied=[(Robot,Rs)]);
          % check restrictions on component level
          unsatisfied_restr_for_required_comp(ActionC, ActionI, Robot, CompC, Unsatisfied)
        ),
        not( member((_,[]), Unsatisfied) )
    ), Xs)
  )), !,
  member((CompC, Unsatisfied), Xs).


%% unsatisfied_restr_for_required_comp(ActionC, ActionD, Robot, CompC, RestrictedComponents).
%
% Checks if any action restriction imposed on component level
% is unsatisfied.
%
% @param ActionC   Action class to be checked
% @param ActionD   Action description or instance to be checked
% @param Robot   Robot instance to be checked
% @param CompC   Required component class with unsatisfied restrictions
% @param RestrictedComponents   Tuples of component and unsatisfied restrictions
% 
unsatisfied_restr_for_required_comp(ActionC, ActionD, Robot, CompC, RestrictedComponents) :-
  with_action_description(ActionD, ActionI, Robot,
    findall((CompC, RestrictedComponents), (
      required_comp_for_action(ActionC, CompC),
      findall((Comp,Restr), (
        rdf_reachable(Robot, 'http://knowrob.org/kb/srdl2-comp.owl#subComponent', Comp),
        once(owl_individual_of(Comp, CompC)),
        unsatisfied_restr_for_comp(ActionC, ActionI, Robot, Comp, Restr)
      ), RestrictedComponents)
    ), Xs)
  ), !,
  member((CompC, RestrictedComponents), Xs).


%% unsatisfied_restr_for_comp(ActionC, ActionD, Robot, Comp, Unsatisfied).
%
% Checks if any action restriction is unsatisfied.
%
% @param ActionC   Action class to be checked
% @param ActionD   Action description or instance to be checked
% @param Robot   Robot instance to be checked
% @param Comp   Component to be chacked
% @param Unsatisfied   List of unsatsfied action restrictions
% 
unsatisfied_restr_for_comp(ActionC, ActionD, Robot, Comp, Unsatisfied) :-
  with_action_description(ActionD, ActionI, Robot, (
    findall(R, (
      comp_restricted_action(Comp, ActionC, R),
      % owl_compute_individual_of supports computables in restricted actions!
      \+ owl_compute_individual_of(ActionI, R)
      %\+ owl_individual_of(ActionI, R)
    ), Unsatisfied)
  )).


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
  owl_class_properties(Action, srdl2cap:'dependsOnCapability', Cap).

required_cap_for_action(Action, Cap) :-
  owl_class_properties(Action, srdl2cap:'dependsOnCapability', Cp),
  owl_class_properties(Cp, srdl2cap:'dependsOnCapability', Cap).

required_cap_for_action(Action, Cap) :-
  plan_subevents_recursive(Action, SubAction),
  owl_class_properties(SubAction, srdl2cap:'dependsOnCapability', Cap).


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
    owl_class_properties(RobotClass, srdl2cap:'hasCapability', SubCap),

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

    forall( owl_class_properties(Cap, srdl2comp:'dependsOnComponent', CompT),
            comp_type_available(Robot, CompT) ),

    forall( owl_class_properties(Cap, srdl2cap:'dependsOnCapability', SubCap),
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


%% insufficient_comp_for_action(+ActionC, +ActionD, +Robot, ?Comp) is nondet.
%
% Insufficient components are required, but not available on the robot
% or available but not satisfying some restrictions imposed on the action
% (e.g., weak arms can't lift heavy objects).
%
% @param ActionC   Action class to be checked
% @param ActionD   Action individual or description to be checked
% @param Robot   Robot instance to be checked
% @param Comp    Component required to perform the action
%

insufficient_comp_for_action(ActionC, _, Robot, Comp) :-
  missing_comp_for_action(ActionC, Robot, Comp).

insufficient_comp_for_action(ActionC, ActionD, Robot, Comp) :-
  with_action_description(ActionD, ActionI, Robot,
    findall(Comp, (
      unsatisfied_restr_for_required_comp(ActionC, ActionI, Robot, Comp, Unsatisfied),
      \+ member((_,[]), Unsatisfied)
    ), Cs)
  ),
  member(Comp, Cs).


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
  owl_class_properties(Action, srdl2comp:'dependsOnComponent', Comp).

required_comp_for_action(Action, Comp) :-
  plan_subevents_recursive(Action, SubAction),
  owl_class_properties(SubAction, srdl2comp:'dependsOnComponent', Comp).

% components indirectly required by required capabilities
required_comp_for_action(Action, Comp) :-
  required_cap_for_action(Action, Cap),
  owl_class_properties(Cap, srdl2comp:'dependsOnComponent', Comp).


%% comp_restricted_action(+Comp, ?ActionC, ?RestrictedC) is nondet.
%
% Yields restricted action classes of components that are subclass
% of the action class.
%
% @param Comp     Component required to perform the action
% @param ActionC   Action class to be checked
% @param RestrictedC   Restricted action class possible with component
%
comp_restricted_action(Comp, ActionC, RestrictedC) :-
  findall(R, (
    owl_individual_of(Comp, CompC),
    owl_class_properties(CompC, 'http://knowrob.org/kb/knowrob.owl#actionable', R),
    owl_subclass_of(R, ActionC)
  ), Rs),
  list_to_set(Rs, RestrictedSet),
  member(RestrictedC, RestrictedSet).


%% comp_type_available(+Super, +SubT) is nondet.
%
% Check if Super has a sub-component Sub of type SubT
% (i.e. if a component of that type is available as part of Super)
%
% @param Super  Upper component
% @param SubT   Type of a Component that is part of the Super component
%
comp_type_available(Super, SubT) :-
  % TODO(daniel): would like to make subComponent transitive here, use rdf_reachable?
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


%% comp_installable_for_action(+ActionC, +ActionD, +Robot, -Components).
%
% For each insufficient component for the action, obtain a set of
% sufficient component individuals that could be installed on the robot.
%
% @param ActionC   Action class to be checked
% @param ActionD   Action individual or description to be checked
% @param Robot   Robot instance to be checked
% @param Components    Sequence of installation candidates for each insufficient component
%
comp_installable_for_action(ActionC, ActionD, Robot, Components) :-
  with_action_description(ActionD, ActionI, Robot, (
    findall((CompC,Xs), (
      insufficient_comp_for_action(ActionC, ActionI, Robot, CompC),
      % TODO(daniel): also check that component is sufficient for the action description!
      findall(X, comp_installable_on_robot(X, CompC, Robot), Xs)
    ), Components)
  )).


%% comp_installable_on_robot(+Comp, +CompC, +Robot).
%
% True for components Comp of type CompC that can be installed on the robot.
%
% @param CompC   Component class to be checked
% @param Comp   Component individual to be checked
% @param Robot   Robot instance to be checked
%
comp_installable_on_robot(Comp, CompC, Robot) :-
  once(owl_individual_of(Comp, CompC)),
  ActionD = [an, action,
      [type, srdl2act:installing_hardware_component],
      [object_acted_on, [an, object, [name, Comp] ]]
  ],
  action_feasible_on_robot(srdl2act:'InstallingHardwareComponent', ActionD, Robot).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Pose of semantic components

%% comp_baselink_pose(+Obj,-Pose) is nondet.
%% comp_baselink_pose(+Obj,-Pose,+Interval) is nondet.
%
% Computes the pose of a component composition as the 
% pose of one of its base links.
%
comp_baselink_pose(Obj, Pose) :-
  current_time(Instant),
  comp_baselink_pose_at_time(Obj, Pose, [Instant,Instant]).

comp_baselink_pose(Obj, Pose, Interval) :-
  nonvar(Obj),
  rdf_has(Obj, srdl2comp:baseLinkOfComposition, BaseLink),
  holds(BaseLink, knowrob:pose, Pose, Interval), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Higher-level reasoning methods
%

%% srdl_inFieldOfView(+Agent, ?Object) is nondet.
%% srdl_inFieldOfView(+Agent, ?Object, +Instant) is nondet.
%
% Check if Obj is visible by Agent at time Instant by reading the camera
% properties from the robot's SRDL description and computing whether the
% object center is inside the view frustrum.
%
% @param Agent      Some agent with a srdl:Camera component
% @param Obj        Instance of an object in the scene
% @param Instant    The time instant
% 
srdl_inFieldOfView(Agent, Object) :-
  current_time(Instant),
  srdl_inFieldOfView_at_time(Agent, Object, Instant).

srdl_inFieldOfView(Agent, Object, [Instant,Instant]) :-
  srdl_inFieldOfView_at_time(Agent, Object, Instant).

srdl_inFieldOfView_at_time(Agent, Object, Instant) :-
  nonvar(Agent),
  map_frame_name(MapFrame),
  % read camera properties
  agent_camera(Agent, Camera),
  camera_image_size(Camera, [ImgX,ImgY]),
  camera_hfov(Camera, HFOV),
  VFOV is ImgY / ImgX * HFOV,
  % find object pose in camera frame
  rdf_has(Camera, knowrob:frameName, CameraFrame),
  object_pose_at_time(Camera, Instant, [MapFrame, _, CamPos_world, CamRot_world]),
  rdf_has(Object, knowrob:frameName, ObjectFrame),
  object_pose_at_time(Object, Instant, [MapFrame, _, ObjPos_world, ObjRot_world]),
  transform_between(
      [x,ObjectFrame, ObjPos_world, ObjRot_world],
      [x,CameraFrame, CamPos_world, CamRot_world],
      [CameraFrame, ObjectFrame, [ObjX,ObjY,ObjZ], _]),
  % make the visibility test
  BearingX is atan2(ObjY, ObjX),
  BearingY is atan2(ObjZ, ObjX),
  abs(BearingX) < HFOV/2,
  abs(BearingY) < VFOV/2.

agent_camera(Agent, Camera) :-
  sub_component(Agent, Camera),
  owl_individual_of(Camera, srdl2comp:'Camera'), !.

camera_image_size(Camera, [ImgX,ImgY,ImgZ]) :-
  owl_has(Camera, srdl2comp:imageSizeX, literal(type(_, ImgXa))), atom_number(ImgXa, ImgX),
  owl_has(Camera, srdl2comp:imageSizeY, literal(type(_, ImgYa))), atom_number(ImgYa, ImgY),
  owl_has(Camera, srdl2comp:imageSizeZ, literal(type(_, ImgZa))), atom_number(ImgZa, ImgZ), !.
camera_hfov(Camera, HFOV) :-
  owl_has(Camera, srdl2comp:hfov, literal(type(_, HFOVa))), atom_number(HFOVa, HFOV), !.
    
