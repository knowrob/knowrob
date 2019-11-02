%%
%% Copyright (C) 2014 by Moritz Tenorth
%%
%% This file contains tests for the SRDL
%% tools in KnowRob.
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

:- begin_tests(srdl).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).

:- rdf_db:rdf_register_ns(pr2, 'http://knowrob.org/kb/PR2.owl#', [keep(true)]).

:- use_module(library('srdl')).

:- owl_parser:owl_parse('package://srdl/owl/PR2.owl').

:- dynamic testbot/1.

is_composition(Comp) :-
  rdf_has(Comp,srdlcomp:hasBaseLink,_),
  rdf_has(Comp,srdlcomp:hasEndLink,_),!.

test(robot_create) :-
  robot_create(knowrob:'PR2',PR2),
  asserta(testbot(PR2)),
  %% test created component symbols
  once((
    has_direct_component(PR2,_,pr2:'PR2Base'),
    has_direct_component(PR2,_,pr2:'PR2ArmL'),
    has_direct_component(PR2,_,pr2:'PR2ArmR'),
    has_direct_component(PR2,_,pr2:'PR2GripperL'),
    has_direct_component(PR2,_,pr2:'PR2GripperR')
  )).

test(robot_set_urdf) :-
  ros_package_path('urdfprolog', X),
  atom_concat(X, '/urdf/pr2_for_unit_tests.urdf', FileURL),
  %% load the URDF
  testbot(PR2),
  robot_set_urdf(PR2,FileURL),
  %% test link-component association
  forall(
    has_direct_component(PR2,BodyPart,_),
    ( is_composition(BodyPart) ; component_type(BodyPart,_) )
  ).

%test(comp_unknown_action) :-
  %\+ action_feasible_on_robot(knowrob:'leainhfoiezr8hfddef8ez3radsfij',  pr2:'PR2Robot1'),!.

%test(cap_available_on_robot) :-
  %cap_available_on_robot(srdl2cap:'GraspingCapability', pr2:'PR2Robot1'),!.


%test(required_comp_for_action) :-
  %required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmComponent'),
  %required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmMotionController'),!.


%test(missing_comp_for_action) :-
  %\+ missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  %% FIXME(daniel) below seems semantically wrong because the arm is _not_ a missing component.
  %missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmComponent'),
  %missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmMotionController'),!.


%test(required_cap_for_action) :-
  %required_cap_for_action(act:'MakingPancakes', srdl2cap:'PickingUpAnObjectCapability'),
  %required_cap_for_action(act:'MakingPancakes', srdl2cap:'ArmMotionCapability'),!.


%test(missing_cap_for_action) :-
  %\+ missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  %missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability'),
  %missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'ArmMotionCapability'),!.


%test(missing_for_action) :-
  %\+ missing_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _, _),
  %missing_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability', srdl2comp:'ArmMotionController'),!.

%test(action_feasible_on_robot) :-
  %\+ action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robt1'),
  %action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robot1'),!.

:- end_tests(srdl).

