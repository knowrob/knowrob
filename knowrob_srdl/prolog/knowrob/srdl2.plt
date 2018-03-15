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

:- begin_tests(srdl2).

:- use_module(library('semweb/owl')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('knowrob/srdl2')).

:- owl_parser:owl_parse('package://knowrob_srdl/owl/srdl2-action.owl').
:- owl_parser:owl_parse('package://knowrob_srdl/owl/PR2.owl').
:- owl_parse('package://knowrob_actions/owl/pancake-making.owl').

:- rdf_db:rdf_register_ns(rdf,  'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl,  'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd,  'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(act,  'http://knowrob.org/kb/pancake-making.owl#', [keep(true)]).


test(sub_component) :-
  sub_component(pr2:'PR2Robot1', pr2:pr2_base),!.


test(comp_type_available) :-
  comp_type_available(pr2:'PR2Robot1', srdl2comp:'Camera'),!.

test(comp_unknown_action) :-
  \+ action_feasible_on_robot(knowrob:'leainhfoiezr8hfddef8ez3radsfij',  pr2:'PR2Robot1'),!.

test(cap_available_on_robot) :-
  cap_available_on_robot(srdl2cap:'GraspingCapability', pr2:'PR2Robot1'),!.


test(required_comp_for_action) :-
  required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmComponent'),
  required_comp_for_action(act:'MakingPancakes', srdl2comp:'ArmMotionController'),!.


test(missing_comp_for_action) :-
  \+ missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  % FIXME(daniel) below seems semantically wrong because the arm is _not_ a missing component.
  missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmComponent'),
  missing_comp_for_action(act:'MakingPancakes', pr2:'PR2Robt1', srdl2comp:'ArmMotionController'),!.


test(required_cap_for_action) :-
  required_cap_for_action(act:'MakingPancakes', srdl2cap:'PickingUpAnObjectCapability'),
  required_cap_for_action(act:'MakingPancakes', srdl2cap:'ArmMotionCapability'),!.


test(missing_cap_for_action) :-
  \+ missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _),
  missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability'),
  missing_cap_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'ArmMotionCapability'),!.


test(missing_for_action) :-
  \+ missing_for_action(act:'MakingPancakes', pr2:'PR2Robot1', _, _),
  missing_for_action(act:'MakingPancakes', pr2:'PR2Robo1', srdl2cap:'PickingUpAnObjectCapability', srdl2comp:'ArmMotionController'),!.

test(action_feasible_on_robot) :-
  \+ action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robt1'),
  action_feasible_on_robot(act:'MakingPancakes', pr2:'PR2Robot1'),!.

:- end_tests(srdl2).

