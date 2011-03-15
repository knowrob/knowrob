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
        missing_cap_for_action/3,
        missing_cap_for_action/3,
        missing_comp_for_action/3,
        required_comp_for_action/2
  ]).



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% Library imports

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('comp_ehow')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).


:- owl_parser:owl_parse('../owl/srdl2.owl', false, false, true).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2, 'http://ias.cs.tum.edu/kb/srdl2.owl#', [keep(true)]).


:- rdf_meta
        action_feasible_on_robot(r,r),
        missing_cap_for_action(r,r,r),
        required_cap_for_action(r,r),
        missing_comp_for_action(r,r,r),
        required_comp_for_action(r,r),
        class_properties(r,r,r).
 



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Actions


action_feasible_on_robot(Action, Robot) :-
  missing_cap_for_action(Action, Robot, []),
  missing_comp_for_action(Action, Robot, []).

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

  plan_subevents_recursive(Action, SubActions),
  member(SubAction, SubActions),
  class_properties(SubAction, srdl2:'dependsOnCapability', Cap).




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

  plan_subevents_recursive(Action, SubActions),
  member(SubAction, SubActions),
  class_properties(SubAction, srdl2:'dependsOnComponent', Comp).

% components indirectly required by required capabilities
required_comp_for_action(Action, Comp) :-

  required_cap_for_action(Action, Cap),
  class_properties(Cap, srdl2:'dependsOnComponent', Comp).











% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Capabilities

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
    class_properties(Robot, srdl2:'hasCapability', Cap).

% capability asserted for robot class
cap_available_on_robot(Cap, Robot) :-
    rdfs_individual_of(Robot, RobotClass),
    class_properties(RobotClass, srdl2:'hasCapability', Cap).

% capability depends only on available components or capabilities
cap_available_on_robot(Cap, Robot) :-

    rdfs_subclass_of(Cap, srdl2:'Capability'),

    forall( class_properties(Cap, srdl2:'dependsOnComponent', CompT),
            comp_type_available(Robot, CompT) ),

    forall( class_properties(Cap, srdl2:'dependsOnCapability', SubCap),
            cap_available_on_robot(SubCap, Robot) ).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Components


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
    rdf_has(Sub, rdf:type, SubT).


%% sub_component(?Super, ?Sub) is nondet.
%
% Recursively read all sub-components of a robot or a component
%
% @param Super  Upper component
% @param Sub    Component that is part of the Super component
%
sub_component(Super, Sub) :-
  owl_has(Super, srdl2:'subComponent', Sub).




 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% OWL/ DL Predicates



%% class_properties(?Class, ?Prop, ?Values) is nondet.
% 
% Collect all property values of someValuesFrom-restrictions of a class
% 
% @param Class   Class whose restrictions are being considered
% @param Prop    Property whose restrictions in Class are being considered
% @param Values  List of all classes that appear in a restriction of a superclass of Class along Property

% class_properties(Class, Prop, Values) :-
%   findall(Val, class_properties_1(Class, Prop, Val), Vals),
%   sort(Vals, Values).

class_properties(Class, Prop, Val) :-
  owl_direct_subclass_of(Class, Sup),
  owl_direct_subclass_of(Sup, Sup2),
  owl_restriction(Sup2,restriction(Prop, some_values_from(Val))).

class_properties(Class, Prop, Val) :-
  owl_direct_subclass_of(Class, Sup),
  owl_restriction(Sup,restriction(Prop, some_values_from(Val))).




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Tree search Predicates


%% tree_search_all(+List, +Connective, -Result) is nondet.
%
% Tree search that collects all nodes in the tree
% Expansion of first node via property values in restrictions of class descriptions
%
% @param   List        Search agenda
% @param   Connective  OWL property that spans the tree (via restrictions)
% @returns List of all nodes in the tree
% 
tree_search_all([], _, []).
tree_search_all([H|T], Connective, Result) :-
    class_properties(H, Connective, L1),
    append_unique(T, L1, SearchAgenda),
    tree_search_all(SearchAgenda, Connective, TmpResult),
    append_unique(TmpResult, [H], Result).


%% tree_search_all(+List, +Connective, -Result) is nondet.
%
% Tree search that collects only leaf nodes in the tree
% Expansion of first node via property values in restrictions of class descriptions
%
% @param   List        Search agenda
% @param   Connective  OWL property that spans the tree (via restrictions)
% @returns List of all leaf nodes in the tree
%
tree_search_leaves([], _, []).
tree_search_leaves([H|T], Connective, Result) :-
    class_properties(H, Connective, L1),
    append_unique(T, L1, SearchAgenda),
    tree_search_leaves(SearchAgenda, Connective, TmpResult),
    (
      L1 == []
   ->
      append_unique(TmpResult, [H], Result)
    ;
      Result = TmpResult
    ),
    !.  % commit to first choice





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Utility Predicates




% %% append_unique(+L1, +L2, -Result) is nondet.
% %
% % Append List L2 to L1 in a nonredundant way: check for each member m of L2
% % if it is already member of L1 and add only if not
% %
% % @param   L1  List 1
% % @param   L2  List 2 that is to be appended to list 1
% % @returns Concatenated list without duplicates
% %
% 
% append_unique(L, [], L).
% append_unique(L1, [H|T], Result) :-
%     (
%       %member(H, L1)
%       memberchk(H, L1)
%    ->
%       append_unique(L1, T, Result)
%     ;
%       ( append([H], L1, L2), append_unique(L2, T, Result) )
%     ),
%     !.  % Commit to first choice
% 
% 
% 
% 
% 
% 











% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % Learnability Inference
% 
% 
% % Check if capability Cap is potentially learnable (meaning it could be learned if training data are available)
% learnable_cap(Cap, RobotInst) :-
%     owl_subclass_of(Cap, srdl_cap:'Capability'),
%     % Cond 1: no missing hardware component
%     \+ ( class_properties(Cap, srdl:'needsComponent', ReqList),
%          member(HwComp, ReqList),
%          owl_subclass_of(HwComp, srdl_comp:'HardwareComponent'),
%          comp_type_available(HwComp, RobotInst)
%        ),
%     % Cond 2: learning component existent
%     class_properties(Cap, srdl:'hasLearningComponent', LearnCompList),
%     member(LearningComp, LearnCompList),
%     comp_type_available(LearningComp, RobotInst),
%     !. % Commit to first choice
% 
% 




% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Experience Inference Predicates



% % Compute successProbability of a cyc:PurposefulAction from its attempt and success numbers
% % attempt and success numbers can be directly asserted or computed from childs in action tree
% computeSuccessProbability(Action, SuccessProbability) :-
%     owl_subclass_of(Action, knowrob:'PurposefulAction'),
%     (
%         % jump to subclass of action that is also a subclass of srdl:'SrdlRobotAction'
%         owl_subclass_of(RobotAction, Action),
%         owl_subclass_of(RobotAction, srdl:'SrdlRobotAction'),
%         % Bind NumAttempts and NumSuccesses
%         intValueFromHasValueRestriction(RobotAction, srdl:'hasNumOfSuccesses', NumSuccesses),
%         intValueFromHasValueRestriction(RobotAction, srdl:'hasNumOfAttempts', NumAttempts)
%     ->
%         % Compute success probability
%         SuccessProbability is NumSuccesses / NumAttempts
%     ;
%         (
%             computeSuccessProbabilityFromDescendants(Action, TmpProb)
%         ->
%             SuccessProbability = TmpProb
%         ;
%             fail
%         )
%     ),
%     !. % Commit to first result

% % Compute success probability on the base of all descendants using min prob
% computeSuccessProbabilityFromDescendants(Action, ResultProb) :-
%     tree_search_all([Action], knowrob:'subAction', SubActionsTmp),
%     delete(SubActionsTmp, Action, SubActions),
%     (
%        SubActions == []
%     ->
%         % in case of a taks with no descendants with no attempt and success numbers: fail
%         fail
%     ;
%         % in case of a task with subtasks: enumerate all success probabilities and return min
%         enumerateSuccessProbabilities(SubActions, ProbList),
%         min_list(ProbList, MinProb),
%         ResultProb = MinProb
%     ),
%     !.  % Commit to first solution

% % Compute success probabilities for actions in ActionList
% % Just calls predicate enumerateSuccessProbabilitiesRec
% enumerateSuccessProbabilities(ActionList, Result) :-
%     enumerateSuccessProbabilitiesRec(ActionList, [], Result).
% 
% % Compute success probabilities for each action in action list and return all success probabilities in a list
% enumerateSuccessProbabilitiesRec([], Acc, Acc).
% enumerateSuccessProbabilitiesRec([H|T], Acc, Result) :-
%     enumerateSuccessProbabilitiesRec(T, Acc, TmpResult),
%     (
%         computeSuccessProbability(H, HeadSuccessProb)
%     ->
%         append_unique(TmpResult, [HeadSuccessProb], Result)
%     ;
%         Result = TmpResult
%     ).





% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Debug print predicates


% % Print components that are missing for action Action on robot RobotInst
% % This predicate only calls predicate printMissingComponents/3
% printMissingComponents(Action, RobotInst) :-
%     printMissingComponents(Action, RobotInst, 0),
%     !.
% 
% % Print components that are missing for action Action on robot RobotInst
% printMissingComponents(Action, RobotInst, Indentation) :-
%     (
%         missing_caps_for_action(Action, RobotInst, MissingCap),
%         MissingCap \== []
%     ->
%        printMissingComponentsForCapList(MissingCap, RobotInst, Indentation)
%     ;
%        fail
%     ),
%     !.  % Commit to first result
% 
% % Iterate over cap list and print missing components for every cap
% printMissingComponentsForCapList([], _, _).
% printMissingComponentsForCapList([H|T], RobotInst, Indentation) :-
%     printMissingComponentsForSingleCap(H, RobotInst, Indentation),
%     printMissingComponentsForCapList(T, RobotInst, Indentation).
% 
% % print missing components for a single cap
% printMissingComponentsForSingleCap(Cap, RobotInst, Indentation) :-
%     % owl_subclass_of(Cap, srdl_cap:'PrimitiveCapability'),
%     nl, printIndentation(Indentation), print('### CAPABILITY ### '), print(Cap), nl,
%     findall(X,
%               (
%                   owl_subclass_of(X, srdl:'CapabilityProvisionAlternative'),
%                   class_properties(X, srdl:'providesCapability', CapList),
%                   member(Cap, CapList)
%               ),
%              AltList),
%     printMissingComponentsForAlternativeList(AltList, RobotInst, Indentation).
% 
% % Iterate over alternatives list and print missing components for every alternative of a capability
% printMissingComponentsForAlternativeList([], _, _).
% printMissingComponentsForAlternativeList([H|T], RobotInst, Indentation) :-
%     printMissingComponentsForSingleAlternative(H, RobotInst, Indentation),
%     printMissingComponentsForAlternativeList(T, RobotInst, Indentation).
% 
% % print missing components for a single alternative
% printMissingComponentsForSingleAlternative(Alt, RobotInst, Indentation) :-
%     printIndentation(Indentation), print('# Provision alternative # '), print(Alt), nl,
%     (
%         owl_subclass_of(Alt, srdl:'PrimitiveCapabilityAlternative')
%     ->
%         % CASE 1: primitive cap
%         % collect all requirements
%         class_properties(Alt, srdl:'needsComponent', ReqList),
%         % calculate and print unmet requirements
%         comp_types_not_available(ReqList, RobotInst, UnmetReq),
%         printIndentation(Indentation), print('Unmet requirements: '), nl, printIndentation(Indentation), print(UnmetReq), nl,
%         % calculate and print  met requirements
%         findall(X, ( member(X, ReqList), \+ member(X, UnmetReq) ), MetReq),
%         printIndentation(Indentation), print('Met requirements: '), nl, printIndentation(Indentation), print(MetReq), nl
%     ;
%         % CASE 2: composite cap
%         printIndentation(Indentation), print('Capability provision alternative "'), print(Alt) , print('" is a composite capability provision alternative.'), nl,
% 
% 
%         % collect all requirements
%         class_properties(Alt, srdl:'needsComponent', ReqList),
%         % calculate and print unmet requirements
%         comp_types_not_available(ReqList, RobotInst, UnmetReq),
%         printIndentation(Indentation), print('Unmet requirements: '), nl, printIndentation(Indentation), print(UnmetReq), nl,
% 
% 
%         printIndentation(Indentation), print('In the following alternatives for subcapabilities are shown:'), nl, printIndentation(Indentation), print('==>'),
% 
%         class_properties(Alt, srdl_cap:'hasSubCapability', SubCapList),
%         NextIndentation is Indentation + 4,
%         printMissingComponentsForCapList(SubCapList, RobotInst, NextIndentation),
% 
%         printIndentation(Indentation), print('<==')
%     ).
% 
% 
% 
% % print all attributes of a component
% printAttributesOfComponent(Comp) :-
%     owl_individual_of(Comp, srdl_comp:'Component'),
%     print('Attributes of component '), print(Comp), nl, print('-----'), nl,
%     findall(X, owl_has(Comp, srdl_comp:'hasAttribute', X),AttList),
%     printAttributeList(AttList),
%     !.
% 
% % iterate over attribute list and print each attribute
% printAttributeList([]).
% printAttributeList([H|T]) :-
%     printSingleAttribute(H),
%     printAttributeList(T).
% 
% % print name, value and unit of measure of a single attribute
% printSingleAttribute(Attr) :-
%     owl_has(_, srdl_comp:'hasAttribute', Attr),
%     % print('Attribute '),
% 
%     % attribute name
%     (
%         owl_has(Attr, srdl_comp:'hasAttributeName', NameLiteral)
%     ->
%         NameLiteral = literal(Name),
%         print(Name)
%     ;
%         print('unspecified name')
%     ),
%     print(' = '),
% 
%     % attribute value
%     (
%         owl_has(Attr, srdl_comp:'hasAttributeValue', ValueLiteral)
%     ->
%         ValueLiteral = literal(Value),
%         print(Value)
%     ;
%         print('unspecified value')
%     ),
%     print(' '),
% 
%     % attribute unit of measure
%     (
%         owl_has(Attr, srdl_comp:'hasAttributeUnitOfMeasure', UomLiteral)
%     ->
%         UomLiteral = literal(Uom),
%         print(Uom)
%     ;
%         print('unspecified unit of measure')
%     ),
%     nl,
%     !.
% 
% % Print Indentation number of spaces recursively
% % This is used for formatting print output
% printIndentation(0).
% printIndentation(Indentation) :-
%     print(' '),
%     X is Indentation - 1,
%     printIndentation(X),
%     !.


