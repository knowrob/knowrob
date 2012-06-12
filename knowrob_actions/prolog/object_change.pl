
:- module(object_change,
    [
      transformed_into/2,
      action_effects/2,
      transformed_into_transitive/2,
      comp_thermicallyConnectedTo/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('knowrob_owl')).

:- use_module(library('action_effects')).
:- use_module(library('process_effects')).


:- owl_parser:owl_parse('../owl/object-change.owl', false, false, true).
% :- owl_parser:owl_parse('../owl/pancake-making.owl', false, false, true).

:- rdf_db:rdf_register_ns(knowrob,      'http://ias.cs.tum.edu/kb/knowrob.owl#',      [keep(true)]).
:- rdf_db:rdf_register_ns(object_change, 'http://ias.cs.tum.edu/kb/object-change.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(pancake, 'http://ias.cs.tum.edu/kb/pancake-making.owl#', [keep(true)]).

:-  rdf_meta
        transformed_into(r, r),
        transformed_into_transitive(r, r),
        comp_thermicallyConnectedTo(r,r),
%         class_properties(r,r,r),
        project_and_debug(r,r,r),
        project_action_class(r,r,r),
        add_subactions_for_action(r, ?),
        action_inputs(r,r),
        action_missing_inputs(r,r),
        action_outputs(r,r),
        resource_provided_by_actionseq(r, ?),
        resource_available(r),
        plan_subevents(r,r),
        action_effects(r,r).



%% clean_projection_cache is det.
%
% remove all triples that have been asserted as part of the projection methods
%
clean_projection_cache :-
    rdf_retractall(_, _, _,knowrob_projection).




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% action effects
%


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
  project_action_effects(Action),

  % check for processes that got triggered
  (project_process_effects;true),

  rdf_has(Action, knowrob:postActors, PostActors).





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% object transformations
%


%% transformed_into(?From, ?To)
%
% Compute which objects have been transformed into which other ones
% by actions or processes. This predicate operates on the object
% modification graph created by the action projection rules
%
% @param From Input of some action
% @param To   Output created by this action
%
transformed_into(From, To) :-
  ( owl_has(Event, knowrob:thingIncorporated, From);
    owl_has(Event, knowrob:objectAddedTo, From);
    owl_has(Event, knowrob:inputsDestroyed, From);
    owl_has(Event, knowrob:inputsCommitted, From);
    owl_has(Event, knowrob:transformedObject, From);
    owl_has(Event, knowrob:objectRemoved, From);
    owl_has(Event, knowrob:objectOfStateChange, From);
    owl_has(Event, knowrob:outputsRemaining, From) ),

  ( owl_has(Event, knowrob:outputsRemaining, To);
    owl_has(Event, knowrob:outputsCreated, To)).


%% transformed_into_transitive(?From, ?To)
%
% Transitive version of the transformed_into predicate that tracks
% in- and outputs of actions over several steps and different
% properties
%
% @param From Input of some action
% @param To   Output created by this action
%
transformed_into_transitive(From, To) :-
  transformed_into(From, To).

transformed_into_transitive(From, To) :-
  transformed_into(From, Sth),
  From\=Sth,
  transformed_into_transitive(Sth, To).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% process-relevant object relations
%

%% comp_thermicallyConnectedTo(?Obj1, ?Obj2)
%
% Compute if a heat path exists between two objects. This is the case if
% they are either on top of each other or if one contains the other one
%
% @param Obj1 Object instance
% @param Obj2 Object instance
%
comp_thermicallyConnectedTo(Obj1, Obj2) :-
  rdf_triple(knowrob:'on-Physical', Obj1, Obj2);
  rdf_triple(knowrob:'on-Physical', Obj2, Obj1).


comp_thermicallyConnectedTo(Obj1, Obj2) :-
  rdf_triple(knowrob:'in-ContGeneric', Obj1, Obj2);
  rdf_triple(knowrob:'in-ContGeneric', Obj2, Obj1).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% planning/debugging
%


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
  object_change:plan_subevents(Plan, OrigActionSeq),

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
  class_properties(Action, knowrob:'postActors', Output).
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







%
% project_and_debug(Plan, ActionSeq, Result) :-
%
%   object_change:plan_subevents(Plan, SubEvents),
%
%   % project
%   findall(Inst, (member(Sub, SubEvents), object_change:project_action_class(Sub, Inst, Post)), Actions),
%
%   % check if result created
%   findall(P, class_properties(Plan, knowrob:'taskToAchieve', P), Ps), member(Post, Ps),
%   ( (owl_individual_of(Result, Post), member(A, Actions), owl_has(A, knowrob:postActors, Result)) ->
%       (AddActions=[]) ;
%       (additional_actions_req_for_output(Post, AddActions) )),
%
%   flatten([AddActions|Actions], ActionSeq).
%
%
%
% additional_actions_req_for_output(Post, Actions) :-
%   findall(A, action_outputs(A, Post), As), member(Action, As),
%   add_subactions_for_action(Action, SubActions),
%   append(SubActions, [Action], Actions),
%
%   findall(_, (member(Sub, Actions), object_change:project_action_class(Sub, _, _)), _).








% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
%   vv   COPIED FROM OTHER MODULES -- TO BE REMOVED   vv
%


%% plan_subevents(+Plan, ?SubEvents) is semidet.
%
% Read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents(Plan, SubEvents) :-
  findall(SubEvent, ( owl_has(Plan, rdfs:subClassOf, D),
                      owl_has(D, owl:intersectionOf, I),
                      rdfs_member(R, I),
                      rdf_has(R, owl:onProperty, knowrob:'subAction'),
                      rdf_has(R, owl:someValuesFrom, SubEvent)), Sub),
  predsort(compare_actions_partial_order, Sub, SubEvents).


%% plan_subevents_recursive(+Plan, ?SubEvents) is semidet.
%
% Recursively read all sub-event classes of the imported plan, i.e. single actions that need to be taken
%
% @param Plan Plan identifier
% @param SubEvents List of sub-events of the plan
%
plan_subevents_recursive(Plan, SubEvents) :-
  plan_subevents_recursive_1(Plan, Sub),
  flatten(Sub,SubEvents).

% simple case: no subevents, return action itself
plan_subevents_recursive_1(Plan, []) :-
  plan_subevents(Plan, []).

% if there are subevents, iterate
plan_subevents_recursive_1(Plan, [Plan|SubSubEvents]) :-
  plan_subevents(Plan, SubEvents),

  findall(SubSubEvent, (member(SubEvent, SubEvents),
                       plan_subevents_recursive_1(SubEvent, SubSubEvent)), SubSubEvents).


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




%% class_properties(?Class, ?Prop, ?Values) is nondet.
%
% Collect all property values of someValuesFrom-restrictions of a class
%
% @param Class   Class whose restrictions are being considered
% @param Prop    Property whose restrictions in Class are being considered
% @param Values  List of all classes that appear in a restriction of a superclass of Class along Property

% class_properties(Class, Prop, Val) :-         % read directly asserted properties
%   class_properties_1(Class, Prop, Val).
% class_properties(Class, Prop, Val) :-         % also consider properties of superclasses
%   nonvar(Class),
%   owl_subclass_of(Class, Super), Class\=Super,
%   class_properties_1(Super, Prop, Val).
%
% % read restrictions defined for Class for Prop or a sub-property of Prop
% class_properties_1(Class, Prop, Val) :-
%   nonvar(Class),
%   owl_direct_subclass_of(Class, Sup),
%   owl_direct_subclass_of(Sup, Sup2),
%   ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
%   owl_restriction(Sup2,restriction(SubProp, some_values_from(Val))).
%
% class_properties_1(Class, Prop, Val) :-
%   nonvar(Class),
%   owl_direct_subclass_of(Class, Sup),
%   ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
%   owl_restriction(Sup,restriction(SubProp, some_values_from(Val))).
%
% class_properties_1(Class, Prop, Val) :-
%   var(Class),
%   ( (nonvar(Prop)) -> (rdfs_subproperty_of(SubProp, Prop)) ; (SubProp = Prop)),
%   owl_restriction(Sup,restriction(SubProp, some_values_from(Val))),
%   owl_direct_subclass_of(Class, Sup).


