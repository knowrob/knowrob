
:- module(knowrob_action_execution,
    [
      action_registry/2,
      plan_execution_goal/4,
      execute_plan/4,
      action_filler_binding/2
    ]).
/** <module> The execution of actions.

*action_registry* is a multifile predicate, different action
execution methods need to define a clause of it.
Fillers of task roles and parameters are assigned as
participants of the action.
Also, a status quality is asserted for each action, and its
quale is updated during the course of its execution.
After action execution, the final status is added as participant
of the action and classified by the *Status* parameter of
the task (if any).

@author Daniel Beßler
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('list_util'), [ lazy_findall/3 ]).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_model')).

:- rdf_meta action_registry(r,?),
            action_bindings(r,t),
            plan_execution_goal(r,r,r,?),
            action_filler_binding(t,t),
            execute_plan(r,t,+,+),
            plan_execution_create(r,r,r,t,-,-).

%% action_registry(ActionConcept, Goal)
%
% A multifile predicate that defines Prolog goals
% that would when called perform some action.
%
:- multifile action_registry/2.

%% task_is_executable(Plan,Tsk,ActType,ActGoal) is semidet.
%
plan_execution_goal(Plan,Tsk,ActType,ActGoal) :-
  %plan_defines_task(Plan,Tsk),
  property_range(Plan,
    [dul:definesTask,dul:isExecutedIn],
    ActType),
  action_registry(ActType,ActGoal),
  rdf_has(Plan,ease:isPlanFor,Tsk).

%% execute_plan(Plan,InputDict,OutputDicts,PlanExecution)
%
execute_plan(Plan,InputDict,OutputDicts,PlanExecution) :-
  plan_execution_goal(Plan,Tsk,ActType,ActGoal),
  !, % FIXME <-- this kills choicepoints for different ways to execute the task.
     %           this case should be handled more careful. e.g. can we just ignore
     %           it, or do we need to react?
  action_bindings(Plan,BindingDict),
  plan_execution_create(ActType,Tsk,Plan,InputDict,Act,PlanExecution),
  %%%%%%%%%
  %%%%% ...
  %%%%%%%%%
  setup_call_cleanup(
    %% setup
    event_set_begin_time(Act),
    %% call
    lazy_findall(OutputDict,
      execute_plan_(Act,Tsk,ActGoal,PlanExecution,
                    InputDict,BindingDict,OutputDict),
      OutputDicts),
    %% cleanup
    event_set_end_time(Act)
  ).

execute_plan_(Act,Tsk,Goal,PlanExecution,InputDict,BindingDict,OutputDict) :-
  action_set_active(Act),
  catch(
    %% call
    ( call(Goal,PlanExecution,BindingDict,InputDict,OutputPairs) *->
      action_set_succeeded(Act) ;
      action_set_failed(Act)
    ),
    %% except
    action_failure(Diagnosis),
    %% do
    ( action_set_failed(Act),
      execution_add_diagnosis(PlanExecution,Diagnosis)
    )
  ),
  %%
  ( var(OutputPairs) -> OutputPairs=[] ; true ),
  dict_pairs(OutputDict,_,OutputPairs),
  %%
  assign_status_parameter(Act,Tsk,OutputDict).

plan_execution_create(ActType,Tsk,Plan,InputDict,Act,PlanExecution) :-
  action_create(ActType,Act,belief_state),
  action_set_task(Act,Tsk),
  situation_create(dul:'PlanExecution',PlanExecution,belief_state),
  situation_add(PlanExecution,Act),
  situation_add_satisfies(PlanExecution,Plan),
  %%
  forall(
    ( kb_triple(Plan,dul:describes,X) ; get_dict(_Key,InputDict,X) ),
    action_add_filler(Act,X)
  ),!.

assign_status_parameter(Act,Tsk,OutputDict) :-
  action_status(Act,Status),
  %%
  (( kb_triple(Tsk, dul:defines, Param),
     kb_type_of(Param, ease:'Status') ) ->
     b_set_dict(Param,OutputDict,Status) ; true ).

execution_add_diagnosis(Execution,Diagnosis) :-
  kb_type_of(Diagnosis,dul:'Diagnosis'),!,
  situation_add_sattifies(Execution,Diagnosis).

execution_add_diagnosis(Execution,DiagnosisConcept) :-
  kb_type_of(DiagnosisConcept,owl:'Class'),!,
  kb_create(DiagnosisConcept,Diagnosis),
  execution_add_diagnosis(Execution,Diagnosis).


%% action_filler_binding(Filler:InputDict,Entity:BindingDict)
%
% Find region or participant of action that is classified
% with the same concept as some other entity.
%
action_filler_binding(Filler:InputDict,Entity:BindingDict) :-
  get_dict(Concept,InputDict,Filler),
  get_dict(Concept,BindingDict,Entity).

action_bindings(ExecutionPlan,BindingDict) :-
  findall(R-X, (
    rdf_has(ExecutionPlan, ease_wf:hasBinding, B),
    rdf_has(B, ease_wf:hasBindingFiller, X),
    rdf_has(B, ease_wf:hasBindingRole, R)),
    Pairs),
  dict_pairs(BindingDict,_,Pairs).
