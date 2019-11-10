
:- module(knowrob_action_execution,
    [
      action_registry/2,
      execute_plan/4,
      action_filler_binding/2
    ]).
/** <module> The execution of plans.

@author Daniel Beßler
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl')).
:- use_module(library('list_util'), [ lazy_findall/3 ]).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/action_model')).

:- rdf_meta action_registry(r,?),
            action_filler_binding(t,t),
            execute_plan(r,t,+,+),
            plan_execution_create_(r,r,r,t,-,-).

%% action_registry(?ActionConcept, ?Goal) is semidet.
%
% A multifile predicate that defines Prolog goals
% that would when called perform some action.
%
% @param ActionConcept A sub-class of dul:'Action'
% @param Goal A Prolog goal
%
:- multifile action_registry/2.

%% execute_plan(+Plan,+InputDict,-OutputDicts,-PlanExecution) is semidet.
%
% Executes a plan with given initial grounding for roles and
% parameters (InputDict).
% Grounding inferred from the execution context are added
% to the lazy list OutputDicts.
% Finally, an individual of dul:'Situation' is created describing
% the execution.
%
% @param Plan An individual of type dul:'Plan'
% @param InputDict Initial grounding of roles and parameters
% @param OutputDicts Inferred grounding of roles and parameters
% @param PlanExecution An individual of type dul:'PlanExecution'
%
execute_plan(Plan,InputDict,OutputDicts,PlanExecution) :-
  plan_execution_goal_(Plan,Tsk,ActType,ActGoal),
  !, % FIXME <-- this kills choicepoints for different ways to execute the task.
     %           this case should be handled more careful. e.g. can we just ignore
     %           it, or do we need to react?
  action_bindings_(Plan,BindingDict),
  plan_execution_create_(ActType,Tsk,Plan,InputDict,Act,PlanExecution),
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
      execution_add_diagnosis_(PlanExecution,Diagnosis)
    )
  ),
  %%
  ( var(OutputPairs) -> OutputPairs=[] ; true ),
  dict_pairs(OutputDict,_,OutputPairs),
  %%
  assign_status_parameter_(Act,Tsk,OutputDict).

plan_execution_goal_(Plan,Tsk,ActType,ActGoal) :-
  property_range(Plan,
    [dul:definesTask,dul:isExecutedIn],
    ActType),
  action_registry(ActType,ActGoal),
  rdf_has(Plan,ease:isPlanFor,Tsk).

action_bindings_(ExecutionPlan,BindingDict) :-
  findall(R-X, (
    rdf_has(ExecutionPlan, ease_wf:hasBinding, B),
    rdf_has(B, ease_wf:hasBindingFiller, X),
    rdf_has(B, ease_wf:hasBindingRole, R)),
    Pairs),
  dict_pairs(BindingDict,_,Pairs).


%% action_filler_binding(?Filler,?Entity) is semidet.
%
% Used to resolve a binding constraint.
%
action_filler_binding(Filler:InputDict,Entity:BindingDict) :-
  get_dict(Concept,InputDict,Filler),
  get_dict(Concept,BindingDict,Entity).

		 /*******************************
		 *	'PlanExecution'		*
		 *******************************/

plan_execution_create_(ActType,Tsk,Plan,InputDict,Act,PlanExecution) :-
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

assign_status_parameter_(Act,Tsk,OutputDict) :-
  action_status(Act,Status),
  %%
  (( kb_triple(Tsk, dul:defines, Param),
     kb_type_of(Param, ease:'Status') ) ->
     b_set_dict(Param,OutputDict,Status) ; true ).

execution_add_diagnosis_(Execution,Diagnosis) :-
  kb_type_of(Diagnosis,dul:'Diagnosis'),!,
  situation_add_sattifies(Execution,Diagnosis).

execution_add_diagnosis_(Execution,DiagnosisConcept) :-
  kb_type_of(DiagnosisConcept,owl:'Class'),!,
  kb_create(DiagnosisConcept,Diagnosis),
  execution_add_diagnosis_(Execution,Diagnosis).
