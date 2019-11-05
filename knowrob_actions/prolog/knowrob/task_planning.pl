
:- module(knowrob_task_planning,
    [
      workflow_sequence/2,
      workflow_constituents/3
    ]).
/** <module> Reasoning about plans.

@author Daniel Beßler
*/
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/action_model'), [ workflow_step/2 ]).
:- use_module(library('knowrob/event_graph'),  [ esg_truncated/4,
                                                 esg_events/2
                                               ]).

:-  rdf_meta
      workflow_sequence(r,-).

%% workflow_sequence(+WF, ?StepSequence) is semidet.
%
% StepSequence is an ordered sequence of steps that are steps
% of the workflow WF.
% WF may also be a list of individual tasks with temporal relations
% between them.
%
% @param WF A workflow or a list of steps.
% @param SubTasks An ordered list of steps.
%
workflow_sequence(WF, StepSequence) :-
  \+ is_list(WF),!,
  findall(S, workflow_step(WF,S), Steps),
  workflow_sequence(Steps, StepSequence).
  
workflow_sequence(Steps, StepSequence) :-
  findall(Constraint, (
    member(X,Steps),
    allen_constraint(X,Constraint),
    Constraint =.. [_,_,Other],
    member(Other,Steps)
  ), Constraints),
  %
  esg_truncated(tsk, Steps, Constraints, [Sequence,_,_]),
  esg_events(Sequence, [_|StepSequence]).

%%
workflow_constituents(WF, Constituents, Constraints) :-
  findall(X0, (
    rdf_has(WF,ease:isPlanFor,X0);
    rdf_has(WF,dul:definesTask,X0);
    rdf_has(WF,ease_proc:definesProcess,X0)
  ), Constituents),
  findall(Constraint, (
    member(X1,Constituents),
    allen_constraint(X1,Constraint),
    Constraint =.. [_,_,Other],
    member(Other,Constituents)
  ), Constraints).
  
