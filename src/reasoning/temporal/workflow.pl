:- module(temporal_workflow,
    [ workflow_sequence(r,t)
    ]).
/** <module> Reasoning about plans.

@author Daniel Beßler
*/

:- use_module(library('model/DUL/Workflow'),
        [ workflow_step/2 ]).

:- use_module('interval',
        [ interval_constraint/2 ]).
:- use_module('esg',
        [ esg_truncated/4, esg_event_sequence/2 ]).

%% workflow_sequence(+WF, ?StepSequence) is semidet.
%
% StepSequence is an ordered sequence of steps that are steps
% of the workflow WF.
% WF may also be a list of individual tasks with temporal relations
% between them.
%
% @param WF A workflow or a list of steps.
% @param StepSequence An ordered list of steps.
%
workflow_sequence(WF, StepSequence) :-
  \+ is_list(WF),!,
  findall(S, workflow_step(WF,S), Steps),
  workflow_sequence(Steps, StepSequence).
  
workflow_sequence(Steps, StepSequence) :-
  findall(Constraint, (
    member(X,Steps),
    interval_constraint(X,Constraint),
    Constraint =.. [_,_,Other],
    member(Other,Steps)
  ), Constraints),
  %
  esg_truncated(tsk, Steps, Constraints, [Sequence,_,_]),
  esg_event_sequence(Sequence, [_|StepSequence]).
