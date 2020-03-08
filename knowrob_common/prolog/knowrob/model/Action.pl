
:- module('knowrob/model/Action',
    [
      action_create/3,
      action_status/2,
      action_set_succeeded/1,
      action_set_failed/1,
      action_set_active/1,
      action_set_paused/1,
      action_set_pending/1,
      action_set_cancelled/1,
      action_has_task/2,
      action_set_task/2,
      action_add_filler/2,
      action_performed_by/2,
      action_set_performed_by/2
    ]).
/** <module> Interface to RDF model of actions.

*Action* is defined as an *Event* with at least one Agent that *isParticipantIn* it, and that *executes* a *Task* that typically *isDefinedIn* a *Plan*, *Workflow*, *Project*, etc.

@author Daniel Beßler
@license BSD
*/
%% TODO: it would be great if we could auto-generate the model interface code,
%%         i.e. all the code under 'knowrob/model'

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- use_module(library('knowrob/lang/ask'),  [ kb_triple/3 ]).
:- use_module(library('knowrob/lang/tell'), [ kb_assert/3, kb_retract/3 ]).
:- use_module(library('knowrob/model/Event'), [ event_create/3 ]).

:- rdf_meta
      action_create(r,r,+),
      action_set_succeeded(r),
      action_set_cancelled(r),
      action_set_pending(r),
      action_set_paused(r),
      action_set_active(r),
      action_set_failed(r),
      action_status(r,r),
      action_has_task(r,r),
      action_set_task(r,r),
      action_performed_by(r,r),
      action_set_performed_by(r,r),
      action_set_status_(r,r).

%% action_create(+ActType,-Act,+Graph) is semidet.
%
% Creates an action individual.
%
% @param ActType A sub-class of dul:'Action'.
% @param Act An individual of type ActType.
% @param Graph Name of the RDF graph where facts shall be asserted.
%
action_create(ActType,Act,Graph) :-
  event_create(ActType,Act,Graph).

%% action_status(?Act,?Status) is semidet.
%
% Relates an action to its execution status.
%
% @param Act An individual of type dul:'Action'.
% @param Status The execution status of Act.
%
action_status(Act,Status) :-
  kb_triple(Act, ease_act:hasExecutionState, Status).

%% action_set_succeeded(?Act) is det.
%
% Set the execution status of an action to 'succeeded'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_succeeded(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Succeeded').

%% action_set_failed(?Act) is det.
%
% Set the execution status of an action to 'failed'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_failed(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Failed').

%% action_set_active(?Act) is det.
%
% Set the execution status of an action to 'active'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_active(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Active').

%% action_set_paused(?Act) is det.
%
% Set the execution status of an action to 'paused'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_paused(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Paused').

%% action_set_pending(?Act) is det.
%
% Set the execution status of an action to 'planning'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_pending(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Pending').

%% action_set_cancelled(?Act) is det.
%
% Set the execution status of an action to 'cancelled'.
%
% @param Act An individual of type dul:'Action'.
%
action_set_cancelled(Act) :-
  action_set_status_(Act, ease_act:'ExecutionState_Cancelled').

%%
action_set_status_(Act,Status) :-
  kb_retract(Act, ease_act:hasExecutionState, _),
  kb_assert(Act, ease_act:hasExecutionState, Status).

%% action_has_task(?Act,?Tsk) is semidet.
%
% Relates an action to the task that it executes.
% Note that this relations may not be known, e.g. in case
% of observing another agent performing an action.
% In such a case this predicate fails.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_has_task(Act,Tsk) :-
  kb_triple(Act,dul:executesTask,Tsk).

%% action_set_task(+Act,+Tsk) is semidet.
%
% Asserts the task that is executed by an action.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
action_set_task(Act,Tsk) :-
  kb_assert(Act,dul:executesTask,Tsk).

%% action_add_filler(+Act,+Filler) is semidet.
%
% Asserts that some object or region was involved
% in an action.
% This does not assert what role the object played,
% or what the parameter associated to this region is.
%
% @param Act An individual of type dul:'Action'.
% @param Filler An individual of type dul:'Object' or dul:'Region'.
%
action_add_filler(Act,X) :-
  kb_type_of(X,dul:'Region'),!, (
    kb_triple(Act,dul:hasRegion,X);
    kb_assert(Act,dul:hasRegion,X)
  ), !.

action_add_filler(Act,X) :-
  kb_type_of(X,dul:'Object'),!, (
    kb_triple(Act,dul:hasParticipant,X);
    kb_assert(Act,dul:hasParticipant,X)
  ), !.

action_add_filler(_Action,X) :-
  print_message(warning,
    model_error(not_a_region_or_object(X))),
  fail.

%% action_performed_by(?Act,?Agent) is semidet.
%
% Relates an action to the agent that performs it.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_performed_by(Act,Agent) :-
  kb_triple(Act,ease_act:isPerformedBy,Agent).

%% action_set_performed_by(+Act,+Agent) is semidet.
%
% Asserts the agent that performs an action.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
action_set_performed_by(Act,Agent) :-
  kb_assert(Act,ease_act:isPerformedBy,Agent).
