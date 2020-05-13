:- module(model_DUL_Event,
    [ is_event(r),
      is_event_type(r),
      is_action(r),
      is_task(r),
      % FIXME: pl module process defines this already.
      %        - probably better assert all into another module then user,
      %          then prefer this over user
      %is_process(r),
      has_time_interval(r,r),
      has_participant(r,r),
      has_participant(r,r,r),
      executes_task(r,r),
      task_role(r,r),
      task_role_type(r,r,r),
      task_role_range(r,r,r)
    ]).
/** <module> DUL notion of Event.

In DUL, Object is defined as:
  "Any physical, social, or mental process, event, or state."

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2
    ]).
:- use_module('./Object.pl',
    [ has_object_type/2
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2
    ]).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_event(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Event'.
%
% @param Entity An entity IRI.
%
is_event(Entity) ?+>
  has_type(Entity, dul:'Event').

%% is_event_type(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'EventType'.
%
% @param Entity An entity IRI.
%
is_event_type(Entity) ?+>
  has_type(Entity, dul:'EventType').

%% is_action(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Action'.
%
% @param Entity An entity IRI.
%
is_action(Entity) ?+>
  has_type(Entity, dul:'Action').

%% is_task(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Task'.
%
% @param Entity An entity IRI.
%
is_task(Entity) ?+>
  has_type(Entity, dul:'Task').

%% is_process(+Entity) is semidet.
%
% True iff Entity is an instance of dul:'Process'.
%
% @param Entity An entity IRI.
%
is_process(Entity) ?+>
  has_type(Entity, dul:'Process').

%%
%
has_time_interval(Evt,TimeInterval) ?>
  holds(Evt,dul:hasTimeInterval,TimeInterval).

%has_time_interval(Evt,TimeInterval) +>
  %holds(Evt,dul:hasTimeInterval,TimeInterval).
  

%% has_participant(+Evt,?Participant,?Class) is semidet.
%
% Relates an event to a tuple of an object partipating in
% the event and its type.
%
% @param Evt An individual of type dul:'Event'.
% @param Participant An individual of type dul:'Object'.
% @param Class The most specific type of Participant.
%
has_participant(Evt,Participant) ?+>
  holds(Evt,dul:hasParticipant,Participant).

has_participant(Evt,Participant,Class) ?>
  holds(Evt,dul:hasParticipant,Participant),
  has_object_type(Participant,Class).

%% executes_task(?Act,?Tsk) is semidet.
%
% Relates an action to the task that it executes.
% Note that this relations may not be known, e.g. in case
% of observing another agent performing an action.
% In such a case this predicate fails.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
executes_task(Act,Tsk) ?+>
  holds(Act,dul:executesTask,Tsk).

%% task_role(?Tsk,?Role,?RoleType) is semidet.
%
% Relates a task to roles of objects related to the tasks.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param RoleType A sub-class of dul:'Role'.
%
task_role(Tsk,Role) ?+>
  holds(Tsk, dul:isTaskOf ,Role).

%%
%
task_role_type(Tsk,Role,RoleType) ?>
  holds(Tsk, dul:isTaskOf, Role),
  has_object_type(Role,RoleType).

%%
%
task_role_range(Tsk,Role,Range) ?>
  holds(Tsk,dul:isTaskOf,Role),
  holds(Role,dul:classifies,only(Range)).
