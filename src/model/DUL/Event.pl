:- module(model_DUL_Event,
    [ is_event(r),
      is_event_type(r),
      is_action(r),
      is_process(r),
      is_task(r),
      % FIXME: pl module process defines this already.
      %        - probably better assert all into another module then user,
      %          then prefer this over user
      %is_process(r),
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
    [ has_type/2 ]).
:- use_module('./Object.pl',
    [ has_object_type/2 ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).

% setup tabled ask calls (the "g_" is prepended in expand_term)
:- table(g_is_event/1).
:- table(g_is_event_type/1).
:- table(g_is_action/1).
:- table(g_is_process/1).
:- table(g_is_task/1).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ namespace(dul)
    ]).

%% is_event(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Event'.
%
% @param Entity An entity IRI.
%
is_event(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Event').

%% is_event_type(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'EventType'.
%
% @param Entity An entity IRI.
%
is_event_type(Entity), [table(?)] ?+>
  has_type(Entity, dul:'EventType').

%% is_action(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Action'.
%
% @param Entity An entity IRI.
%
is_action(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Action').

%% is_task(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Task'.
%
% @param Entity An entity IRI.
%
is_task(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Task').

%% is_process(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Process'.
%
% @param Entity An entity IRI.
%
is_process(Entity), [table(?)] ?+>
  has_type(Entity, dul:'Process').

%% has_participant(+Evt,?Participant,?Class) is nondet.
%
% A relation between an object and a process, e.g.
% - 'John took part in the discussion',
% - 'a large mass of snow fell during the avalanche', or
% - 'a cook, some sugar, flour, etc. are all present in the cooking of a cake'.
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

%% executes_task(?Act,?Tsk) is nondet.
%
% A relation between an action and a task, e.g. 'putting some water in a pot
% and putting the pot on a fire until the water starts bubbling'
% executes the task 'boiling'.
%
% @param Act An individual of type dul:'Action'.
% @param Tsk An individual of type dul:'Task'.
%
executes_task(Act,Tsk) ?+>
  holds(Act,dul:executesTask,Tsk).

%% task_role(?Tsk,?Role) is nondet.
%
% A relation between roles and tasks, e.g.
% 'students have the duty of giving exams'
% (i.e. the Role 'student' hasTask the Task 'giving exams').
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
%
task_role(Tsk,Role) ?+>
  holds(Tsk, dul:isTaskOf ,Role).

%% task_role_type(?Tsk,?Role,?RoleType) is nondet.
%
% Same as task_role/2 but in addition unifies the
% type of the parameter with the third argument.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param RoleType A sub-class of dul:'Role'.
%
task_role_type(Tsk,Role,RoleType) ?>
  holds(Tsk, dul:isTaskOf, Role),
  has_object_type(Role,RoleType).

%% task_role_range(?Tsk,?Role,?Range) is nondet.
%
% Same as task_role/2 but in addition unifies the
% range of the parameter with the third argument.
%
% @param Tsk An individual of type dul:'Task'.
% @param Role An individual of type dul:'Role'.
% @param Range A sub-class of dul:'Object'.
%
task_role_range(Tsk,Role,Range) ?>
  holds(Tsk,dul:isTaskOf,Role),
  holds(Role,dul:classifies,only(Range)).
