:- module(model_DUL_Situation,
    [ is_description(r),
      is_situation(r),
      is_goal(r),
      is_design(r),
      is_diagnosis(r),
      is_plan(r),
      is_plan_execution(r),
      is_norm(r),
      is_transition(r),
      plan_has_goal(r,r),
      satisfies(r,r),
      is_setting_for(r,r)
    ]).
/** <module> DUL notion of Situation.

@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
    [ has_type/2 ]).
:- use_module('./Event.pl',
    [ is_event/1,
      is_action/1
    ]).
:- use_module('./Object.pl',
    [ is_object/1,
      is_agent/1
    ]).
:- use_module(library('db/tripledb'),
    [ tripledb_load/2 ]).

% load RDF data
:- tripledb_load('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
    [ graph(tbox),
      namespace(dul)
    ]).

%% is_description(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Description'.
%
% @param Entity An entity IRI.
%
is_description(Entity) ?+>
  has_type(Entity, dul:'Description').

%% is_situation(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Situation'.
%
% @param Entity An entity IRI.
%
is_situation(Entity) ?+>
  has_type(Entity, dul:'Situation').

%% is_goal(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Goal'.
%
% @param Entity An entity IRI.
%
is_goal(Entity) ?+>
  has_type(Entity, dul:'Goal').

%% is_design(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Design'.
%
% @param Entity An entity IRI.
%
is_design(Entity) ?+>
  has_type(Entity, dul:'Design').

%% is_diagnosis(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Diagnosis'.
%
% @param Entity An entity IRI.
%
is_diagnosis(Entity) ?+>
  has_type(Entity, dul:'Diagnosis').

%% is_plan(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Plan'.
%
% @param Entity An entity IRI.
%
is_plan(Entity) ?+>
  has_type(Entity, dul:'Plan').

%% is_plan_execution(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PlanExecution'.
%
% @param Entity An entity IRI.
%
is_plan_execution(Entity) ?+>
  has_type(Entity, dul:'PlanExecution').

% TODO
%is_plan_execution(Entity,Plan,Action) ?+>
  %is_plan_execution(Sit),
  %is_plan(Plan),
  %holds(Sit,dul:satisfies,Plan),
  %holds(Sit,dul:includesEvent,Action),
  %holds(Plan,ease:isPlanFor,Tsk),
  %holds(Action,dul:executesTask,Tsk).

%% is_norm(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Norm'.
%
% @param Entity An entity IRI.
%
is_norm(Entity) ?+>
  has_type(Entity, dul:'Norm').

%% is_transition(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Transition'.
%
% @param Entity An entity IRI.
%
is_transition(Entity) ?+>
  has_type(Entity, dul:'Transition').

%% plan_has_goal(?Plan,?Goal) is nondet.
%
% Relates a plan to its goal.
%
% @param Plan An individual of type dul:'Plan'.
% @param Goal An individual of type dul:'Description'.
%
plan_has_goal(Plan,Goal) ?+>
  holds(Plan,dul:hasComponent,Goal),
  has_type(Goal,dul:'Goal').

%% satisfies(?Sit,?Descr) is nondet.
%
% A relation between a Situation and a Description,
% e.g. the execution of a Plan satisfies that plan.
%
% @param Sit An individual of type dul:'Situation'.
% @param Descr An individual of type dul:'Description'.
%
satisfies(Sit,Descr) ?+>
  holds(Sit,dul:satisfies,Descr).

%% is_setting_for(+Sit,+Entity) is nondet.
%
% A relation between situations and entities, e.g.
% 'this morning I've prepared my coffee with a new fantastic Arabica',
% i.e.: the preparation of my coffee this morning is the setting
% for (an amount of) a new fantastic Arabica.
%
% @param Sit An individual of type dul:'Situation'.
% @param Entity A named individual.
%
is_setting_for(Sit,Entity) ?>
  holds(Sit,dul:isSettingFor,Entity).

is_setting_for(Sit,Entity) +>
  { ask(is_action(Entity)) },
  holds(Sit,dul:includesAction,Entity).

is_setting_for(Sit,Entity) +>
  { ask(is_event(Entity)) },
  holds(Sit,dul:includesEvent,Entity).

is_setting_for(Sit,Entity) +>
  { ask(is_agent(Entity)) },
  holds(Sit,dul:includesAgent,Entity).

is_setting_for(Sit,Entity) +>
  { ask(is_object(Entity)) },
  holds(Sit,dul:includesObject,Entity).

is_setting_for(Sit,Entity) +>
  holds(Sit,dul:isSettingFor,Entity).
