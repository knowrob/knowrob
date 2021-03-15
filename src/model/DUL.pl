:- module(model_DUL,
	[ is_object(r),
	  is_quality(r),  % ?Quality
	  is_concept(r),
	  is_role(r),   % ?Role
	  is_agent(r),
	  is_physical_object(r),
	  is_physical_artifact(r),
	  is_social_object(r),
	  is_event(r),
	  is_event_type(r),
	  is_action(r),
	  is_process(r),
	  is_task(r),
      is_region(r),
      is_parameter(r),
      is_time_interval(r),
      is_space_region(r),
      is_amount(r),
      is_physical_attribute(r),
      is_social_attribute(r),
      is_description(r),
      is_situation(r),
      is_goal(r),
      is_design(r),
      is_diagnosis(r),
      is_plan(r),
      is_plan_execution(r),
      is_norm(r),
      is_transition(r),
	  has_object_type(r,r),
	  has_quality_type(r,r),
	  has_location(r,r),
	  has_role(r,r), % ?Object, ?Role
	  has_part(r,r),
	  % FIXME: pl module process defines this already.
	  %        - probably better assert all into another module then user,
	  %          then prefer this over user
	  %is_process(r),
	  is_classified_by(r,r),
	  has_participant(r,r),
	  has_participant(r,r,r),
	  executes_task(r,r),
	  has_task_role(r,r),
	  task_role(r,r),
	  task_role_type(r,r,r),
	  task_role_range(r,r,r),
      has_region(r,r),
      has_parameter(r,r),
      has_parameter(r,r,r),
      has_parameter_range(r,r,r),
      has_assignment(r,r),
      has_data_value(r,?),
      has_time_interval(r,r),
      plan_has_goal(r,r),
      satisfies(r,r),
      is_setting_for(r,r)
	]).
/** <module> Predicates for the DOLCE+DnS Ultralite ontology.

DUL is a lightweight, easy-to-apply foundational ontology
for modeling physical and social contexts.

@see http://www.ontologydesignpatterns.org/ont/dul/DUL.owl
@author Daniel Beßler
@license BSD
*/

:- use_module(library('model/RDFS'),
	[ has_type/2 ]).
:- use_module(library('lang/db'),
	[ load_owl/2 ]).

% load RDF data
:- load_owl('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
	[ namespace(dul) ]).
:- load_owl('http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl',
	[ namespace(io) ]).

%% is_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Object'.
%
% @param Entity An entity IRI.
%
is_object(Entity) ?+>
	has_type(Entity, dul:'Object').

%% is_quality(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Quality'.
%
% @param Entity An entity IRI.
%
is_quality(Entity) ?+>
	has_type(Entity, dul:'Quality').

%% is_concept(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Concept'.
%
% @param Entity An entity IRI.
%
is_concept(Entity) ?+>
	has_type(Entity, dul:'Concept').

%% is_role(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Role'.
%
% @param Entity An entity IRI.
%
is_role(Entity) ?+>
	has_type(Entity, dul:'Role').

%% is_agent(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Agent'.
%
% @param Entity An entity IRI.
%
is_agent(Entity) ?+>
	has_type(Entity, dul:'Agent').

%% is_physical_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PhysicalObject'.
%
% @param Entity An entity IRI.
%
is_physical_object(Entity) ?+>
	has_type(Entity, dul:'PhysicalObject').

%% is_physical_artifact(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PhysicalArtifact'.
%
% @param Entity An entity IRI.
%
is_physical_artifact(Entity) ?+>
	has_type(Entity, dul:'PhysicalArtifact').

%% is_social_object(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SocialObject'.
%
% @param Entity An entity IRI.
%
is_social_object(Entity) ?+>
	has_type(Entity, dul:'SocialObject').

%% has_quality_type(+Entity,?Type) is nondet.
%
% Relates an entity to its types that are sub-classes
% of the Quality concept.
%
% @param Entity named individual
% @param Type class resource
%
has_quality_type(Entity,Type) ?>
	has_type(Entity,Type),
	triple(Type,rdfs:subClassOf,dul:'Quality').

%% has_object_type(+Entity,?Type) is nondet.
%
% Relates an entity to its types that are sub-classes
% of the Object concept.
%
% @param Entity named individual
% @param Type class resource
%
has_object_type(Entity,Type) ?>
	has_type(Entity,Type),
	triple(Type,rdfs:subClassOf,dul:'Object').

%% has_location(?Object, ?Location) is nondet.
%
% A generic, relative spatial location, holding between any entities. E.g.
% - 'the cat is on the mat',
% - 'Omar is in Samarcanda',
% - 'the wound is close to the femural artery'.
%
% @param Object named individual
% @param Location named individual
%
has_location(Object, Location) ?+>
	holds(Object, dul:hasLocation, Location).

%% has_role(?Entity,?Role) is nondet.
%
% Relates an object to its roles.
%
% @param Entity named individual
% @param Role named individual
%
has_role(Entity,Role) ?+>
	holds(Role, dul:classifies, Entity).

%% has_part(?Entity,?Part) is nondet.
%
% Relates an object to its parts.
%
% @param Entity IRI atom
% @param Part IRI atom
%
% TODO: not object related, probably best to add a module DUL.pl and add it there
%
has_part(Entity,Part) ?+>
	holds(Entity, dul:hasPart, Part).

%% is_event(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Event'.
%
% @param Entity An entity IRI.
%
is_event(Entity) ?+>
	has_type(Entity, dul:'Event').

%% is_event_type(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'EventType'.
%
% @param Entity An entity IRI.
%
is_event_type(Entity) ?+>
	has_type(Entity, dul:'EventType').

%% is_action(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Action'.
%
% @param Entity An entity IRI.
%
is_action(Entity) ?+>
	has_type(Entity, dul:'Action').

%% is_task(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Task'.
%
% @param Entity An entity IRI.
%
is_task(Entity) ?+>
	has_type(Entity, dul:'Task').

%% is_process(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Process'.
%
% @param Entity An entity IRI.
%
is_process(Entity) ?+>
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

%% is_classified_by(+Evt, ?Tsk) is nondet
%
% 
%
% @param Evt The Event
% @param Task The task that classifies the Event
is_classified_by(Evt, Task) ?+>
	holds(Evt,dul:isClassifiedBy,Task).

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

has_task_role(Tsk,Role) ?+>
	task_role(Tsk,Role).

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


%% is_region(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Region'.
%
% @param Entity An entity IRI.
%
is_region(Entity) ?+>
	has_type(Entity, dul:'Region').

%% is_parameter(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Parameter'.
%
% @param Entity An entity IRI.
%
is_parameter(Entity) ?+>
	has_type(Entity, dul:'Parameter').

%% is_space_region(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SpaceRegion'.
%
% @param Entity An entity IRI.
%
is_space_region(Entity) ?+>
	has_type(Entity, dul:'SpaceRegion').

%% is_amount(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'Amount'.
%
% @param Entity An entity IRI.
%
is_amount(Entity) ?+>
	has_type(Entity, dul:'Amount').

%% is_physical_attribute(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'PhysicalAttribute'.
%
% @param Entity An entity IRI.
%
is_physical_attribute(Entity) ?+>
	has_type(Entity, dul:'PhysicalAttribute').

%% is_social_attribute(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'SocialAttribute'.
%
% @param Entity An entity IRI.
%
is_social_attribute(Entity) ?+>
	has_type(Entity, dul:'SocialAttribute').

%% is_time_interval(?Entity) is nondet.
%
% True iff Entity is an instance of dul:'TimeInterval'.
%
% @param Entity An entity IRI.
%
is_time_interval(Entity) ?+>
	has_type(Entity, dul:'TimeInterval').

%% has_region(?Entity,?Region) is nondet.
%
% A relation between entities and regions, e.g.
% - 'the number of wheels of that truck is 12',
% - 'the time of the experiment is August 9th, 2004',
% - 'the whale has been localized at 34 degrees E, 20 degrees S'.
%
% @param Entity An entity IRI.
% @param Region An region IRI.
%
has_region(Entity,Region) ?+>
	holds(Entity, dul:hasRegion, Region).

%% has_parameter(?Entity,?Param) is nondet.
%
% A Concept can have a Parameter that constrains the attributes
% that a classified Entity can have in a certain Situation,
% e.g. a WheelDriver Role definedIn the ItalianTrafficLaw has
% a MinimumAge parameter on the Amount 16.
%
% @param Entity An entity IRI.
% @param Param An parameter IRI.
%
has_parameter(Entity,Param) ?+>
	holds(Entity,dul:hasParameter,Param).

%% has_parameter(?Entity,?Param,?ParamType) is nondet.
%
% Same as has_parameter/2 but in addition unifies the
% type of the parameter with the third argument.
%
% @param Entity entity IRI.
% @param Param parameter IRI.
% @param Param parameter type IRI.
%
has_parameter(Entity,Param,ParamType) ?>
	holds(Entity,dul:hasParameter,Param),
	has_object_type(Param,ParamType).

%% has_parameter_range(?Entity,?Param,?Range) is nondet.
%
% Same as has_parameter/2 but in addition unifies the
% range of the parameter with the third argument.
%
% @param Entity entity IRI.
% @param Param parameter IRI.
% @param Range parameter range IRI.
%
has_parameter_range(Entity,Param,Range) ?>
	holds(Entity,dul:hasParameter,Param),
	holds(Param,dul:classifies,only(Range)).

%% has_assignment(?Param,?Region) is nondet.
%
% Associates a parameter to an assignment.
%
% @param Param parameter IRI.
% @param Region region IRI.
%
has_assignment(Param,Region) ?+>
	holds(Param,dul:classifies,Region).

%% has_data_value(?Entity,?DataValue) is nondet.
%
% A datatype property that encodes values from a datatype for an Entity. 
%
% @param Entity entity IRI.
% @param DataValue typed data value.
%
has_data_value(Entity,DataValue) ?+>
	triple(Entity,dul:hasDataValue,DataValue).

%% has_time_interval(+Entity,?Interval) is semidet. 
%
%
has_time_interval(Entity,TimeInterval) ?+>
	triple(Entity,dul:hasTimeInterval,TimeInterval).

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
	triple(Sit,dul:isSettingFor,Entity).

is_setting_for(Sit,Entity) +>
	(	is_action(Entity) -> triple(Sit,dul:includesAction,Entity)
	;	is_event(Entity)  -> triple(Sit,dul:includesEvent,Entity)
	;	is_agent(Entity)  -> triple(Sit,dul:includesAgent,Entity)
	;	is_object(Entity) -> triple(Sit,dul:includesObject,Entity)
	;	triple(Sit,dul:isSettingFor,Entity)
	).
