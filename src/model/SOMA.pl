:- module(model_SOMA,
	[ is_episode(r),
	  is_manipulation_action(r),
	  is_mental_action(r),
	  is_mental_task(r),
	  is_physical_task(r),
	  is_performed_by(r,r),
	  is_computational_agent(r),
	  is_digital_object(r),
	  is_kino_dynamic_data(r),
	  is_chemical_process(r),
	  is_physical_process(r),
	  is_force_interaction(r),
	  is_motion(r),
	  is_process_flow(r),
	  is_process_type(r),
	  is_progression(r),
	  is_state(r),
	  is_physical_state(r),
	  is_social_state(r),
	  is_configuration(r),
	  is_state_type(r),
	  is_binding(r),
	  is_succeedence(r),
	  has_subevent(r,r),
	  action_status(r,r),
	  action_succeeded(r),
	  action_failed(r),
	  action_active(r),
	  action_paused(r),
	  action_pending(r),
	  action_cancelled(r),
	  event_interval(r,?,?),
	  task_effect(r,t),
	  has_kinematics_file(r,?,?),
	  has_process_role(r,r),
	  plan_defines_task(r,r),
	  workflow_step(r,r),
	  workflow_first_step(r,r),
	  workflow_role_range(r,r,r),
	  workflow_constituent(r,r),
      has_interval_begin(r,?),
      has_interval_end(r,?),
      has_interval_duration(r,?),
      is_interval_equal(r,r),
	  %% Features
	  is_feature(r),
	  object_feature(r,r),
	  object_feature_type(r,r,r),
	  %% Affordances
	  is_affordance(r),
	  is_disposition(r),
	  has_disposition(r,r),
	  has_disposition_type(r,r,r),
	  disposition_trigger_type(r,r),
	  %% Roles & Parameters
	  is_patient(r),
	  is_instrument(r),
	  is_location(r),
	  is_destination(r),
	  is_origin(r),
	  %% Qualities
	  is_physical_quality(r),
	  is_social_quality(r),
	  is_intrinsic(r),
	  is_extrinsic(r),
	  object_localization(r,r),
	  object_shape(r,?,?,?,?),
	  object_shape_type(r,r),
	  object_mesh_path(r,?),
	  object_color_rgb(r,?,?,?),
	  object_dimensions(r,?,?,?)
	]).
/** <module> Predicates for the SOMA ontology.

@author Daniel Beßler
@license BSD
*/

% TODO: how to handle assert new quality values if quality
%		does not exist yet? --> should be done via a special predicate

:- use_module(library('semweb/rdf_db'),
	[ rdf_equal/2 ]).
:- use_module('RDFS',
	[ has_type/2 ]).
:- use_module('DUL',
	[ has_parameter_range/3, task_role_range/3 ]).

:- load_owl('http://www.ease-crc.org/ont/SOMA.owl',
	[ namespace(soma) ]).

:- multifile object_shape/5.

		 /*******************************
		 *	    ACTIONS		*
		 *******************************/

%% is_manipulation_action(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ManipulationAction'.
%
% @param Entity An entity IRI.
%
is_manipulation_action(Entity) ?+>
	has_type(Entity, soma:'ManipulationAction').

%% is_mental_action(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'MentalAction'.
%
% @param Entity An entity IRI.
%
is_mental_action(Entity) ?+>
	has_type(Entity, soma:'MentalAction').

%% is_physical_task(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalTask'.
%
% @param Entity An entity IRI.
%
is_physical_task(Entity) ?+>
	has_type(Entity, soma:'PhysicalTask').

%% is_mental_task(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'MentalTask'.
%
% @param Entity An entity IRI.
%
is_mental_task(Entity) ?+>
	has_type(Entity, soma:'MentalTask').

%% is_episode(?Entity) is nondet.
%
is_episode(Entity) ?+>
	has_type(Entity, soma:'Episode').

%% is_performed_by(?Act,?Agent) is nondet.
%
% Relates an action to the agent that performs it.
%
% @param Act An individual of type dul:'Action'.
% @param Agent An individual of type dul:'Agent'.
%
is_performed_by(Act,Agent) ?+>
	holds(Act, soma:isPerformedBy, Agent).

%% action_status(?Act,?Status) is semidet.
%
% Relates an action to its execution status.
%
% @param Act An individual of type dul:'Action'.
% @param Status The execution status of Act.
%
action_status(Act,Status) ?+>
	holds(Act, soma:hasExecutionState, Status).

%% action_succeeded(?Act) is det.
%
% Set the execution status of an action to 'succeeded'.
%
% @param Act An individual of type dul:'Action'.
%
action_succeeded(Act) ?+>
	action_status(Act, soma:'ExecutionState_Succeeded').

%% action_failed(?Act) is det.
%
% Set the execution status of an action to 'failed'.
%
% @param Act An individual of type dul:'Action'.
%
action_failed(Act) ?+>
	action_status(Act, soma:'ExecutionState_Failed').

%% action_active(?Act) is det.
%
% Set the execution status of an action to 'active'.
%
% @param Act An individual of type dul:'Action'.
%
action_active(Act) ?+>
	action_status(Act, soma:'ExecutionState_Active').

%% action_paused(?Act) is det.
%
% Set the execution status of an action to 'paused'.
%
% @param Act An individual of type dul:'Action'.
%
action_paused(Act) ?+>
	action_status(Act, soma:'ExecutionState_Paused').

%% action_pending(?Act) is det.
%
% Set the execution status of an action to 'planning'.
%
% @param Act An individual of type dul:'Action'.
%
action_pending(Act) ?+>
	action_status(Act, soma:'ExecutionState_Pending').

%% action_cancelled(?Act) is det.
%
% Set the execution status of an action to 'cancelled'.
%
% @param Act An individual of type dul:'Action'.
%
action_cancelled(Act) ?+>
	action_status(Act, soma:'ExecutionState_Cancelled').

%% event_interval(?Event,?Since,?Until) is nondet.
%
% Returns the start and end time of an event
%
% @param Evt An individual of type dul:'Event'.
% @param Since begin of the interval
% @param Until end of the interval
%
event_interval(EV, Since, Until) ?>
	triple(EV, dul:hasTimeInterval, TI),
	triple(TI, soma:hasIntervalBegin, Since),
	triple(TI, soma:hasIntervalEnd, Until).

event_interval(EV, Since, Until) +>
	new_iri(TI, dul:'TimeInterval'),
	has_type(TI, dul:'TimeInterval'),
	triple(EV, dul:hasTimeInterval, TI),
	triple(TI, soma:hasIntervalBegin, Since),
	triple(TI, soma:hasIntervalEnd, Until).

%% task_effect(?EventType, ?Effect) is nondet
%
% Relates an action or task to roles that imply change,
% and that need to be taken by some object when the task is executed.
%
%	| created(Type)			| An object has been created.	|
%	| destroyed(Type)		| An object has been destroyed.	|
%	| altered(Type,QualityType)	| An object has been changed.	|
%	| linked(Type)			| An object has been linked. 	|
%	| deposited(Type)		| An object has been deposited. |
%	| commited(Type)		| An object has been commited. 	|
%	| included(Type)		| An object has been included.	|
%	| extracted(Type)		| An object has been extracted.	|
%
% @param ActOrTsk An individual of type dul:'Action' or dul:'Task', or a subclass of dul:'Task'.
%
task_effect(Tsk,Effect) :-
	ground(Effect)
	->	once(task_effect_(Tsk,Effect))
	;	task_effect_(Tsk,Effect).

task_effect_(Tsk, created(Type)) :-
	task_role_range(Tsk,soma:'CreatedObject',Type).

task_effect_(Tsk, destroyed(Type)) :-
	task_role_range(Tsk,soma:'DestroyedObject',Type).

task_effect_(Tsk, linked(Type)) :-
	task_role_range(Tsk,soma:'LinkedObject',Type).

task_effect_(Tsk, commited(Type)) :-
	task_role_range(Tsk,soma:'CommitedObject',Type).

task_effect_(Tsk, deposited(Type)) :-
	task_role_range(Tsk,soma:'DepositedObject',Type).

task_effect_(Tsk, extracted(Type)) :-
	task_role_range(Tsk,soma:'ExtractedObject',Type).

task_effect_(Tsk, included(Type)) :-
	task_role_range(Tsk,soma:'IncludedObject',Type).

task_effect_(Tsk, altered(Type,QualityType)) :-
	task_role_range(Tsk,soma:'AlteredObject',Type),
	has_parameter_range(Tsk,soma:'Setpoint',Region),
	get_altered_quality_type_(Type,Region,QualityType).

%%
get_altered_quality_type_(Concept,Region,Quality_type) :-
	get_altered_quality_type__(Concept,Region,Quality_type),
	\+ rdf_equal(Quality_type,dul:'Quality'),!.

get_altered_quality_type__(_Concept,Region,Quality_type) :-
	holds(Region, dul:isRegionFor, only(Quality_type)).

get_altered_quality_type__(Concept,_Region,Quality_type) :-
	holds(Concept, soma:isTriggerDefinedIn, only(Affordance)),
	subclass_of(Affordance,only(soma:describesQuality,Quality_type)).

		 /*******************************
		 *	    IO		*
		 *******************************/

%% is_computational_agent(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'ComputationalAgent'.
%
% @param Entity An entity IRI.
%
is_computational_agent(Entity) ?+>
	has_type(Entity, soma:'ComputationalAgent').

%% is_digital_object(?Entity) is semidet.
%
% True iff Entity is an instance of soma:'DigitalObject'.
%
% @param Entity An entity IRI.
%
is_digital_object(Entity) ?+>
	has_type(Entity, soma:'DigitalObject').

%% has_kinematics_file(?OBJ,?DOI,?Format) is semidet.
%
is_kino_dynamic_data(IO) ?+>
	has_type(IO, soma:'KinoDynamicData').

%% has_kinematics_file(?OBJ,?DOI,?Format) is semidet.
%
% Associates an object to KinoDynamicData about the object.
%
% @param Obj An entity IRI.
% @param Identifier The DOI of a data file.
% @param Format File format identifier string (i.e. the file extension).
%
has_kinematics_file(Obj,Identifier,Format) +>
	new_iri(IO,soma:'KinoDynamicData'),
	new_iri(IR,io:'DigitalResource'),
	has_type(IO,soma:'KinoDynamicData'),
	has_type(IR,io:'DigitalResource'),
	triple(IO, dul:isAbout, Obj),
	triple(IR, dul:realizes, IO),
	triple(IR, soma:hasPersistentIdentifier, Identifier),
	triple(IR, soma:hasDataFormat, Format).

has_kinematics_file(Obj,Identifier,Format) ?>
	triple(IO, dul:isAbout, Obj),
	has_type(IO,soma:'KinoDynamicData'),
	triple(IR, dul:realizes, IO),
	triple(IR, soma:hasPersistentIdentifier, Identifier),
	triple(IR, soma:hasDataFormat, Format).

		 /*******************************
		 *	    PROCESSES/STATES		*
		 *******************************/

%% is_chemical_process(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ChemicalProcess'.
%
% @param Entity An entity IRI.
%
is_chemical_process(Entity) ?+>
	has_type(Entity, soma:'ChemicalProcess').

%% is_physical_process(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalProcess'.
%
% @param Entity An entity IRI.
%
is_physical_process(Entity) ?+>
	has_type(Entity, soma:'PhysicalProcess').

%% is_process_flow(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ProcessFlow'.
%
% @param Entity An entity IRI.
%
is_process_flow(Entity) ?+>
	has_type(Entity, soma:'ProcessFlow').

%% is_process_type(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ProcessType'.
%
% @param Entity An entity IRI.
%
is_process_type(Entity) ?+>
	has_type(Entity, soma:'ProcessType').

%% is_motion(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Motion'.
%
% @param Entity An entity IRI.
%
is_motion(Entity) ?+>
	has_type(Entity, soma:'Motion').

%% is_force_interaction(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'ForceInteraction'.
%
% @param Entity An entity IRI.
%
is_force_interaction(Entity) ?+>
	has_type(Entity, soma:'ForceInteraction').

%% is_progression(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Progression'.
%
% @param Entity An entity IRI.
%
is_progression(Entity) ?+>
	has_type(Entity, soma:'Progression').

%% has_process_role(?ProcType,?Role) is nondet.
%
% A relation between roles and process types.
%
% @param ProcType An individual of type soma:'ProcessType'.
% @param Role An individual of type dul:'Role'.
%
has_process_role(Tsk,Role) ?+>
	holds(Tsk, soma:isProcessTypeOf ,Role).

%% is_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'State'.
%
% @param Entity An entity IRI.
%
is_state(Entity) ?+>
	has_type(Entity, soma:'State').

%% is_physical_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalState'.
%
% @param Entity An entity IRI.
%
is_physical_state(Entity) ?+>
	has_type(Entity, soma:'PhysicalState').

%% is_social_state(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'SocialState'.
%
% @param Entity An entity IRI.
%
is_social_state(Entity) ?+>
	has_type(Entity, soma:'SocialState').

%% is_configuration(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Configuration'.
%
% @param Entity An entity IRI.
%
is_configuration(Entity) ?+>
	has_type(Entity, soma:'Configuration').

%% is_state_type(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'StateType'.
%
% @param Entity An entity IRI.
%
is_state_type(Entity) ?+>
	has_type(Entity, soma:'StateType').


%% has_subevent(+Event,?Sub) is nondet.
%
%
has_subevent(Event,Sub) ?>
	holds(Event,dul:hasConstituent,Sub).

has_subevent(Event,Sub) ?>
	holds(Event,soma:hasPhase,Sub).

has_subevent(Event,Sub) +>
	(	is_action(Sub)  -> holds(Event,dul:hasConstituent,Sub)
	;	is_process(Sub) -> holds(Event,soma:hasPhase,Sub)
	;	is_state(Sub)   -> holds(Event,soma:hasPhase,Sub)
	;	fail
	).


		 /*******************************
		 *	    DESCRIPTIONS		*
		 *******************************/

%% is_binding(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Binding'.
%
% @param Entity An entity IRI.
%
is_binding(Entity) ?+>
	has_type(Entity, soma:'Binding').

%% is_succeedence(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Succeedence'.
%
% @param Entity An entity IRI.
%
is_succeedence(Entity) ?+>
	has_type(Entity, soma:'Succeedence').

%% plan_defines_task(?Plan,?Tsk) is semidet.
%
% Relates a plan to the task it defines.
%
% @param Plan An individual of type dul:'Plan'.
% @param Tsk An individual of type dul:'Task'.
%
plan_defines_task(Plan,Tsk) ?+>
	holds(Plan,soma:isPlanFor,Tsk).

%% plan_has_goal(?WF,?Step) is semidet.
%
% Relates a workflow to one of its steps.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_step(WF,Step) ?+>
	holds(WF,soma:hasStep,Step).

%% workflow_first_step(?WF,?Step) is semidet.
%
% Relates a workflow to the dedicated first step of it.
%
% @param WF An individual of type dul:'Workflow'.
% @param Step An individual of type dul:'Task'.
%
workflow_first_step(WF,Step) ?+>
	holds(WF,soma:hasFirstStep,Step).

%% workflow_constituent(+WF, ?Constituent) is semidet.
%
% @param WF A workflow.
%
workflow_constituent(WF,X) ?>
	triple(WF,in([
		string(dul:describes),
		string(dul:definesTask),
		string(soma:definesProcess)
	]), X).

%% workflow_role_range(?WF,?Role,?ObjectType) is semidet.
%
% Relates a workflow to roles of objects defined by the tasks
% of the workflow, and also infers the required type for fillers
% of the role.
%
% @param WF An individual of type dul:'Workflow'.
% @param Role An individual of type dul:'Role'.
% @param ObjectType A sub-class of dul:'Object'.
%
workflow_role_range(WF,Role,ObjectType) ?>
	workflow_constituent(WF,Tsk),
	task_role_range(Tsk,Role,ObjectType).

		 /*******************************
		 *	    FEATURES		*
		 *******************************/

%% is_feature(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Feature'.
%
% @param Entity An entity IRI.
%
is_feature(Entity) ?+>
	has_type(Entity,soma:'Feature').

%% object_feature(+Obj, ?Feature) is nondet.
%
% Associates an object resource to features it hosts.
%
% @param Obj      Object resource
% @param Feature  Feature resource
%
object_feature(Obj, Feature) ?+>
	holds(Obj,soma:hasFeature,Feature).

%% object_feature(?Obj, ?Feature, ?FeatureType) is nondet.
%
% Same as object_feature/2 but additionally unifies
% the feature type.
%
% @param Obj      Object resource
% @param Feature  Feature resource
% @param FeatureType  Class resource
%
object_feature_type(Obj, Feature, FeatureType) ?>
	object_feature(Obj,Feature),
	has_object_type(Feature,FeatureType).

		 /*******************************
		 *	    AFFORDANCES		*
		 *******************************/

%% is_affordance(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Affordance'.
%
% @param Entity An entity IRI.
%
is_affordance(Entity) ?+>
	has_type(Entity,soma:'Affordance').

%% is_disposition(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Disposition'.
%
% @param Entity An entity IRI.
%
is_disposition(Entity) ?+>
	has_type(Entity,soma:'Disposition').

%% has_disposition(?Obj, ?Disposition) is nondet.
%
% Relates an object to its dispositions.
%
% @param Obj          Object resource
% @param Disposition  Disposition resource
%
has_disposition(Obj, Disposition) ?+>
	holds(Obj,soma:hasDisposition,Disposition).

%% has_disposition(?Obj:iri, ?Disposition:iri, +DispositionType:iri) is nondet.
%
% Relates an object to one of its dispositions that is an instance
% of given disposition type.
%
% @param Obj               Object resource
% @param Disposition       Disposition resource
% @param DispositionType   Class resource
%
has_disposition_type(Obj, Disposition, DispositionType) ?>
	holds(Obj,soma:hasDisposition,Disposition),
	has_quality_type(Disposition,DispositionType).

%% disposition_trigger_type(?Disposition, ?TriggerType) is nondet.
%
% Relates a disposition to the type of objects that can be the 
% trigger of the disposition.
%
% @param Disposition  Disposition resource
% @param TriggerType  Class resource
%
disposition_trigger_type(Disposition,TriggerType) ?>
	holds(Disposition, soma:affordsTrigger, only(TriggerRole)),
	subclass_of(TriggerRole, only(dul:classifies,TriggerType)).

		 /*******************************
		 *	    ROLES & PARAMETERS		*
		 *******************************/

%% is_patient(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Patient'.
%
% @param Entity An entity IRI.
%
is_patient(Entity) ?>
	has_role(Entity,Role),
	has_object_type(Role,soma:'Patient').

%% is_instrument(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Instrument'.
%
% @param Entity An entity IRI.
%
is_instrument(Entity) ?>
	has_role(Entity,Role),
	has_object_type(Role,soma:'Instrument').

%% is_location(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Location'.
%
% @param Entity An entity IRI.
%
is_location(Entity) ?>
	has_role(Entity,Role),
	has_object_type(Role,soma:'Location').

%% is_destination(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Destination'.
%
% @param Entity An entity IRI.
%
is_destination(Entity) ?>
	has_role(Entity,Role),
	has_object_type(Role,soma:'Destination').

%% is_origin(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Origin'.
%
% @param Entity An entity IRI.
%
is_origin(Entity) ?>
	has_role(Entity,Role),
	has_object_type(Role,soma:'Origin').

		 /*******************************
		 *	    QUALITIES		*
		 *******************************/

%% is_physical_quality(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'PhysicalQuality'.
%
% @param Entity An entity IRI.
%
is_physical_quality(Entity) ?+>
	has_type(Entity, soma:'PhysicalQuality').

%% is_social_quality(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'SocialQuality'.
%

% @param Entity An entity IRI.
%
is_social_quality(Entity) ?+>
	has_type(Entity, soma:'SocialQuality').

%% is_intrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Intrinsic'.
%
% @param Entity An entity IRI.
%
is_intrinsic(Entity) ?+>
	has_type(Entity, soma:'Intrinsic').

%% is_extrinsic(?Entity) is nondet.
%
% True iff Entity is an instance of soma:'Extrinsic'.
%
% @param Entity An entity IRI.
%
is_extrinsic(Entity) ?+>
	has_type(Entity, soma:'Extrinsic').

%% has_interval_begin(I,End) is semidet.
%
% The start time of I 
%
% @param I Time point, interval or temporally extended entity
% 
has_interval_begin(TI, Begin) ?+>
	triple(TI, soma:hasIntervalBegin, Begin).

%% has_interval_end(I,End) is semidet.
%
% The end time of I 
%
% @param I Time point, interval or temporally extended entity
% 
has_interval_end(TI, End) ?+>
	triple(TI, soma:hasIntervalEnd, End).

%% has_interval_duration(Event, Duration) is nondet.
%
% Calculate the duration of the the TemporalThing Event
%
% @param Event Identifier of a TemporalThing
% @param Duration Duration of the event
%
has_interval_duration(TI, Duration) ?>
	has_interval_begin(TI, Begin),
	has_interval_end(TI, End),
	Duration is (End-Begin).

%% is_interval_equal(?I1,?I2) is semidet.
%
% Interval I1 is equal to I2
%
% @param I1 Instance of a knowrob:TimeInterval
% @param I2 Instance of a knowrob:TimeInterval
% 
is_interval_equal(TI1, TI2) ?>
	has_interval_begin(TI1, Begin),
	has_interval_begin(TI2, Begin),
	has_interval_end(TI1, End),
	has_interval_end(TI2, End).

%% object_localization(?Obj, ?Loc) is nondet.
%
% Relates an object to its localization quality.
%
% @param Obj object resource
% @param Loc localization quality
%
object_localization(Obj,Loc) ?+>
	holds(Obj,soma:hasLocalization,Loc).

%% object_shape_type(?Obj, ?ShapeType) is nondet.
%
% Relates an object to the type of its shape(s).
% An object may have multiple shapes associated that
% provide different level of detail.
%
% @param Obj Object resource
% @param ShapeType IRI of shape type
%
object_shape_type(Obj, ShapeType) ?>
	holds(Obj,soma:hasShape,Shape),
	holds(Shape,dul:hasRegion,ShapeRegion),
	has_type(ShapeRegion,ShapeType).

%% object_mesh_path(?Obj, -FilePath) is nondet.
%
% True if FilePath is a path to a mesh file (stl or dae) for Obj.
%
% @param Obj      Object resource
% @param FilePath the file path
%
object_mesh_path(Obj, FilePath) ?+>
	% TODO: get_or_create SHA,REG in projection
	holds(Obj, soma:hasShape, SHA),
	holds(SHA, dul:hasRegion, REG),
	holds(REG, soma:hasFilePath, FilePath),
	!.

object_mesh_path(Obj, FilePath) ?>
	holds(Obj,soma:hasFilePath,FilePath ).

%% object_color_rgb(?Obj, ?R, ?G, ?B) is nondet.
%
% True if Col is the main color of Obj.
% Col is encoded as as [float red, green, blue], on a scale of 0-1.
%
% @param Obj object resource
% @param Col rgb color data
% 
object_color_rgb(OBJ, R, G, B) ?+>
	% TODO: get_or_create COL,REG in projection
	holds(OBJ, soma:hasColor, COL),
	holds(COL, dul:hasRegion, REG),
	holds(REG, soma:hasRGBValue, term([R,G,B])),
	!.

object_color_rgb(OBJ, R, G, B) ?>
	holds(OBJ, soma:hasRGBValue, term([R,G,B])).

%%
shape_bbox(ShapeRegion, Depth, Width, Height) ?+>
	holds(ShapeRegion, soma:hasDepth, Depth),
	holds(ShapeRegion, soma:hasWidth, Width),
	holds(ShapeRegion, soma:hasHeight, Height),
	!.

% TODO
%shape_bbox(ShapeRegion, Diameter, Diameter, Diameter) ?>
%	holds(ShapeRegion, soma:hasRadius, Radius),
%	Diameter is 2 * Radius,
%	!.

%% object_dimensions(?Obj, ?Depth, ?Width, ?Height) is nondet.
%
% True if Depth x Width x Height are (exactly) the extends of the bounding box of Obj.
% NOTE that we use ROS conventions here: Coordinate systems in ROS are
% always right-handed, with X forward, Y left, and Z up. 
%
% @param Obj    Object resource
% @param Depth  Depth of the bounding box (x-dimension)
% @param Width  Width of the bounding box (y-dimension)
% @param Height Height of the bounding box (z-dimension)
% 
object_dimensions(Obj, Depth, Width, Height) ?+>
	% TODO: get_or_create SHA,REG in projection
	holds(Obj, soma:hasShape, SHA),
	holds(SHA, dul:hasRegion, REG),
	shape_bbox(REG, Depth, Width, Height),
	!.

object_dimensions(Obj, Depth, Width, Height) ?>
	shape_bbox(Obj, Depth, Width, Height).


%%
shape_term(SR, mesh(File, [X,Y,Z])) ?>
	triple(SR, soma:hasFilePath, File),
	% FIXME: knowrob namespace should not be used here
	triple(SR, 'http://knowrob.org/kb/knowrob.owl#hasXScale', X),
	triple(SR, 'http://knowrob.org/kb/knowrob.owl#hasYScale', Y),
	triple(SR, 'http://knowrob.org/kb/knowrob.owl#hasZScale', Z),
	!.

shape_term(SR, mesh(File, [1,1,1])) ?>
	triple(SR, soma:hasFilePath, File),
	!.

shape_term(SR, box(X,Y,Z)) ?>
	triple(SR, soma:hasDepth,  X),
	triple(SR, soma:hasWidth,  Y),
	triple(SR, soma:hasHeight, Z),
	!.

shape_term(SR, cylinder(Radius,Length)) ?>
	triple(SR, soma:hasLength, Length),
	triple(SR, soma:hasRadius, Radius),
	!.

shape_term(SR, sphere(Radius)) ?>
	triple(SR, soma:hasRadius, Radius),
	!.

%%
shape_origin(SR, Pos, Rot) ?>
	% FIXME: knowrob namespace should not be used here
	triple(SR,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin),
	triple(Origin, soma:hasPositionVector, Pos),
	triple(Origin, soma:hasOrientationVector, Rot).

%% object_shape(?Obj, ?Frame, ?ShapeTerm, ?ShapeOrigin, ?MaterialTerm) is nondet.
%
% Relates objects to shapes and their origin (usually a pose relative to the object).
% The shape is represented as a Prolog term that encodes the shape type
% and its geometrical properties.
%
% ShapeTerm may be one of:
% - mesh(File,Scale)
% - box(X,Y,Z)
% - cylinder(Radius,Length)
% - sphere(Radius)
%
% ShapeOrigin is a list of frame-position-quaternion.
%
% @param Obj IRI atom
% @param ShapeTerm A shape term
% @param ShapeOrigin The origin of the shape
% @param MaterialTerm List of material properties
%
object_shape(Obj, Frame, ShapeTerm, [Frame,Pos,Rot], material(rgba(R,G,B,A))) ?>
	triple(Obj,soma:hasShape,Shape),
	iri_xml_namespace(Obj, _, Frame),
	% SHAPE
	once(triple(Shape,dul:hasRegion,SR)),
	shape_term(SR, ShapeTerm),
	ignore(shape_origin(SR, Pos, Rot)),
	% COLOR
	% TODO: use object_color_rgb when ignore supports this
	% ignore(object_color_rgba(Obj,R,G,B,A)),
	ignore(once(triple(Obj,soma:hasColor,Color))),
	ignore(once(triple(Color,dul:hasRegion,CR))),
	ignore(triple(CR, soma:hasRGBValue, term([R,G,B]))),
	ignore(triple(CR, soma:hasTransparencyValue,A)).

