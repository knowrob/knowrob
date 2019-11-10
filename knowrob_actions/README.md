knowrob_actions
===

Methods for reasoning about OWL action descriptions,
and instances.
Actions are organized in a subsumption hierarchy
and their class description can further include 
information about sub-actions, preconditions,
effects, and actors (i.e., inputs and outputs).

### Action model

In KnowRob, an *Action* is defined as an *Event* where at least one agent that participates in the event executes a *Task* which is typically defined in a *Plan*. Tasks are used to classify actions, similar to how roles are used to classify objects within some situational context. There may be multiple plans defining the same task which is useful to capture different ways to achieve the same goal. The distinction between *Action* and *Task* is further important as it enables us to put individual tasks into discourse without referring to a particular execution of them (i.e. an *Action*). This is needed because a *Plan* is a generalization of action executions, abstracting away from individual objects that were involved by only referring to the roles they have played.

**Plans** are used to structure tasks, asserting how they are composed of steps and in which order they should be executed. KnowRob supports relations from Allen's Interval Algebra to assert ordering constraints between steps, and also allows to only specify partial ordering. Each step of a plan is a task itself, and may also be defined by some plan(s). However, the action model of KnowRob allows to go deeper by decomposing a task into *phases*. A phase is a *Process* or *State* that occurs during task execution which includes force dynamic events, and motions. Processes are classified by one of the *ProcessType* concepts, and states are classified by one of the *Gestallt* concepts defined in the model.

<p align="center">
<img src="img/plan.png" width="500">
</p>

**Roles** are used to classify objects that participate in some event. This includes the agent that performed the action, tools that were used, objects that were affected, as well as locations of interest. KnowRob defines a taxonomy of roles with the most general concepts being *AffectedObject*, *Location*, and *Tool*. The list of concepts defined below is comprehensive but might not be complete. However, it provides a rich labelset to classify objects in the scope of an activity. They are further used to implicitely encode pre-conditions of plan executions as the existence of objects that are potential filler of the roles is required.

<p align="center">
<img src="img/classification.png" width="400">
</p>

### Action planning

The action model asserts the structure of tasks via plans that decompose them into different steps and phases. Plans further assert a partial ordering between these constituents from which possible sequences of step and phase occurences that would manifest a plan execution can be derived.

To infer possible sequences of steps for a given plan, the following can be used:

    workflow_sequence(Plan, Steps)

This will internally build up a, so called, endpoint sequence graph (ESG), a datastructure that captures temporal relations between event endpoints (i.e. when they start and when they end), and from which all possible sequences can be sampled.

More generally, to read all the steps and phases of a plan, and temporal constraints between them, the following can be used:

    workflow_constituents(Plan, Constituents, Constraints)

Where *Constraints* is a list of binary relations between steps/phases of the plan, for example, the constraint `<(A,B)` means that the occurance of an event of type *A* is strictly before the occurance of an event of type *B*.

### Action detection

The model of plans can be casted as grammar for a parser that yields actions from observed event endpoints which are used as tokens for the parser. This can be used to classify activities given evidence that is more easy to observe such as state changes, contact events, and motions.

A parser can be created given a list of plans:

    parser_create(Parser,[
        ptest:'Grasping_WF0',
        ptest:'Lifting_WF0',
        ....
    ])

Which will create ESG's for each of the plans.

The parser can then be used to detect an activity by invoking it with a sequence of tokens:

    detect_activity([
        tok(0.0,c, -(ptest:'Touching'),        ['TestHand','TestObject']),
        tok(0.9,a, -(ptest:'Supporting'),      ['TestTable','TestObject']),
        ...],
        DetectedActivity).

### Action effects

Action effects are implemented using SWRL rules represented in an OWL ontology.
They can be projected into the knowledge base after all preactors were specified,
and the action was performed.
The predicate action_effects_apply/2 checks the condition of
each rule associated to the action class,
and if satisfied projects the implications into the RDF triple store.
This builds up the Object Transformation Graph, but also does explicit
assertions, if some relation of an involved object changed according to the SWRL rule.
Rule implications may also create processes that are ongoing until some criterion
is fulfilled.
