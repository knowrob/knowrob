KnowRob Reasoning
=======

KnowRob uses the Web Ontology Language (OWL) in combination with
rule based reasoning powered by SWI Prolog.
OWL ontologies can be parsed (owl_parse/1) which causes new facts to be
asserted in the RDF triple store of SWI Prolog.
SWI Prolog already provides querying with RDFS semantics
(rdf_has/3, rdfs_individual_of/2).

Ontop of the RDFS semantics KnowRob implements a minimal OWL reasoner in Prolog.
owl_individual_of/2 checks if a resource satisfies a class description
according to OWL-Description entailment rules,
and owl_has/3 checks if a relation can be deduced using OWL inference rules.
Some OWL2 features such as property chains are also supported,
but the reasoner is not OWL2 complete.

RDF triples may concretely be represented in the RDF triple store of SWI Prolog.
But KnowRob supports multiple sources for triples including
computational methods that ground relations in the data structures of the robot's
control program, or in its senses.
This concept is called *virtual knowledge base* in KnowRob.
Several virtual triple sources are pre-defined in KnowRob
including computable properties for
qualitative spatial reasoning (comp_spatial),
temporal reasoning (knowrob_common),
ROS tf data (knowrob_memory, knowrob_mongo),
and SWRL rules (swrl).

KnowRob is also capable to handle temporal information,
allowing robots to reason about past events.
Please note that OWL is generally considered inappropriate for
explicating time because it is limited to binary relations.
One approach to handle time is to *reify* relations to concepts
that also have time parameters.
The ontology used by KnowRob comes with a few reification concepts
that are handled within KnowRob.
However, KnowRob also supports explicit time in its QA interface. The time
specified is used as parameter for the triple store
(or to select appropiate reifications of relations).
In case of the default RDF triple store of SWI Prolog,
all triples are assumed to hold forever.
However, a dedicated *temporalized* triple store can be used
to represent triples that only hold limited time.
The time value specified in the query is used as parameter
for the triple store in that case.
knowrob_mem implements such a temporalized triple store.
The main interface to access temporalized triples is the
holds/2 predicate:

    holds(knowrob:volumeOfObject(Obj, 15.0), 25.0)

### Action planning

The action model asserts the structure of tasks via plans that decompose them into different steps and phases. Plans further assert a partial ordering between these constituents from which possible sequences of step and phase occurences that would manifest a plan execution can be derived.

To infer possible sequences of steps for a given plan, the following can be used:

    workflow_sequence(+Plan, ?Steps)

This will internally build up a, so called, endpoint sequence graph (ESG), a datastructure that captures temporal relations between event endpoints (i.e. when they start and when they end), and from which all possible sequences can be sampled.

More generally, to read all the steps and phases of a plan, and temporal constraints between them, the following can be used:

    workflow_constituents(+Plan, -Constituents, -Constraints)

Where *Constraints* is a list of binary relations between steps/phases of the plan, for example, the constraint `<(A,B)` means that the occurance of an event of type *A* is strictly before the occurance of an event of type *B*.

### Action detection

A library of plans can be casted as grammar for a parser that yields actions from observed event endpoints which are used as tokens for the parser. This can be used to classify activities given evidence that is more easy to observe such as state changes, contact events, and motions.

A parser can be created given a list of plans:

    parser_create(Parser,[
        ptest:'Grasping_WF0',
        ptest:'Lifting_WF0',
        ....
    ])

Which will create ESG's for each of the plans.

The parser runs in its own thread which can be started by calling `parser_start/1`. Once started, tokens can be added to the parser using the predicate `parser_push_token/2`. When done, the parser can be stopped again by calling `parser_stop/1`. This will synchronize all threads, and block until the parser thread exits. A second variant `parser_stop/2` provides the remaining parser output in its second argument.

    parser_start(Parser),
    parser_push_token(Parser,tok(0.0,c, -(ptest:'Touching'), ['TestHand','TestObject']),
    ...
    parser_stop(Parser,Outputs).

The default mode of the parser is that it yields all actions that are detected no matter if they conflict with each other (i.e. because they provide different interpretations of the same event). However, the predicate `parser_start/2` allows to switch to another mode by setting the *compose* flag in the option list which is the second argument of the predicate. This flag causes the parser to compose individual actions into an activity without conflicting actions.

### Action effects

The effects of actions on objects are derived from the roles the objects play during the action. For example, whenever an object plays the role of being the *AlteredObject* one of its qualities is being altered to some level. Hence the effect of the action is an alteration of the region of the quality which may be automatically updated by KnowRob after the action has been executed. Other distinct cases are objects being destroyed or created in which case KnowRob updates the lifetime of them, and objects being combined, included, or excluded in which case KnowRob may update relations between these objects.

The predicate `action_effect/2` relates an action (or task) to its effects, for example:

    action_effect(pancake:'CrackingAnEgg', destroyed(pancake:'Egg'))

This states that some *Egg* is destroyed when *CrackingAnEgg* tasks are executed. It is also possible to query for tasks that have certain effects by calling this predicate with the first argument ungrounded:

    action_effect(Tsk, destroyed(pancake:'Egg'))

Which corresponds to the question *how can eggs be destroyed*. Finally, with the second argument ungrounded, all known effects of the task are inferred.

Some action effects may imply new relations, for example, that one object is part of another after the action has been performed. Consequently, KnowRob also provides a predicate to maintain these relations automatically based on the roles objects play during the action. To infer and assert these relations, the predicate `action_effects_apply/2` can be used, for example:

    action_effects_apply(pancake:'CrackingAnEgg',_{
      'DestroyedObject': 'Egg_0'
    })

Note that "destroying" an object in KnowRob does not mean that knowledge about it disappears. Each object has an associated lifetime, and an event that destroys some object determines the end of its lifetime rather then vanishing its existence in the knwoledge base.
