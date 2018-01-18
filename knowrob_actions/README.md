knowrob_actions
===

Methods for reasoning about OWL action descriptions,
and instances.
Actions are organized in a subsumption hierarchy
and their class description can further include 
information about sub-actions, preconditions,
effects, and actors (i.e., inputs and outputs).

### Changing objects

Actions may change, create, or destroy objects.
Robots may reason about which action has a desired
effect, and if an action can be instantiated to be performed
in the current situation.

Information about object change is expressed as preactors (i.e., entities available prior action execution),
and postactors (i.e., entities that changed in some way during the action).
Action actors span an "Object Transformation Graph" that (implicitly) describes how objects
changed over time under the influence of action.
Following OWL properties express the actors of an action:
[objectActedOn](http://knowrob.org/kb/knowrob.owl#objectActedOn),
[thingIncorporated](http://knowrob.org/kb/knowrob.owl#thingIncorporated),
[objectAddedTo](http://knowrob.org/kb/knowrob.owl#objectAddedTo),
[inputsCommitted](http://knowrob.org/kb/knowrob.owl#inputsCommitted),
[inputsDestroyed](http://knowrob.org/kb/knowrob.owl#inputsDestroyed),
[transformedObject](http://knowrob.org/kb/knowrob.owl#transformedObject),
[objectRemoved](http://knowrob.org/kb/knowrob.owl#objectRemoved),
[objectOfStateChange](http://knowrob.org/kb/knowrob.owl#objectOfStateChange),
[outputsRemaining](http://knowrob.org/kb/knowrob.owl#outputsRemaining),
[outputsCreated](http://knowrob.org/kb/knowrob.owl#outputsCreated)

Action class descriptions may restrict preactors to describe action preconditions,
for instance, that an object of a particular type must be used or
that it needs to satisfy some relation.

### Action planning

Complex tasks may be decomposed into more atomic steps executable by the robot,
such as different motion phases that a robot performs to grasp an object
from a table.
The [subAction](http://knowrob.org/kb/knowrob.owl#subAction) relation is used to describe this hierarchy,
and additional partial ordering constraints can be used to restrict
possible sequences of sub actions.

#### Preconditions

Preconditions are expressed with preactor restrictions.
The fulfillment of the restrictions can simply be checked with owl_individual_of/2.

#### Effects

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
