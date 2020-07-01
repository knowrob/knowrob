KnowRob
=======

![CI](https://github.com/daniel86/knowrob/workflows/CI/badge.svg)

KnowRob is a knowledge processing system designed for robots.
Its purpose is to equip robots with the capability to organize information in re-usable
knowledge chunks, and to perform reasoning in an expressive logic.
It further provides a set of tools for visualization and acquisition of knowledge.

In order to interact with other robot components,
the [Robot Operating System](https://www.ros.org/) (ROS)
is used via [rosprolog](https://github.com/knowrob/rosprolog).
*rosprolog* provides a ROS node that manages a pool of Prolog engines
in which KnowRob can be loaded such that its querying interface
is exposed via the node.

The core of KnowRob is an extendible querying interface that
provides basic operations *ask*, *tell*, *forget*, and *remember*.
Their argument is some statement written in the *KnowRob Querying Language*.
Language phrases are terms whose semantics is defined
in form of Prolog rules using special operators such as *?>* (the ask operator),
or *+>* (the tell operator).

Knowledge is represented in form of temporalized triples --
each subject-predicate-object triple has an additional field
that restricts the temporal scope in which the statement
represented by the triple is true.
A configurable backend is used to store and retrieve temporalized triples --
as a falback implementation, KnowRob provides a simple MongoDB
implementation of a temporalized triple store.

One important aspect in knowledge representation for robots is that
a lot of knowledge is *implicitly* encoded in the control structures
of the robot. Hence, one goal is to make the knowledge in robot
control structures *explicit*.
KnowRob does that through *Ontology-based Data Access* (OBDA).
So called, *semantic data accessors* are used to map data to symbols in
an ontology, often by accessing some database, or by reading from
a message queue, etc.

KnowRob uses an ensemble of reasoners approach where inferences
of different reasoners are combined into correlated knowledge pieces.
The reason for choosing this approach is that there is no single
formalism that is suited for every reasoning tasks.
Instead, given a problem, one should decide what the most suitable
formalism is to tackle it.
Consequently, KnowRob can be configured to solve specific problems
by loading corresponding reasoning modules that implement a common interface.
KnowRob also ships with a set of reasoning modules including
an (incomplete) OWL reasoner, a SWRL reasoner, and some specialized
reasoning modules that handle domain-specific problems
such as reasoning about time intervals using Allen's interval
calculus.

Please visit http://www.knowrob.org for
more information and [installation instructions](http://www.knowrob.org/installation),
and have a look at additional README files in the *src* directory.
