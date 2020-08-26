KnowRob
=======

![CI](https://github.com/knowrob/knowrob/workflows/CI/badge.svg)

KnowRob is a knowledge processing system designed for robots.
Its purpose is to equip robots with the capability to organize information in re-usable
knowledge chunks, and to perform reasoning in an expressive logic.
It further provides a set of tools for visualization and acquisition of knowledge.

## Getting Started

These instructions will get you a copy of KnowRob up and running on your local machine.

### Prerequisites

- ROS (*ROS melodic* for the master branch)
- SWI Prolog >= 7.6
- libmongoc

### Installation

KnowRob uses the [catkin](http://wiki.ros.org/catkin) buildsystem that has been the main ROS buildsystem.
We have prepared different *.rosinstall* setup files that you can add to your ROS workspace as described [here](http://www.ros.org/wiki/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

```Bash
rosdep update
cd ~/catkin_ws/src
wstool merge https://raw.github.com/knowrob/knowrob/master/rosinstall/knowrob-base.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ~/catkin_ws
catkin_make
```

You may further need to set the *SWI_HOME_DIR* environment variable to the installation location of *swipl*:

```
export SWI_HOME_DIR=/usr/lib/swi-prolog
```

### Launching

In order to interact with other robot components,
the [Robot Operating System](https://www.ros.org/) (ROS)
is used via [rosprolog](https://github.com/knowrob/rosprolog).
*rosprolog* provides a ROS node that manages a pool of Prolog engines
in which KnowRob can be loaded such that its querying interface
is exposed via the node.
KnowRob provides a launch file that starts the *rosprolog* node, and initializes KnowRob:

```
roslaunch knowrob knowrob.launch
```

Please refer to the *rosprolog* documentation for how to interact with this node.
However, KnowRob can also be launched without ROS node through a script offered by *rosprolog*:

```
rosrun rosprolog rosprolog knowrob
```

Launching KnowRob without the ROS node may help debugging.

## Getting Familiar

Here we provide an overview about functionality of KnowRob.

### Querying

The core of KnowRob is an extendible querying interface that
provides basic operations *ask*, *tell*, *forget*, and *remember*.
Their argument is some statement written in the [KnowRob Querying Language](src/lang/README.md).
Language phrases are terms whose semantics is defined
in form of Prolog rules using special operators such as *?>* (the ask operator),
or *+>* (the tell operator).

### Model

KnowRob structures knowledge according to models represented using RDF.
Some models are very basic and domain-independent such as the OWL model
that e.g. distinguishes between object and datatype properties, or the
toplevel ontology SOMA which is supported by KnowRob.
[KnowRob Models](src/model/README.md) is a collection of such models
that are explicitely supported by KnowRob.
However, support for other models may be added through plugins.

### Triple Store and Data Access

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

For more information on database backends in KnowRob, please have a look
[here](src/db/README.md).

### Reasoning

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
More complete information about reasoning in KnowRob can be found
[here](src/reasoning/README.md).

## Further Information

- Sourcecode documentation is available [here](https://knowrob.github.io/knowrob/)
- A blog and more wiki pages are avaiabale at: http://www.knowrob.org
