KnowRob
=======

[![Build Status](https://travis-ci.org/knowrob/knowrob.svg?branch=master)](https://travis-ci.org/knowrob/knowrob)

KnowRob is a knowledge base for robots.
It equips robots with the capability to organize information in re-usable
knowledge chunks, and to perform reasoning in an expressive logic formalism.
It further provides a set of tools for visualization and acquisition of knowledge.

The main programming language used is Prolog. Prolog is used to load RDF data,
and for the declaration of rules that realize KnowRob's QA interface.
Several C++ libraries are wrapped into Prolog rules to make them available
in the QA interface.
RDF data may be loaded from file, or gathered from computational methods
such as webstores, or commensense rules.
This concept is called *virtual knowledge base* in KnowRob.

One of the tasks KnowRob was originally designed for is to bridge the gap between abstract instructions
and their execution.
KnowRob can supply the difference between the (often shallow and symbolic) information in the
instructions and the (detailed, grounded and often real-valued) information needed for execution. For filling these
information gaps, a robot first has to identify them in the instructions, reason about suitable information sources,
and combine pieces of information from different sources and of different structure into a coherent knowledge base.

Another crucial functionality KnowRob provides is the acquisition of *experiential knowledge*.
These are situation-dependant knowledge chunks that describe an epsiode of a robot (or human) executing a task.
Formal descriptions are combined with sensor data which allows to use an expressive language for
the selection of training examples for training a model over the experiences of a robot or human.

This repository contains
KnowRob core packages and general issue tracker for the KnowRob knowledge base.

Please visit http://www.knowrob.org for
more information and [installation instructions](http://www.knowrob.org/installation).
