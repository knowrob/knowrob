knowrob_household
===

This package holds household specific artifacts
including Prolog reasoning rules and in particular ontologies
describing household concepts.

KnowRob was originally developed for autonomous robots
performing everyday activities in a kitchen environment.
It has been successfully employed for tasks such as making pancake,
popcorn, and pizza as well as setting tables and cleaning them again.
This package was created in an attempt to make the core packages
of KnowRob independent of the household domain, and by that opening
doors for non household application scenarios.

### Household ontology

The household ontology is decomposed along different semantic rooms
that regularly are part of a household.
These are organized in a taxonomy in
household.owl
from where ontologies of different semantic rooms is imported.

Semantic room ontologies are further decomposed into *furniture*, *item*, and *activity* ontology.

The *furniture ontology* describes furniture pieces that are characteristic for a
particular semantic room, 
what they are used for
([typePrimaryFunction-deviceUsedFor](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-deviceUsedFor))
and what they most likely contain
([typePrimaryFunction-containerFor](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-containerFor)).

The *item ontology* describes objects that can regularly be found in some semantic room.
These are usually subclass of
[HumanScaleObject](http://knowrob.org/kb/knowrob.owl#HumanScaleObject),
and can be used by the robot 
in some way for action.
For tools, the primary function is also described, such that robots can reason about
which tool could be used to perform a desired action.
In the case of kitchen, items are mainly subclasses of
[FoodVessel](http://knowrob.org/kb/knowrob.owl#FoodVessel)
and
[FoodUtensil](http://knowrob.org/kb/knowrob.owl#FoodUtensil).
Vessels describe what they usually contain and
for which actions they are used
([typePrimaryFunction-containerFor](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-containerFor),
 [typePrimaryFunction-itemUsedFor](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-itemUsedFor))
and utensils describe with
which objects they are usable and for which actions
([typePrimaryFunction-usableWith](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-usableWith),
 [typePrimaryFunction-toolUsedFor](http://knowrob.org/kb/knowrob.owl#typePrimaryFunction-toolUsedFor)).

The *activity ontology* described actions specific to the semantic room
in terms of action actors (see knowrob_actions).


### Groceries ontology

The kitchen ontology also imports a separate groceries ontology that describes a taxonomy of food and drinks 
regularly appearing in kitchen environments.
These are rather general class descriptions without much visual object information for perception.
This is because groceries may exist in many visual variances, and thus it is hard to put general
statements about the visual appearance of them.
More specific kitchen classes can be found in
[iai-kitchen-objects.owl](https://github.com/code-iai/iai_maps/blob/master/iai_kitchen/owl/iai-kitchen-objects.owl).
