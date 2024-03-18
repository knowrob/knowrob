\page ros_tf TF integration

KnowRob stores the most recent transforms in local memory.
This is also the source of data for a TF publisher running in this module.
There is a distinction between frames that are thought to be "managed" by KnowRob
and those that are not.
KnowRob will only publish managed frames on the TF topic.
This is needed to avoid clashes between other TF publishing nodes such as the
robot_state_publisher and the one in KnowRob.


## TF logging

This module includes a logger for TF messages stored in a mongo database.
The logger keeps track of previous transforms, and only writes into the database
if some threshold was exceeded.
TF messages are not stored in array format as this would complicate indexing the data.
The logger can be toggled on or off through a setting:

    setting(tf:use_logger, false).


## TF fluent

The fastest way to query persisted TF data is through the fluent predicate tf/4:

| Predicate | Arguments |
| ---   | --- |
| tf/4  | +Frame, -ReferenceFrame, -Position, -Rotation |

The predicate compiles into a mongo DB query, and can be potentially
combined with other goals that can be computed server-side by mongo DB.
As it is defined as a fluent predicate, it will allways yield the most up-to-date
TF record of the child frame.
It also supports querying past fluent values in case they have been persisted in the
database.
There exists another variant with two arguments where the second argument is a list
of parent frame, position, and rotation.
Note that this will only work if TF data is stored in mongo DB, e.g. by using the TF logger
in this module.

It is also possible to write into the database by referring to tf/4 in an
assertion, as in:

    kb_call(assert(tf(myframe, map, [0,0,0], [0,0,0,1]))).

However, this will _not_ update the internal TF memory KnowRob maintains
(see tf_mem_set_pose/3).
Further note that tf/4 does not allow to request a pose in some specific reference frame,
for that purpose is_at/2 can be used.


## TF computable

In many cases, a pose needs to be transformed into some specific reference frame in which
it was not persisted in the database.
A computable predicate is_at/2 is defined to support exactly this: to compute the pose of
a frame wrt. some reference frame.
Within this module, is_at/2 is implemented by the predicate tf_get_pose/4.
For example:

    kb_call(is_at(ns:'MyObject', [target_frame, Position, Rotation]))).

