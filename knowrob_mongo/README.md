knowrob_mongo
===

This package implements a mongo client for KnowRob that transparently
integrates into the reasoning process by generating symbols on
demand from mongo database records.

Each DB collection corresponds to a ROS topic and is named accordingly,
the records in collections are messages published on the topic
and their structure is given by ROS message definitions.
Using [mongodb-log](https://github.com/code-iai/ros-mongodb_log) 
ROS messages can be recorded in advance or while KnowRob is running.
ROS messages are usually stamped, and thus a timestamp value ends up
in the DB records that should be used for indexing such that
KnowRob can quickly find records given some time instant.

At the moment, this module uses a state that corresponds to the currently
active mongo database.
The state can be changed using mng_db/1.

### Querying DB objects
The main entry point for querying mng_query/3.
Its first argument is the name of the collection from which records
should be retrieved (i.e., the topic name).
The second argument is used to hold the retrieved record(s),
and the last argument is a key-value pattern that must be fulfilled
by matching DB objects.

Another useful predicate is mng_query_latest/4
that yields ordered records before some constant time instant (latest record first).
For example, to read only the latest image published
on the topic "kinect_head_rgb_image_color", and before time Instant, one can write:
==
mng_query_latest('kinect_head_rgb_image_color', one(DBObj), 'header.stamp', Instant).
==

Finally, mng_query_incremental/3 allows to hook a Prolog goal into
incrementally querying the mongo DB to allow integration of
mongo query processing with Prolog
backtracking.

### Republishing messages
This package also implements a message factory that is able to generate
ROS messages from DB records, and to republish them on a user specified topic.
This can be done with the predicate mng_republish/5
for which users must specify DB object (queried before),
message type specifiers, and the topic on which the message should
be published.

For example, camera info messages could be republished like this:
==
mng_republish(DBObj, 'sensor_msgs.CameraInfo', 'sensor_msgs/CameraInfo', 'camera_info', Msg).
==
