knowrob_mongo
===

This package implements a mongo client for KnowRob that transparently
integrates logged data into the reasoning process.
This is accomplished by symbolic data abstraction on demand 
given a reasoning task at hand.
Abstracted symbols correspond to OWL properties in the [KnowRob ontology](http://knowrob.org/doc/knowrob_taxonomy)
(e.g., logged spatial information may be abstracted to some specific spatial relation on demand).

Each DB collection corresponds to a ROS topic and is named accordingly,
the records in collections are messages published on the topic
and their structure is given by ROS message definitions.
Using [mongodb-log](https://github.com/code-iai/ros-mongodb_log) 
ROS messages can be recorded in advance or while KnowRob is running.
ROS messages are usually stamped, and thus a timestamp value ends up
in the DB records that should be used for indexing such that
KnowRob can quickly find records given some time instant.

As an example, consider the following DB layout
which is based on the [PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) message
send via the communication topic `tf`:

    {"tf": {
        "header": {
            "seq":      "integer",
            "stamp":    "time",
            "frame_id": "string"
        },
        "pose": {
            "position":    { "x": "double", "y": "double", "z": "double" },
            "orientation": { "x": "double", "y": "double", "z": "double", "w": "double" }
        }
    }}

In the layout, the name `tf` corresponds to a named collection in the DB that collects
recorded PoseStamped messages send via the named topic `tf`.
The collection should be indexed by the key `header.stamp` such that the mongo server
can quickly filter out records based on the timestamp when they were acquired.
`Note` in case of array values (as for tf) it is important
that the indexed key has identical values for all array members (e.g., the same header stamp
in case of tf) -- a patched [robot state publisher](https://github.com/code-iai/robot_state_publisher)
should be used for robot transforms to avoid this issue.

KnowRob uses a state variable for database selection,
only one database may be active at any time.
Users may use mng_db/1 to select the currently active database,
the default database is named *roslog*.
Note that database selection is not threadsafe at the moment (i.e., DB will be selected
for all threads).

### Querying DB objects
The main entry point for querying is mng_query/3.
Its first argument is the name of the collection from which records
should be retrieved (i.e., the topic name).
The second argument is a compound term used to configure the amount
of records to be queried (one, all, or some), and also binds 
the resulting DB objects to a Prolog variable.
The last argument is a key-value pattern that must be fulfilled
by matching DB objects.
The keys must comply with the structure of the mongo collection.
Nested objects can be retrieved by using multi-level keys separated with
a dot (e.g., 'header.frame_id', or 'pose.position.x').
Values are passed to the mongo server as is with the exception of
terms date(Date) which are mapped to mongo date representation internally.

Another useful predicate is mng_query_latest/4
that yields ordered records before some constant time instant (latest record first).
For example, to read only the latest image published
on the topic `kinect_head_rgb_image_color`, and before time `Instant`, one can write:

    mng_query_latest('kinect_head_rgb_image_color', one(DBObj), 'header.stamp', Instant).


Finally, mng_query_incremental/3 allows to hook a Prolog goal into
incrementally querying the mongo DB, and to allow integration of
mongo query processing with Prolog
backtracking.
This goes along these lines:

    mng_query_incremental('tf', my_goal, [['header.frame_id', 'FRAME1']]).

Where my_goal is called for each resulting DB object. Incremental processing is aborted
as soon as my_goal yields false.

### Republishing messages
This package also implements a message factory that is able to generate
ROS messages from DB records, and to republish them on a user specified topic.
This can be done with the predicate mng_republish/5
for which users must specify DB object (queried before),
message type specifiers, and the topic on which the message should
be published.

For example, camera info messages could be republished like this:

    mng_republish(DBObj, 'sensor_msgs.CameraInfo', 'sensor_msgs/CameraInfo', 'camera_info', Msg).

