\page mongodb_backend MongoDB Backend

This package implements a mongo client using libmongoc-1.0.0,
and declares a couple of Prolog predicates to interact with mongo DB
from Prolog code.
This includes predicates used to store, update or retrieve documents,
as well as exporting, importing collections, and setting up their
search indices.

One usecase is logging of ROS messages,
in which case
each DB collection corresponds to a ROS topic and is named accordingly.
The records in collections are messages published on the topic
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
The collection should be indexed by the value `header.stamp` such that the mongo server
can quickly filter out records based on the timestamp when they were acquired.
`Note` in case of array values (as for tf) it is important
that the indexed value has identical values for all array members (e.g., the same header stamp
in case of tf) -- however, *it is advised to avoid generating indices over arrays*.
For tf, this means that it might be best to store each tranform in an individual document.

### Querying mongo
Documents in mongo DB can be retrieved by first creating a getAnswerCursor on
a named collection, and then reading individual documents until
the getAnswerCursor has reached the last document matching the query.
Cursors can limit the results, sort them, and filter them
according to a pattern given in the query.
The query is generally given as Prolog list holding two elements:
the value (or operator) and the value.
However, to avoid conversion problems, the value must be wrapped in an atom indicating its type.

As an example, below is a query that retrieves documents from a collection named *triples*.
The getAnswerCursor only retrieves documents where the *subject* value has the value "Obj1",
and where the *begin* field has a date value smaller then the Unix timestamp *1579888948.52*.

```Prolog
mng_cursor_create(roslog,triples,Cursor),
mng_cursor_filter(Cursor,['subject',['$eq',string('Obj1')]]),
mng_cursor_filter(Cursor,['begin',['$lte',time(1579888948.52)]]),
mng_cursor_next(Cursor,Doc),
mng_cursor_destroy(Cursor).
```

`Note` Make sure to destroy a getAnswerCursor once you are done with it. Usually you would want to wrap the getAnswerCursor operation into a call of *setup_call_cleanup/3*.

More complex filters may use, e.g., disjunction as in:

```Prolog
mng_cursor_filter(Cursor,['$or',
    array([['end',    ['$gte',time(Stamp)]],
           ['end',    ['$exists',bool(0)]]])]).
```

These expressions are generically mapped to BSON terms. Hence,
any command supported by your mongo server can be written in such
an expression.
