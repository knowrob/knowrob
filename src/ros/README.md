knowrob_comm
=======

Communication interfaces that are used to handle messages in the knowledge base.
In particular, these are interfaces to the ROS ecosystem and an internal messaging system
used to notify components of the knowledge base.

### object_state_publisher.py

An additional script is provided that is used to publish 
object transforms on the ROS tf topic along with
marker visualization messages.
This is realized as a rosprolog client that 
initially loads all known objects, and
provides a service to mark objects as "dirty" which causes
the script to reload the dirty objects, and
to republish tf and marker visualization messages.

You can run the publisher with:

    rosrun knowrob object_state_publisher.py
