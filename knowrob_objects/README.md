knowrob_objects
===

Utilities for reasoning about objects,
and for maintaining beliefs about them.

In particular relevant are visual and
geometrical properties such as
color (object_color/3),
size (object_dimensions/4),
mesh (object_mesh_path/2), and
pose (current_object_pose/2, object_pose_at_time/3) of an object.

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

    rosrun knowrob_objects object_state_publisher.py
