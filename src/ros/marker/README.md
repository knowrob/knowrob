\page ros_marker Marker visualization

The ROS-based tool RViz defines a commonly used message format
for 3D scene visualization supported by KnowRob.
Knowledge about objects is used to generate marker visualization
messages and tools such as RViz will then be able to visiualize
a 3D scene accordingly.
In particular, knowledge about certain qualities of objects is exploited such
as the color or shape of the object.

For example, this code can be used to generate visualization markers for the PR2 robot in initial pose
using "map" as root frame of the world:

    load_owl('package://knowrob/owl/robots/PR2.owl'),
    urdf_load('http://knowrob.org/kb/PR2.owl#PR2_0', 'package://knowrob/urdf/pr2_for_unit_tests.urdf', [load_rdf]),
    urdf_set_pose_to_origin('http://knowrob.org/kb/PR2.owl#PR2_0',map),
    marker:republish.
