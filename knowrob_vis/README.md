knowrob_vis
===

Utilities for the visualization of knowledge pieces.
This package follows a client-server scheme:
The server side (knowrob_vis) generates only messages describing what
should be visualized, and clients can subscribe to these messages
and update their canvas accordingly.

One of the visualization modules is based on
[rviz Markers](http://www.ros.org/wiki/rviz/DisplayTypes/Marker).
These describe some 3D visual information, such as some
basic 3D shapes, but also complex meshes.
RViz can be used as visualization client for marker messages generated in KnowRob.
However, the RViz message definition is extended by KnowRob with some new marker types,
these won't show in RViz.
knowrob_vis also comes with a minimal implementation of a marker visualization
client based on the
[ros3djs library](https://github.com/RobotWebTools/ros3djs)
that runs in browsers with webgl support.
The server can be started with visualisation_server/0, and then be accessed via
http://localhost:1111

The second visualization module implemented in this package is the data visualization
module data_vis.pl.
Data vis messages contain the set of records over which the data visualization
should be generated.
How in particular the data is to be displayed must be determined by the data visualization
client.
At the moment only [openEASE](http://www.open-ease.org/) implements such a client.

### Marker visualization

First, setup the rosbridge communication interface and initialize the `knowrob_vis` package.

    roslaunch rosbridge_server rosbridge_websocket.launch
    rosrun rosprolog rosprolog knowrob_vis


Markers must be explicitly published by the user with following query ([openEASE](http://www.open-ease.org/) does this automatically):

    marker_publish.


#### Marker Types

The set of supported primitive marker types includes
arrow, cube, sphere, cylinder, line_strip,
line_list, cube_list, sphere_list, points, text_view_facing,
mesh_resource and triangle_list.

In addition, KnowRob supports a set of complex marker types.
  * **cylinder_tf(Frame0,Frame1)**: A cylinder between 2 links
  * **link(Frame)**: A marker that takes the pose of a link
  * **trajectory(Frame)**: A trajectory of a link
  * **pointer(Frame0,Frame1)**: Arrow that points from Frame0 in the direction of Frame1
  * **object(Name)**: A marker that represents an OWL individual (including children objects)
  * **agent(Name)**: A marker that represents an OWL agent individual (including all links)
  * **stickman(Name)**: A special human skeleton visualization, kinematic structure is spanned by the OWL individual indentified by Name
