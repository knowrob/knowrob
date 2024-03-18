\page ros_vis Marker Visualization

Utilities for the visualization of knowledge pieces.
This package follows a client-server scheme:
The server side (knowrob_vis) generates only messages describing what
should be visualized, and clients can subscribe to these messages
and update their canvas accordingly.

One of the supported clients is [RViz](http://wiki.ros.org/rviz)
which uses so called *Marker* messages to tell RViz what should
be displayed.
These describe some 3D visual information, such as some
basic 3D shapes, but also complex meshes.

The second visualization interface implemented in this package is the data visualization
module data_vis.pl.
[Data visualization messages](https://raw.githubusercontent.com/code-iai/iai_common_msgs/master/data_vis_msgs/msg/DataVis.msg) contain
a set of data records together with some visualization hints such
as diagram type, axis labels, font size, etc.
How in particular the data is displayed is up to the data visualization
client.
At the moment only [openEASE](http://www.open-ease.org/) implements such a client.
