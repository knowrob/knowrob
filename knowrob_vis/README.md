knowrob_vis
===

Utilities for the visualization of knowledge pieces.
This package follows a client-server scheme:
The server side (knowrob_vis) generates only messages describing what
should be visualized, and clients can subscribe to these messages
and update their canvas accordingly.

The visualization module marker_vis.pl is based on
RViz [marker messages](http://www.ros.org/wiki/rviz/DisplayTypes/Marker).
These describe some 3D visual information, such as some
basic 3D shapes, but also complex meshes.
RViz can be used as visualization client for marker messages generated in KnowRob.
However, the marker message definition is extended by KnowRob with some new marker types,
these won't show up in RViz.

The second visualization interface implemented in this package is the data visualization
module data_vis.pl.
[Data visualization messages](https://raw.githubusercontent.com/code-iai/iai_common_msgs/master/data_vis_msgs/msg/DataVis.msg) contain
a set of data records together with some visualization hints such
as diagram type, axis labels, font size, etc.
How in particular the data is displayed is up to the data visualization
client.
At the moment only [openEASE](http://www.open-ease.org/) implements such a client.

knowrob_vis comes with a minimal implementation of a visualization
client based on the
[ros3djs library](https://github.com/RobotWebTools/ros3djs)
that runs in browsers with webgl support.
The server can be started with visualisation_server/0, and then be accessed
on [localhost](http://localhost:1111).

### Marker visualization

First, setup the rosbridge communication interface and initialize the `knowrob_vis` package.

    roslaunch rosbridge_server rosbridge_websocket.launch
    rosrun rosprolog rosprolog knowrob_vis

Start the local visualization server, and connect to it with your browser (port 1111).

    ?- visualisation_server.

Create some visualization markers, and publish them to be displayed by visualization clients:

    ?- marker(cube(a),X),
       marker_color(X,[1,0,0]),
       marker_scale(X,2),
       marker_publish.

#### Marker Types

The set of supported primitive marker types is
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

#### 3D object meshes

KnowRob supports the visualization of objects using 3D surface meshes in the STL or Collada file formats.
These meshes can either be attached to an object instance or to an object class,
in which case they are used for visualization of all of its instances.
The following OWL snippet shows how an instance
can be linked to its CAD model using the [pathToCadModel](http://knowrob.org/kb/knowrob.owl#pathToCadModel) property:

     <owl:NamedIndividual rdf:about="&map;milk1">
         <rdf:type rdf:resource="&knowrob;CowsMilk-Product"/>
         <knowrob:pathToCadModel rdf:datatype="&xsd;string"
                             package://knowrob_tutorial/cad/milk.kmz</knowrob:pathToCadModel>
     </owl:NamedIndividual>

### Data visualization

Data visualization clients may use the Google Visualization API (Licensed under Creative Commons 3).
More information can be found on https://developers.google.com/chart/
