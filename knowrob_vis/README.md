Based on [[rviz Markers][http://www.ros.org/wiki/rviz/DisplayTypes/Marker]], this visualization module can display 3D information in a browser-based canvas that has been built using the [[ros3djs library][https://github.com/RobotWebTools/ros3djs]].

---+++ Usage

First, setup the rosbridge communication interface and initialiaze the `knowrob_vis` package.
==
roslaunch rosbridge_server rosbridge_websocket.launch
rosrun rosprolog  rosprolog knowrob_vis
==
  
Markers must be explicitly published by the user with following query ([[openEASE][http://www.open-ease.org/]] does this automatically).
==
marker_publish.
==

---+++ Marker Types

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
  