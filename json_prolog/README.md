json_prolog
===

The interactive Prolog shell that [rosprolog](http://ros.org/wiki/rosprolog|rosprolog)
provides is good for exploring KnowRob, visualizing knowledge,
developing new functions and debugging Prolog code.

However, if you would like to use KnowRob in your robot's control program,
you need a way to send queries from your program.
This functionality is provided by the json_prolog package.
It provides a service that exposes a Prolog shell via ROS.
You can run the json_prolog service using a launch file such as the following
```
<launch>
  <arg name="initial_package" default="knowrob_common" />
  <arg name="initial_goal" default="knowrob_common" />
  <arg name="num_pl_threads" default="2" />
  <arg name="num_ros_threads" default="4" />
  
  <param name="initial_package" type="string" value="$(arg initial_package)" />
  <param name="initial_goal" type="string" value="$(arg initial_goal)" />
  
  <param name="num_pl_threads" type="int" value="$(arg num_pl_threads)" />
  <param name="num_ros_threads" type="int" value="$(arg num_ros_threads)" />
  
  <node name="json_prolog" pkg="json_prolog" type="json_prolog_node" cwd="node" output="screen" />
</launch>
```

The json_prolog_node reads some optional ROS parameters for the initial package
to be loaded (that you also give as argument when starting rosprolog)
and a command to be executed at startup.

