rosprolog
===

rosprolog is a bidirectional interface between
[SWI Prolog](http://www.swi-prolog.org/) and ROS,
and a wrapper around [roscpp](http://wiki.ros.org/roscpp).
It provides predicates that enable Prolog programmers
to quickly interface with ROS subsystems, and to maintain dependencies
to other ROS packages.
It further implements a ROS node that enables other ROS components
to query a Prolog engine.

### Packages

rosprolog packages are normal ROS packages that, in addition,
contain some special files and folders.
This common structure allows to automatically load the package and all its dependencies. 

    your_package
    |- package.xml
    |- CMakeLists.txt
    |- owl
       |- your_ontology.owl
    |- prolog
       |- init.pl
       |- your_package
          |- your_module.pl
          |- your_module.plt

### rosprolog console

To run rosprolog, use:

    rosrun rosprolog rosprolog

 
To start a prolog package within rosprolog (incl. calling its init.pl file and all init.pl of referenced packages), use:

    rosrun rosprolog rosprolog <pkgname>

### rosprolog node

The interactive Prolog shell is good for exploring KnowRob, visualizing knowledge,
developing new functions and debugging Prolog code.

However, if you would like to use KnowRob in your robot's control program,
you need a way to send queries from your program.
This functionality is provided by the rosprolog package.
It provides a service that exposes a Prolog shell via ROS.
You can run the rosprolog service using a launch file such as the following
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
  
  <node name="rosprolog" pkg="rosprolog" type="rosprolog_node" cwd="node" output="screen" />
</launch>
```

The rosprolog_node reads some optional ROS parameters for the initial package
to be loaded (that you also give as argument when starting rosprolog)
and a command to be executed at startup.

### rosprolog node console

You can get an interactive console for a running rosprolog node with

    rosrun rosprolog rosprolog_commandline.py

### Unit testing

Unit testing in Prolog is done via the library *plunit*.
*rosprolog* defines a CMake macro that allows adding *plunit*
testcases to the *rostest* suite.

    catkin_add_plunit(knowrob/comp_spatial)

Test files should by placed next to the source file that is tested
but with a *plt* file extension.
The testcase must be named exactly like the path to the module,
i.e., in above case "knowrob/comp_spatial".

Another way to run the tests is by invoking the *rosprolog-test* script:

    rosrun rosprolog rosprolog-test comp_spatial:"knowrob/comp_spatial"
