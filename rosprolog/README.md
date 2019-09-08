rosprolog
===

rosprolog is a [SWI Prolog](http://www.swi-prolog.org/) interface to ROS,
and a wrapper around [roscpp](http://wiki.ros.org/roscpp).
It provides predicates that enable Prolog programmers
to quickly interface with ROS subsystems, and to maintain dependencies
to other ROS packages.

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

### Usage

To run rosprolog, use:

    rosrun rosprolog rosprolog

 
To start a prolog package within rosprolog (incl. calling its init.pl file and all init.pl of referenced packages), use:

    rosrun rosprolog rosprolog <pkgname>

### rosprolog node

The interactive Prolog shell that [rosprolog](http://ros.org/wiki/rosprolog|rosprolog)
provides is good for exploring KnowRob, visualizing knowledge,
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

### Unit testing

To run unit tests for some modules in a rosprolog package, use:

    rosrun rosprolog rosprolog-test <pkgname> <modulenames>

