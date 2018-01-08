rosprolog
===

rosprolog is a wrapper around swi-prolog and roscpp that uses an additional user-init file.
Some new predicates are added in this package:
 * register_ros_package/1 Loads the initialization file of a KnowRob ROS package.
 * use_ros_module/2 Load a prolog module from a KnowRob ROS package.
 * ros_package_path/2 Get the absolute path of a package.
 * ros_info/1, ros_warn/1, ros_error/1
   Send logging messages to ROS master.
 * tf_lookup_transform/3 TFTransformListener interface.

To run rosprolog, use:

    rosrun rosprolog rosprolog

 
To start a prolog package within rosprolog (incl. calling its init.pl file and all init.pl of referenced packages), use:

    rosrun rosprolog rosprolog <pkgname>


For a introduction on KnowRob packages, please consult [knowrob.org](http://knowrob.org/doc/create_your_own_knowrob_package).

