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

### Unit testing

To run unit tests for some modules in a rosprolog package, use:

    rosrun rosprolog rosprolog-test <pkgname> <modulenames>

