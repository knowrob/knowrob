# This file contains useful helper functions to setup the environment
# correctly for nodes using rosprolog, i.e. for setting classpath and
# ld_library_path correctly.


# search for files 'classpath.txt' in all packages the package $1 depends on
# and concatenate the classpath variables
function get_pkg_classpath() {
    ROS_CP=""

    # find classpath for package itself
    PKG_PATH=`rospack find $1`
    for cp in `find $PKG_PATH -name classpath.txt`; do
      ROS_CP=`cat ${cp}`:$ROS_CP
    done

    # find classpath for all dependencies
    for dep in `rospack depends $1`; do
      PKG_PATH=`rospack find $dep`
      for cp in `find $PKG_PATH -name classpath.txt`; do
        ROS_CP=`cat ${cp}`:$ROS_CP
      done
    done
    export $2=$ROS_CP
}
