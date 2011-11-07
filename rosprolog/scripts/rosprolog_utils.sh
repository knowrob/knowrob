# This file contains useful helper functions to setup the environment
# correctly for nodes using rosprolog, i.e. for setting classpath and
# ld_library_path correctly.

function get_pkg_classpath() {
    ROS_EXPORT_CP=$(rospack export --lang=java --attrib=classpath $1)
    export $2="$(echo $ROS_EXPORT_CP | sed 's/ /:/g')"
}

function get_pkg_ld_lib_path() {
    ROS_EXPORT_LD=$(rospack export --lang=java --attrib=ld_lib_path $1)
    export $2=$(echo $ROS_EXPORT_LD | sed 's/ /:/g'):$(rospack find rosjava_jni)/bin
}
