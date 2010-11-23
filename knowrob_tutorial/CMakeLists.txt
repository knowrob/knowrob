cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


#######################################
# reset local paths in Prolog files

rosbuild_find_ros_package(knowrob_tutorial)
set( LOCAL_PACKAGE_PATH ${knowrob_tutorial_PACKAGE_PATH} )

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )




set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)



# external dependencies in other packages
rosbuild_find_ros_package(rosjava)
add_classpath(${rosjava_PACKAGE_PATH}/bin)

rosbuild_find_ros_package(rosjava_deps)
add_classpath(${rosjava_deps_PACKAGE_PATH}/rosjava_msgs.jar)

rospack_add_java_executable(ROSClient edu.tum.cs.ias.knowrob.tutorial.ROSClient)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
