
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

#######################################
# Classpath

add_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi-bin.jar)


# external dependencies in other packages
# rosbuild_find_ros_package(rosjava)
# add_classpath(${rosjava_PACKAGE_PATH}/bin)


# generate launch script
rospack_add_java_executable(SemanticMapToOWL edu.tum.cs.ias.knowrob.SemanticMapToOWL)
rospack_add_java_executable(SemanticMapToOWLTestClient edu.tum.cs.ias.knowrob.SemanticMapToOWLTestClient)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
