cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


# external dependencies in other packages
rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

rosbuild_find_ros_package(rosjava)
add_classpath(${rosjava_PACKAGE_PATH}/bin)

# generate launch script
rospack_add_java_executable(BarcodeWebLookup edu.tum.cs.ias.knowrob.BarcodeWebLookup)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)