cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(knowrob_common)

# set local package path of knowrob_cad_models in owl and prolog files
rosbuild_find_ros_package(knowrob_cad_parser)
set( LOCAL_PACKAGE_PATH ${knowrob_cad_parser_PACKAGE_PATH} )


add_classpath(${PROJECT_SOURCE_DIR}/lib/processing/core.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/processing/PeasyCam.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/dae4j.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.16.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/processing/core.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/processing/PeasyCam.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/dae4j.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.16.jar)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/commons-net-3.0.1.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/commons-net-3.0.1.jar)

rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/jpl/jpl.jar)
add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
