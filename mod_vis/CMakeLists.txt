cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(mod_vis)
rosbuild_find_ros_package(knowrob_cad_models)
rosbuild_find_ros_package(knowrob_cad_parser)
rosbuild_find_ros_package(knowrob_common)

add_classpath(${PROJECT_SOURCE_DIR}/lib/processing/core.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/processing/PeasyCam.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/dae4j.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.16.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/controlP5.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/processing/core.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/processing/PeasyCam.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/dae4j.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.16.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/controlP5.jar)

add_classpath(${knowrob_cad_parser_PACKAGE_PATH}/bin)
add_runtime_classpath(${knowrob_cad_parser_PACKAGE_PATH}/bin)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/snakeyaml/snakeyaml-1.11.jar)

rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/jpl/jpl.jar)
add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(communication_vis edu.tum.cs.ias.knowrob.vis.applets.CommunicationVisApplet)
