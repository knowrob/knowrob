cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


# set local package path of knowrob_cad_models in owl and prolog files
rosbuild_find_ros_package(knowrob_mesh_reasoning)
rosbuild_find_ros_package(knowrob_common)
rosbuild_find_ros_package(knowrob_cad_models)
rosbuild_find_ros_package(knowrob_cad_parser)
rosbuild_find_ros_package(ias_knowledge_base)

set( LOCAL_PACKAGE_PATH ${knowrob_mesh_reasoning_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

add_classpath(${PROJECT_SOURCE_DIR}/lib/commons-math3-3.0/commons-math3-3.0.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/ejml-0.21.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/commons-math3-3.0/commons-math3-3.0.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/ejml-0.21.jar)

add_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/processing/core.jar)
add_runtime_classpath(${knowrob_cad_parser_PACKAGE_PATH}/lib/processing/core.jar)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/vecmath.jar)

add_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-14.0.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/guava-14.0.jar)

add_classpath(${mod_vis_PACKAGE_PATH}/lib/processing/PeasyCam.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/processing/PeasyCam.jar)

add_classpath(${mod_vis_PACKAGE_PATH}/lib/dae4j.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/dae4j.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/log4j-1.2.16.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/log4j-1.2.16.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/commons-net-3.0.1.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/commons-net-3.0.1.jar)

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

add_classpath(${knowrob_cad_parser_PACKAGE_PATH}/bin)
add_runtime_classpath(${knowrob_cad_parser_PACKAGE_PATH}/bin)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
