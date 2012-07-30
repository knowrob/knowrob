cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#######################################
# reset local paths in Prolog files

rosbuild_find_ros_package(mod_battery_state)
set( LOCAL_PACKAGE_PATH ${mod_battery_state_PACKAGE_PATH} )

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )

rosbuild_find_ros_package(ias_semantic_map)
set( IAS_MAP_PATH_PREFIX ${ias_semantic_map_PACKAGE_PATH} )


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


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


# external dependencies in other packages
rosbuild_find_ros_package(rosjava_jni)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)

rospack_add_java_executable(ROSClient edu.tum.cs.ias.knowrob.mod_battery_state.ROSClient)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
