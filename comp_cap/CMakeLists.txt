cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_include(rosprolog utils)

#######################################
# reset paths in OWL files
rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

rosbuild_find_ros_package(mod_srdl)
set( SRDL_PATH_PREFIX ${mod_srdl_PACKAGE_PATH})
fix_ros_destination_path(SRDL_PATH_PREFIX)

rosbuild_find_ros_package(comp_cap)
set( LOCAL_PACKAGE_PATH ${comp_cap_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#rosbuild_add_library(servicecpp servicecpp.cpp)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


#######################################

# external dependencies in other packages
rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

rosbuild_find_ros_package(rosjava)
add_classpath(${rosjava_PACKAGE_PATH}/bin)

rosbuild_find_ros_package(comp_cap)
add_classpath(${rosjava_PACKAGE_PATH}/lib)
add_runtime_classpath(${rosjava_PACKAGE_PATH}/lib)

rospack_add_java_executable(XmlRpcException org.apache.xmlrpc.XmlRpcException)
rospack_add_java_executable(XmlRpcClient org.apache.xmlrpc.client.XmlRpcClient)
rospack_add_java_executable(XmlRpcClientConfigImpl org.apache.xmlrpc.client.XmlRpcClientConfigImpl)
rospack_add_java_executable(CapabilityBase edu.tum.cs.ias.knowrob.comp_cap.CapabilityBase)
rospack_add_java_executable(CapROSClient edu.tum.cs.ias.knowrob.comp_cap.CapROSClient)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

add_ld_preload(${PROJECT_SOURCE_DIR}/lib)

execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/lib/org 
							   ${PROJECT_SOURCE_DIR}/bin/org)
#execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/lib/libservicecpp.so 					                                                           ${PROJECT_SOURCE_DIR}/bin/libservicecpp.so)