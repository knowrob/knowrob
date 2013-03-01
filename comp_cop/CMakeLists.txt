cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

rosbuild_include(rosprolog utils)

#######################################
# reset paths in OWL files
rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

rosbuild_find_ros_package(comp_cop)
set( LOCAL_PACKAGE_PATH ${comp_cop_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


#######################################
# Classpath
add_classpath(${PROJECT_SOURCE_DIR}/lib/apache-mime4j-0.6.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xmlrpc-1.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/google-api-translate-java-0.92.jar)

add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/apache-mime4j-0.6.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xmlrpc-1.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/google-api-translate-java-0.92.jar)

# external dependencies in other packages
rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

rosbuild_find_ros_package(rosjava_jni)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)

rosbuild_find_ros_package(knowrob_common)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/commons-logging-1.1.1.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/commons-codec-1.3.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpclient-4.0.1.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpcore-4.0.1.jar)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpmime-4.0.1.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/commons-logging-1.1.1.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/commons-codec-1.3.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpclient-4.0.1.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpcore-4.0.1.jar)
add_runtime_classpath(${knowrob_common_PACKAGE_PATH}/lib/httpcomponents-client/httpmime-4.0.1.jar)

# generate launch script
rospack_add_java_executable(BarcodeWebLookup edu.tum.cs.ias.knowrob.BarcodeWebLookup)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)
