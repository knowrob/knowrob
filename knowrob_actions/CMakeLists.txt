cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# external dependencies in other packages
rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

rosbuild_find_ros_package(rosjava_jni)
add_classpath(${rosjava_jni_PACKAGE_PATH}/bin)

rosbuild_find_ros_package(knowrob_common)
add_classpath(${knowrob_common_PACKAGE_PATH}/bin)
add_classpath(${knowrob_common_PACKAGE_PATH}/lib/commons-lang-2.6.jar)

add_classpath(/usr/share/java/json-lib.jar)
add_classpath(/usr/share/java/commons-collections3.jar)
add_classpath(/usr/share/java/ezmorph.jar)
add_classpath(/usr/share/java/commons-beanutils.jar)
add_classpath(/usr/share/java/commons-lang.jar)
add_classpath(/usr/share/java/commons-logging.jar)

add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_runtime_classpath(/usr/share/java/json-lib.jar)
add_runtime_classpath(/usr/share/java/commons-collections3.jar)
add_runtime_classpath(/usr/share/java/ezmorph.jar)
add_runtime_classpath(/usr/share/java/commons-beanutils.jar)
add_runtime_classpath(/usr/share/java/commons-lang.jar)
add_runtime_classpath(/usr/share/java/commons-logging.jar)


add_jar_dir(lib)
add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rosbuild_include(rosprolog utils)
rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
