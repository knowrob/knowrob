cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(jpl)
rosbuild_find_ros_package(mod_vis)
rosbuild_find_ros_package(json_prolog)
rosbuild_find_ros_package(srldb)


set( MOD_DIALOG_PATH ${PROJECT_SOURCE_DIR} )

file(GLOB inputs src/edu/tum/cs/ias/knowrob/mod_dialog/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

# copy config file to binary tree
configure_file(${PROJECT_SOURCE_DIR}/src/edu/tum/cs/ias/knowrob/mod_dialog/jsgf.config.xml ${PROJECT_SOURCE_DIR}/bin/edu/tum/cs/ias/knowrob/mod_dialog/jsgf.config.xml COPYONLY)

add_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)
add_classpath(${json_prolog_PACKAGE_PATH}/bin)
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_classpath(/usr/share/java/json-lib.jar)
add_classpath(/usr/share/java/commons-collections3.jar)
add_classpath(/usr/share/java/ezmorph.jar)
add_classpath(/usr/share/java/commons-beanutils.jar)
add_classpath(/usr/share/java/commons-lang.jar)
add_classpath(/usr/share/java/commons-logging.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/WSJ_8gau_13dCep_16k_40mel_130Hz_6800Hz.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/WSJ_8gau_13dCep_8kHz_31mel_200Hz_3500Hz.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/js.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jsapi.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/sphinx4.jar)
add_jar_dir(${srldb_PACKAGE_PATH}/lib)

add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/controlP5.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/core.jar)
add_runtime_classpath(${mod_vis_PACKAGE_PATH}/lib/vecmath.jar)
add_runtime_classpath(${json_prolog_PACKAGE_PATH}/bin)
add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_runtime_classpath(/usr/share/java/json-lib.jar)
add_runtime_classpath(/usr/share/java/commons-collections3.jar)
add_runtime_classpath(/usr/share/java/ezmorph.jar)
add_runtime_classpath(/usr/share/java/commons-beanutils.jar)
add_runtime_classpath(/usr/share/java/commons-lang.jar)
add_runtime_classpath(/usr/share/java/commons-logging.jar)

add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/WSJ_8gau_13dCep_16k_40mel_130Hz_6800Hz.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/WSJ_8gau_13dCep_8kHz_31mel_200Hz_3500Hz.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/js.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jsapi.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/sphinx4.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(DialogModule edu.tum.cs.ias.knowrob.mod_dialog.DialogModule)
rospack_add_java_executable(SpeechRecognitionNode edu.tum.cs.ias.knowrob.mod_dialog.SpeechRecognitionNode)
