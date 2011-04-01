cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)


# replace image path in communication vis applet
rosbuild_find_ros_package(mod_vis)
set( LOCAL_PACKAGE_PATH ${mod_vis_PACKAGE_PATH} )

file(GLOB inputs src/de/tum/in/fipm/kipm/gui/visualisation/applets/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )



add_classpath(${PROJECT_SOURCE_DIR}/lib/controlP5.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/vecmath.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/pdf.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/itext.jar)

add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/controlP5.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/core.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/vecmath.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/pdf.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/itext.jar)

rosbuild_find_ros_package(jpl)
add_classpath(${jpl_PACKAGE_PATH}/jpl/jpl.jar)
add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(communication_vis de.tum.in.fipm.kipm.gui.visualisation.applets.CommunicationVisApplet)
