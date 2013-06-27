cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

# set ias kb package path
rosbuild_find_ros_package(ias_knowledge_base)
set( IAS_KB_PACKAGE_PATH ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(IAS_KB_PACKAGE_PATH)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

# set local package path of mod_srdl in owl and prolog files
rosbuild_find_ros_package(mod_srdl)
set( SRDL_PACKAGE_PATH ${mod_srdl_PACKAGE_PATH} )
fix_ros_destination_path(SRDL_PACKAGE_PATH)

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

# Build urdf converter (converter from urdf to srdl)
add_jar_dir(lib)
add_java_source_dir( ${PROJECT_SOURCE_DIR}/src )
rospack_add_java_executable( urdf_to_owl org.roboearth.wp5.io.URDF2SRDL)
