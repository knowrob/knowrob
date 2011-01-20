cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

rosbuild_include(rosprolog utils)

# replace path to local OWL files
rosbuild_find_ros_package(semweb)
set( LOCAL_PACKAGE_PATH ${semweb_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )
