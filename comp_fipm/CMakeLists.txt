cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

# replace paths

rosbuild_find_ros_package(comp_fipm)
set( PL_PATH_PREFIX "${comp_fipm_PACKAGE_PATH}/prolog/" )
fix_ros_destination_path(PL_PATH_PREFIX)

file(GLOB inputs prolog/*.in)

foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


# add exported jar files to the classpath
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/bnj_fipm.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/fipmbase.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/kipm.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/kipmmodels.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/srldb.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/jgraph.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/mysql-connector-java-3.0.11-stable-bin.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/log4j.jar)
add_classpath(${ias_comp_ros_PACKAGE_PATH}/lib/mckoi_fipm.jar)



