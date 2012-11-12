cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_include(rosprolog utils)

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(ias_knowledge_base_PACKAGE_PATH)

rosbuild_find_ros_package(ias_semantic_map)
set( OWL_PATH_PREFIX_SEM ${ias_semantic_map_PACKAGE_PATH} )
fix_ros_destination_path(ias_semantic_map_PACKAGE_PATH)

rosbuild_find_ros_package(semweb)
set( OWL_PATH_PREFIX_SEMWEB ${semweb_PACKAGE_PATH} )
fix_ros_destination_path(semweb_PACKAGE_PATH)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


# replace path to local OWL files
rosbuild_find_ros_package(comp_eye_traj)
set( LOCAL_PACKAGE_PATH ${comp_eye_traj_PACKAGE_PATH} )
fix_ros_destination_path(comp_eye_traj_PACKAGE_PATH)

file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


