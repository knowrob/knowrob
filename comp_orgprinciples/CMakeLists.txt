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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



#######################################
# reset paths in OWL files
rosbuild_find_ros_package(ias_knowledge_base)
set( IAS_KNOWLEDGE_BASE_PACKAGE_PATH ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(IAS_KNOWLEDGE_BASE_PACKAGE_PATH)

rosbuild_find_ros_package(comp_germandeli)
set( GERMANDELI_PACKAGE_PATH ${comp_germandeli_PACKAGE_PATH} )
fix_ros_destination_path(GERMANDELI_PACKAGE_PATH)

rosbuild_find_ros_package(ias_semantic_map)
set( IAS_SEMANTIC_MAP_PACKAGE_PATH ${ias_semantic_map_PACKAGE_PATH} )
fix_ros_destination_path(IAS_SEMANTIC_MAP_PACKAGE_PATH)

rosbuild_find_ros_package(comp_orgprinciples)
set( LOCAL_PACKAGE_PATH ${comp_orgprinciples_PACKAGE_PATH} )
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
#######################################
