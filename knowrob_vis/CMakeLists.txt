
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_vis)

find_package(catkin REQUIRED COMPONENTS data_vis_msgs roscpp knowrob_common)

catkin_package(CATKIN_DEPENDS roscpp knowrob_common)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

include_directories(include ${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(marker_vis SHARED src/marker_vis.cpp)
target_link_libraries(marker_vis ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(marker_vis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
