
cmake_minimum_required(VERSION 2.8.3)
project(rosprolog)

find_package(catkin REQUIRED COMPONENTS roscpp tf roslib rospack)

catkin_package(CATKIN_DEPENDS roscpp roslib rospack)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
message(STATUS "SWIPL found.")

include_directories(include ${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(rosprolog SHARED src/rosprolog.cpp)
target_link_libraries(rosprolog ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
