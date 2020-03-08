
cmake_minimum_required(VERSION 2.8.3)
project(knowrob)

find_package(catkin REQUIRED COMPONENTS rosprolog rosowl geometry_msgs message_generation roscpp roslib)

catkin_python_setup()

add_message_files(FILES
    ObjectState.msg
    ObjectStateArray.msg
    EventToken.msg)
generate_messages(DEPENDENCIES geometry_msgs)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

install_prolog_pack(list_util
  URL "https://github.com/mndrix/list_util/archive/v0.13.0.zip")
install_prolog_pack(delay
  URL "https://github.com/mndrix/delay/archive/v0.3.3.zip")

catkin_package(CATKIN_DEPENDS rosprolog roscpp roslib)
catkin_add_plunit(
  knowrob/computable
  knowrob/knowrob
  knowrob/temporal
  knowrob/wup_similarity)

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
include_directories(include ${SWIPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(object_state_publisher SHARED src/object_state_publisher.cpp)
target_link_libraries(object_state_publisher ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(object_state_publisher 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

add_library(pl_eigen SHARED src/pl_eigen.cpp)
target_link_libraries(pl_eigen ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(pl_eigen
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
