
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_objects)

find_package(catkin REQUIRED COMPONENTS knowrob_common geometry_msgs message_generation rosprolog roscpp roslib)

catkin_python_setup()

add_message_files(FILES ObjectState.msg)
add_service_files(FILES UpdateObjectState.srv DirtyObject.srv)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS knowrob_common rosprolog roscpp roslib)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
message(STATUS "SWIPL found.")

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
message(STATUS "INCLUDE ${catkin_INCLUDE_DIRS}")
message(STATUS "LIBS ${catkin_LIBRARIES}")

include_directories(include ${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(knowrob_objects SHARED src/knowrob_objects.cpp)
target_link_libraries(knowrob_objects ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(knowrob_objects 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
