
cmake_minimum_required(VERSION 2.8.3)
project(rosprolog)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp roslib rospack 
    tf
    json_prolog_msgs
    geometry_msgs
    message_generation
)

# export Python libraries
catkin_python_setup()

add_service_files(
  FILES
  JSONWrapper.srv
)

add_message_files(
  FILES 
  MessageJSON.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
      LIBRARIES rosprolog_kb rosprolog_client
      INCLUDE_DIRS include
      CATKIN_DEPENDS json_prolog_msgs geometry_msgs roscpp roslib rospack
      CFG_EXTRAS pack.cmake plunit.cmake
)

# find SWI Prolog libraries
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

# used to parse JSON strings in the C++ client
# TODO: better use RapidJSON
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

# C++ client library
include_directories(include ${catkin_INCLUDE_DIRS})

# TODO: make shared library with PrologEngine & PrologPool
######
add_library(rosprolog_kb
	SHARED
	src/rosprolog_kb/rosprolog_kb.cpp
	src/rosprolog_kb/ros_param.cpp
	src/rosprolog_kb/ros_terms.cpp
	src/rosprolog_kb/json_ros.cpp
	src/rosprolog_kb/tf.cpp
	src/rosprolog_node/PrologEngine.cpp
	src/rosprolog_node/PrologPool.cpp)
add_dependencies(rosprolog_kb rosprolog_generate_messages_cpp)
target_link_libraries(rosprolog_kb ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

######
add_executable(rosprolog_node
	src/rosprolog_node/PrologNode.cpp
	src/rosprolog_node/PrologEngine.cpp
	src/rosprolog_node/PrologPool.cpp
)
target_link_libraries(rosprolog_node ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rosprolog_node ${catkin_EXPORTED_TARGETS})

######
add_library(rosprolog_client
	src/rosprolog_client/PrologClient.cpp
	src/rosprolog_client/PrologQuery.cpp
	src/rosprolog_client/PrologBindings.cpp)
target_link_libraries(rosprolog_client ${JSON_GLIB_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rosprolog_client ${catkin_EXPORTED_TARGETS})

######
add_executable(test_rosprolog examples/test_rosprolog.cpp)
target_link_libraries(test_rosprolog rosprolog_client ${catkin_LIBRARIES})
add_dependencies(test_rosprolog ${catkin_EXPORTED_TARGETS})

install(
  DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(
  PROGRAMS src/rosprolog_client/rosprolog_client.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
  
FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
