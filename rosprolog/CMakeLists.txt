
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

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
      LIBRARIES rosprolog rosprolog_client
      INCLUDE_DIRS include
      CATKIN_DEPENDS json_prolog_msgs geometry_msgs roscpp roslib rospack
      CFG_EXTRAS pack.cmake plunit.cmake
)

# find SWI Prolog libraries
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

# TODO: get rid of this
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

# C++ client library
include_directories(include ${catkin_INCLUDE_DIRS})

######
add_library(rosprolog SHARED src/rosprolog.cpp)
add_dependencies(rosprolog rosprolog_generate_messages_cpp)
target_link_libraries(rosprolog ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

######
add_executable(rosprolog_node
	src/PrologNode.cpp
	src/PrologEngine.cpp
	src/PrologPool.cpp
)
target_link_libraries(rosprolog_node ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(rosprolog_node ${catkin_EXPORTED_TARGETS})

######
add_library(rosprolog_client
	src/PrologClient.cpp
	src/PrologQuery.cpp
	src/PrologBindings.cpp)
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
