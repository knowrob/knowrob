cmake_minimum_required(VERSION 2.8.3)
project(json_prolog)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp roslib rospack 
    rosprolog
    knowrob_common
    json_prolog_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES json_prolog_client
  CATKIN_DEPENDS json_prolog_msgs roscpp roslib rospack
)

# find SWI Prolog libraries
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

# export Python libraries
catkin_python_setup()

# C++ client library
include_directories(include ${catkin_INCLUDE_DIRS})

pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

add_executable(json_prolog_node
	src/JSONPrologNode.cpp
	src/JSONPrologEngine.cpp
	src/JSONPrologPool.cpp
)
target_link_libraries(json_prolog_node ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(json_prolog_node ${catkin_EXPORTED_TARGETS})

add_library(json_prolog_client
	src/prolog.cpp
	src/prolog_query_proxy.cpp
	src/prolog_bindings.cpp
	src/prolog_value.cpp)
target_link_libraries(json_prolog_client ${JSON_GLIB_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(json_prolog_client ${catkin_EXPORTED_TARGETS})

add_executable(test_json_prolog examples/test_json_prolog.cpp)
target_link_libraries(test_json_prolog json_prolog_client ${catkin_LIBRARIES})
add_dependencies(test_json_prolog ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
