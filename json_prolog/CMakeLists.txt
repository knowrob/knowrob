cmake_minimum_required(VERSION 2.8.3)
project(json_prolog)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    rosjava_build_tools rosjava_bootstrap rosjava_messages rosjava_build_tools
    rospy
    roscpp
    rosprolog
    knowrob_common
    json_prolog_msgs
)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(INCLUDE_DIRS include
  LIBRARIES json_prolog
  CATKIN_DEPENDS json_prolog_msgs
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

add_library(json_prolog src/prolog.cpp src/prolog_query_proxy.cpp src/prolog_bindings.cpp src/prolog_value.cpp)
target_link_libraries(json_prolog ${JSON_GLIB_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(json_prolog ${catkin_EXPORTED_TARGETS})

add_executable(cpl_interface_node src/cpl_interface.cpp)
target_link_libraries(cpl_interface_node ${catkin_LIBRARIES} ${SWIPL_LIBRARIES})
add_dependencies(cpl_interface_node cpl_gencpp)

add_executable(test_json_prolog examples/test_json_prolog.cpp)
target_link_libraries(test_json_prolog json_prolog ${catkin_LIBRARIES})
add_dependencies(test_json_prolog ${catkin_EXPORTED_TARGETS})

add_executable(object_positions examples/object_positions.cpp)
target_link_libraries(object_positions json_prolog ${catkin_LIBRARIES})
add_dependencies(object_positions ${catkin_EXPORTED_TARGETS})


install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
  
install(FILES json_prolog/build/classpath.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/build )
