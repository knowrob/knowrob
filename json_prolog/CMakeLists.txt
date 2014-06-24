cmake_minimum_required(VERSION 2.8.3)
project(json_prolog)

find_package(catkin REQUIRED rosjava_build_tools roscpp rospy json_prolog_msgs)


catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})



catkin_package(
    DEPENDS rosprolog
)


include(FindPkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

# C++ client library
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

add_library(json_prolog src/prolog.cpp src/prolog_query_proxy.cpp src/prolog_bindings.cpp src/prolog_value.cpp)
target_link_libraries(json_prolog ${JSON_GLIB_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(json_prolog ${catkin_EXPORTED_TARGETS})

add_executable(test_json_prolog examples/test_json_prolog.cpp)
target_link_libraries(test_json_prolog json_prolog ${catkin_LIBRARIES})
add_dependencies(test_json_prolog ${catkin_EXPORTED_TARGETS})

add_executable(object_positions examples/object_positions.cpp)
target_link_libraries(object_positions json_prolog ${catkin_LIBRARIES})
add_dependencies(object_positions ${catkin_EXPORTED_TARGETS})

