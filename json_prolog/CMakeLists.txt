cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)

rosbuild_find_ros_package(jpl)

add_jni_path(${jpl_PACKAGE_PATH}/prolog)
add_ld_preload(/usr/lib/libswipl.so)
add_ld_preload(/usr/lib/libpl.so)
  
add_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_classpath(/usr/share/java/json-lib.jar)
add_classpath(/usr/share/java/commons-collections3.jar)
add_classpath(/usr/share/java/ezmorph.jar)
add_classpath(/usr/share/java/commons-beanutils.jar)
add_classpath(/usr/share/java/commons-lang.jar)
add_classpath(/usr/share/java/commons-logging.jar)

add_runtime_classpath(${jpl_PACKAGE_PATH}/prolog/jpl.jar)
add_runtime_classpath(/usr/share/java/json-lib.jar)
add_runtime_classpath(/usr/share/java/commons-collections3.jar)
add_runtime_classpath(/usr/share/java/ezmorph.jar)
add_runtime_classpath(/usr/share/java/commons-beanutils.jar)
add_runtime_classpath(/usr/share/java/commons-lang.jar)
add_runtime_classpath(/usr/share/java/commons-logging.jar)

add_java_source_dir( ${PROJECT_SOURCE_DIR}/src/java )
rospack_add_java_executable( json_prolog edu.tum.cs.ias.knowrob.json_prolog.JSONPrologNode )

# C++ client library

find_package(PkgConfig)
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

rosbuild_add_library(json_prolog src/prolog.cpp src/prolog_query_proxy.cpp src/prolog_bindings.cpp src/prolog_value.cpp)
target_link_libraries(json_prolog ${JSON_GLIB_LIBRARIES})

rosbuild_add_executable(test_json_prolog examples/test_json_prolog.cpp)
target_link_libraries(test_json_prolog json_prolog)

rosbuild_add_executable(object_positions examples/object_positions.cpp)
target_link_libraries(object_positions json_prolog)
