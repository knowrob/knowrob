cmake_minimum_required(VERSION 2.8.7)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(UseJava)

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


find_jar(JSON_LIB_JAR NAMES json-lib)
add_classpath(${JSON_LIB_JAR})
add_runtime_classpath(${JSON_LIB_JAR})

find_jar(COMMONS_BEANUTILS_JAR NAMES commons-beanutils)
add_classpath(${COMMONS_BEANUTILS_JAR})
add_runtime_classpath(${COMMONS_BEANUTILS_JAR})

find_jar(COMMONS_COLLECTIONS3_JAR commons-collections3)
add_classpath(${COMMONS_COLLECTIONS3_JAR})
add_runtime_classpath(${COMMONS_COLLECTIONS3_JAR})

find_jar(COMMONS_LOGGING_JAR commons-logging)
add_classpath(${COMMONS_LOGGING_JAR})
add_runtime_classpath(${COMMONS_LOGGING_JAR})

find_jar(EZMORPH_JAR ezmorph)
add_classpath(${EZMORPH_JAR})
add_runtime_classpath(${EZMORPH_JAR})

find_jar(COMMONS_LANG_JAR commons-lang)
add_classpath(${COMMONS_LANG_JAR})
add_runtime_classpath(${COMMONS_LANG_JAR})


add_java_source_dir( ${PROJECT_SOURCE_DIR}/src/java )
rospack_add_java_executable( json_prolog edu.tum.cs.ias.knowrob.json_prolog.JSONPrologNode )
rospack_add_java_executable( java_test_client JSONPrologTestClient )

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
