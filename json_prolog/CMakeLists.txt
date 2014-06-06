# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.7)
project(json_prolog)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rosprolog rospy json_prolog_msgs)


include(UseJava)

find_jar(JSON_LIB_JAR NAMES json-lib)
# add_classpath(${JSON_LIB_JAR}) # TODO
# add_runtime_classpath(${JSON_LIB_JAR})

find_jar(COMMONS_BEANUTILS_JAR NAMES commons-beanutils)
# add_classpath(${COMMONS_BEANUTILS_JAR}) # TODO
# add_runtime_classpath(${COMMONS_BEANUTILS_JAR})

find_jar(COMMONS_COLLECTIONS3_JAR commons-collections3)
# add_classpath(${COMMONS_COLLECTIONS3_JAR}) # TODO
# add_runtime_classpath(${COMMONS_COLLECTIONS3_JAR})

find_jar(COMMONS_LOGGING_JAR commons-logging)
# add_classpath(${COMMONS_LOGGING_JAR}) # TODO
# add_runtime_classpath(${COMMONS_LOGGING_JAR})

find_jar(EZMORPH_JAR ezmorph)
# add_classpath(${EZMORPH_JAR}) # TODO
# add_runtime_classpath(${EZMORPH_JAR})

find_jar(COMMONS_LANG_JAR commons-lang)
# add_classpath(${COMMONS_LANG_JAR}) # TODO
# add_runtime_classpath(${COMMONS_LANG_JAR})

# C++ client library

# TODO
# find_package(PkgConfig)
# pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
# add_definitions(${JSON_GLIB_CFLAGS})
# link_directories(${JSON_GLIB_LIBRARIY_DIRS})
# 
# add_library(json_prolog src/prolog.cpp src/prolog_query_proxy.cpp src/prolog_bindings.cpp src/prolog_value.cpp)
# target_link_libraries(json_prolog ${JSON_GLIB_LIBRARIES})
# 
# add_executable(test_json_prolog examples/test_json_prolog.cpp)
# target_link_libraries(test_json_prolog json_prolog)
# 
# add_executable(object_positions examples/object_positions.cpp)
# target_link_libraries(object_positions json_prolog)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS rosprolog rospy json_prolog_msgs libjson-java libjson-glib
    CATKIN_DEPENDS # TODO
)