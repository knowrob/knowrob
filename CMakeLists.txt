cmake_minimum_required(VERSION 2.8.3)
project(knowrob)

set(SOMA_VERSION "1.1.0")

# include additional CMake macros
include("cmake/ontologies.cmake")

find_package(catkin REQUIRED COMPONENTS
    rosprolog roscpp roslib urdf
    geometry_msgs message_generation)

catkin_python_setup()

add_message_files(FILES
    EventToken.msg)
generate_messages(DEPENDENCIES geometry_msgs)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS rosprolog roscpp roslib
	CFG_EXTRAS ontologies.cmake
)

# install additional Prolog code
install_prolog_pack(list_util
	URL "https://github.com/mndrix/list_util/archive/v0.13.0.zip")
install_prolog_pack(delay
	URL "https://github.com/mndrix/delay/archive/v0.3.3.zip")

# install ontologies hosted on http servers.
# copy them into the workspace such the we can load them
# from a local source at runtime.
install_ontology(
	URL		  "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl"
	VERSION   "3.32")
install_ontology(
	URL		  "http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl"
	VERSION   "1.8")
install_ontology(
	URL       "https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA.owl"
	VERSION   "${SOMA_VERSION}")

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
include_directories(include ${SWIPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(kb_algebra SHARED src/utility/algebra.cpp)
target_link_libraries(kb_algebra ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(kb_algebra
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

##############
#### DB
##############

pkg_check_modules(MONGOC REQUIRED libmongoc-1.0)
# find_package(libmongoc-1.0 1.7 REQUIRED)
include_directories(${MONGOC_INCLUDE_DIRS})
link_directories(${MONGOC_LIBRARY_DIRS})

add_library(mongo_kb
	SHARED
	src/db/mongo/mongo_kb.cpp
	src/db/mongo/bson_pl.cpp
	src/db/mongo/MongoException.cpp
	src/db/mongo/MongoInterface.cpp
	src/db/mongo/MongoDatabase.cpp
	src/db/mongo/MongoCollection.cpp
	src/db/mongo/MongoCursor.cpp
	src/db/mongo/MongoWatch.cpp)
target_link_libraries(mongo_kb
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES})

##############
#### Pugins
##############

add_library(tf_knowrob SHARED
	src/ros/tf/tf.cpp
	src/ros/tf/memory.cpp
		src/ros/tf/logger.cpp
	src/ros/tf/publisher.cpp
	src/ros/tf/republisher.cpp)
target_link_libraries(tf_knowrob
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES}
	mongo_kb)
add_dependencies(tf_knowrob
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_library(wrench_knowrob SHARED
	src/ros/wrench/wrench.cpp
	src/ros/wrench/memory.cpp
	src/ros/wrench/logger.cpp)
target_link_libraries(wrench_knowrob
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES}
	mongo_kb)
add_dependencies(wrench_knowrob
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_library(marker_knowrob SHARED
	src/ros/marker/publisher.cpp)
target_link_libraries(marker_knowrob
	${SWIPL_LIBRARIES}
	${catkin_LIBRARIES})
add_dependencies(marker_knowrob
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_library(urdf_parser SHARED src/ros/urdf/parser.cpp)
target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(urdf_parser
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

##############
##############

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
