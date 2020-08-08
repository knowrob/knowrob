cmake_minimum_required(VERSION 2.8.3)
project(knowrob)

set(SOMA_VERSION "1.0.0")

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

install_prolog_pack(list_util
  URL "https://github.com/mndrix/list_util/archive/v0.13.0.zip")
install_prolog_pack(delay
  URL "https://github.com/mndrix/delay/archive/v0.3.3.zip")

catkin_package(
	CATKIN_DEPENDS rosprolog roscpp roslib
)

#
macro(install_ontology ONTOLOGY_NAME ONTOLOGY_VERSION ONTOLOGY_URL)
	set(ONTOLOGY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/owl/external")
	set(VERSION_FILE "${ONTOLOGY_DIR}/${ONTOLOGY_NAME}/${ONTOLOGY_VERSION}")
	set(ONTOLOGY_FILE "${ONTOLOGY_DIR}/${ONTOLOGY_NAME}.owl")
	if (NOT EXISTS "${VERSION_FILE}")
		message(STATUS "Installing ${ONTOLOGY_NAME} ontology.")
		file(REMOVE_RECURSE "${ONTOLOGY_DIR}/${ONTOLOGY_NAME}")
		file(REMOVE_RECURSE "${ONTOLOGY_DIR}/${ONTOLOGY_NAME}.owl")
		file(DOWNLOAD
			"${ONTOLOGY_URL}"
			"${ONTOLOGY_FILE}"
		)
		file(WRITE "${VERSION_FILE}" "")
	endif()
endmacro()
install_ontology("DUL" "3.32"
	"http://www.ontologydesignpatterns.org/ont/dul/DUL.owl")
install_ontology("IOLite" "1.8"
	"http://www.ontologydesignpatterns.org/ont/dul/IOLite.owl")
install_ontology("SOMA" "${SOMA_VERSION}"
	"https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA.owl")

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
include_directories(include ${SWIPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(kb_algebra SHARED src/utility/algebra.cpp)
target_link_libraries(kb_algebra ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(kb_algebra
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(urdf_parser SHARED src/model/URDF/parser.cpp)
target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(urdf_parser
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
	src/db/mongo/MongoCursor.cpp)
target_link_libraries(mongo_kb
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES})

##############
#### Pugins
##############

add_library(tf_plugin SHARED
	src/comm/ros/tf/tf_plugin.cpp
	src/comm/ros/tf/memory.cpp
	src/comm/ros/tf/logger.cpp
	src/comm/ros/tf/publisher.cpp)
target_link_libraries(tf_plugin
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES}
	mongo_kb)
add_dependencies(tf_plugin
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

add_library(marker_plugin SHARED
	src/comm/ros/marker/publisher.cpp)
target_link_libraries(marker_plugin
	${SWIPL_LIBRARIES}
	${catkin_LIBRARIES})
add_dependencies(marker_plugin
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS})

##############
##############

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
