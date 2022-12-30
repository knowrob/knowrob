cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
set (CMAKE_CXX_STANDARD 17)

project(knowrob)

set(SOMA_VERSION "current")

# include additional CMake macros
include("cmake/ontologies.cmake")

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(spdlog REQUIRED)

enable_testing()
find_package(GTest REQUIRED)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

# install additional Prolog code
# FIXME: is added by rosprolog
#install_prolog_pack(list_util
#	URL "https://github.com/mndrix/list_util/archive/v0.13.0.zip")
#install_prolog_pack(delay
#	URL "https://github.com/mndrix/delay/archive/v0.3.3.zip")

# install ontologies hosted on http servers.
# copy them into the workspace such the we can load them
# from a local source at runtime.
# TODO: there is a similar macros called FetchContent_* shipped with CMake.
#       Have a look if install_ontology can use it or be replaced with it.
install_ontology(
	URL		  "http://www.ease-crc.org/ont/DUL.owl"
	VERSION   "3.34")
install_ontology(
	URL       "https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA.owl"
	VERSION   "${SOMA_VERSION}")
install_ontology(
	URL       "https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA-HOME.owl"
	VERSION   "${SOMA_VERSION}")

set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
include_directories(include ${SWIPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

##############
########## ROS1 block
##############
# NOTE: seems catkin_package must be called *before* any library is build.

find_package(catkin QUIET COMPONENTS
		rosprolog roscpp roslib tests/urdf
		geometry_msgs message_generation)

if (catkin_FOUND)
	message(STATUS "Building with ROS1 support.")
	
	include_directories(${catkin_INCLUDE_DIRS})
	
	catkin_python_setup()

	add_message_files(FILES EventToken.msg)
	generate_messages(DEPENDENCIES geometry_msgs)
	
	catkin_package(
		CATKIN_DEPENDS rosprolog roscpp roslib
		CFG_EXTRAS ontologies.cmake
	)
else()
	message(STATUS "Building without ROS1 support.")
endif()

##############
#### MONGO DB
##############

pkg_check_modules(MONGOC REQUIRED libmongoc-1.0)
# find_package(libmongoc-1.0 1.7 REQUIRED)
include_directories(${MONGOC_INCLUDE_DIRS})
link_directories(${MONGOC_LIBRARY_DIRS})

add_library(mongo_kb SHARED
		src/mongodb/mongo_kb.cpp
		src/mongodb/bson_pl.cpp
		src/mongodb/MongoException.cpp
		src/mongodb/MongoInterface.cpp
		src/mongodb/MongoDatabase.cpp
		src/mongodb/MongoCollection.cpp
		src/mongodb/MongoCursor.cpp
		src/mongodb/MongoWatch.cpp)
target_link_libraries(mongo_kb
		${SWIPL_LIBRARIES}
		${MONGOC_LIBRARIES})

##############
#### QUERY ANSWERING
##############

add_library(hyres SHARED
		src/ThreadPool.cpp
		src/logging.cpp
		src/terms.cpp
		src/queries.cpp
		src/Blackboard.cpp
		src/ReasonerManager.cpp
		src/prolog/PrologQuery.cpp
		src/prolog/PrologReasoner.cpp
		src/prolog/logging.cpp
		src/prolog/algebra.cpp
		src/mongolog/MongologReasoner.cpp
        src/mongolog/MongologTests.cpp)
target_link_libraries(hyres
		${SWIPL_LIBRARIES}
		spdlog::spdlog
		GTest::gtest)

add_executable(test_qa tests/test_qa.cpp)
target_link_libraries(test_qa hyres)

##############
########## ROS1 block
##############

if (catkin_FOUND)
	# install into ROS workspace
	add_dependencies(mongo_kb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	add_dependencies(hyres ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	
	### TF messages ###
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

	### RViz marker ###
	add_library(marker_knowrob SHARED src/ros/marker/publisher.cpp)
	target_link_libraries(marker_knowrob
		${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	add_dependencies(marker_knowrob
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})
	
	### URDf ###
	add_library(urdf_parser SHARED src/ros/urdf/parser.cpp)
	target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	add_dependencies(urdf_parser
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})
endif()

##############
########## Unit Testing
##############

# add an executable target for GTest.
# but the testing code is actually in the *hyres* library
# where gtest won't find them without using the "--no-as-needed"
# flag for the linker.
add_executable(all_gtests tests/gtests.cpp)
target_link_libraries(all_gtests
		-Wl,--whole-archive,--no-as-needed
		hyres
		-Wl,--no-whole-archive
		GTest::gtest_main)

##############
##############

# TODO: look into installation
#install(DIRECTORY owl/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
#install(DIRECTORY prolog/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
