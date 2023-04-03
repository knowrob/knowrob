cmake_minimum_required(VERSION 3.5.0)
set (CMAKE_CXX_FLAGS "-Wswitch -std=c++0x -pthread ${CMAKE_CXX_FLAGS}")
set (CMAKE_CXX_STANDARD 17)

project(knowrob)

IF( NOT WIN32 )
	INCLUDE( FindPkgConfig )
ENDIF()

set(SOMA_VERSION "current")

# include additional CMake macros
include("cmake/ontologies.cmake")

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(spdlog REQUIRED)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.50 REQUIRED COMPONENTS program_options serialization)

enable_testing()
find_package(GTest REQUIRED)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

pkg_check_modules(RAPTOR REQUIRED raptor2)
include_directories(include ${RAPTOR_INCLUDE_DIRS})

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
include_directories(include
		${SWIPL_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${Boost_INCLUDE_DIRS})

# define some constants for the compilation of source files that
# are used to load project-relative files.
# TODO: needs to be defined differently when catkin is used?
add_definitions(-DKNOWROB_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\")
add_definitions(-DKNOWROB_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")

##############
########## ROS1 block
##############
# NOTE: seems catkin_package must be called *before* any library is build.

find_package(catkin QUIET COMPONENTS
		rosprolog roscpp roslib tests/urdf
		geometry_msgs message_generation json_prolog_msgs)

if (catkin_FOUND)
	message(STATUS "Building with ROS1 support.")

	add_definitions(-DKNOWROB_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")
	
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

#add_library(mongo_kb SHARED
#		src/mongodb/mongo_kb.cpp
#		src/mongodb/bson_pl.cpp
#		src/mongodb/MongoException.cpp
#		src/mongodb/MongoInterface.cpp
#		src/mongodb/MongoDatabase.cpp
#		src/mongodb/MongoCollection.cpp
#		src/mongodb/MongoCursor.cpp
#		src/mongodb/MongoWatch.cpp)
#target_link_libraries(mongo_kb
#		${SWIPL_LIBRARIES}
#		${MONGOC_LIBRARIES})

##############
#### QUERY ANSWERING
##############

add_library(knowrob_qa SHARED
		src/knowrob.cpp
		src/ThreadPool.cpp
        src/Logger.cpp
        src/queries/DependencyGraph.cpp
		src/graphs/ReasoningGraph.cpp
		src/reasoner/Blackboard.cpp
        src/KnowledgeBase.cpp
		src/DataSource.cpp
		src/Statement.cpp
		src/graphs/PossibleWorld.cpp
		src/modalities/ConfidenceValue.cpp
		src/modalities/ConfidenceInterval.cpp
		src/modalities/Timepoint.cpp
		src/modalities/TimeInterval.cpp
		src/modalities/BeliefModality.cpp
		src/modalities/KnowledgeModality.cpp
		src/modalities/PastModality.cpp
		src/terms/Term.cpp
		src/terms/Variable.cpp
		src/terms/ListTerm.cpp
		src/terms/OptionList.cpp
		src/terms/Substitution.cpp
		src/terms/Unifier.cpp
		src/terms/Constant.cpp
		src/modalities/ModalOperator.cpp
		src/graphs/PrefixRegistry.cpp
		src/graphs/KnowledgeGraph.cpp
		src/graphs/GraphPattern.cpp
		src/formulas/Formula.cpp
		src/formulas/CompoundFormula.cpp
		src/formulas/Conjunction.cpp
		src/formulas/Disjunction.cpp
		src/formulas/Implication.cpp
        src/formulas/Negation.cpp
		src/formulas/Predicate.cpp
		src/formulas/Bottom.cpp
		src/formulas/Top.cpp
		src/formulas/ModalFormula.cpp
		src/queries/Query.cpp
		src/queries/QueryInstance.cpp
		src/queries/QueryError.cpp
        src/queries/Answer.cpp
		src/queries/AnswerStream.cpp
		src/queries/AnswerQueue.cpp
		src/queries/AnswerCombiner.cpp
		src/queries/AnswerBroadcaster.cpp
		src/queries/QueryParser.cpp
		src/queries/QueryTree.cpp
		src/queries/QueryProcessor.cpp
		src/queries/QueryPipeline.cpp
		src/reasoner/BuiltinEvaluator.cpp
		src/reasoner/DefinedPredicate.cpp
		src/reasoner/ReasonerPlugin.cpp
		src/reasoner/IReasoner.cpp
		src/reasoner/ReasonerManager.cpp
		src/reasoner/ReasonerConfiguration.cpp
        src/reasoner/HybridReasoner.cpp
		src/reasoner/ReasonerAttention.cpp
		src/reasoner/prolog/PrologQuery.cpp
		src/reasoner/prolog/PrologQueryRunner.cpp
		src/reasoner/prolog/PrologThreadPool.cpp
		src/reasoner/prolog/PrologReasoner.cpp
		src/reasoner/prolog/PrologTests.cpp
		src/reasoner/prolog/logging.cpp
		src/reasoner/prolog/algebra.cpp
		src/reasoner/prolog/blackboard.cpp
		src/reasoner/mongolog/MongologReasoner.cpp
		src/reasoner/swrl/SWRLReasoner.cpp
		src/reasoner/esg/ESGReasoner.cpp
		src/queries/BufferedAnswerStream.cpp
		src/mongodb/mongo_kb.cpp
		src/mongodb/bson_pl.cpp
		src/mongodb/MongoInterface.cpp
		src/mongodb/MongoDatabase.cpp
		src/mongodb/MongoCollection.cpp
		src/mongodb/MongoCursor.cpp
		src/mongodb/MongoWatch.cpp
		src/mongodb/MongoKnowledgeGraph.cpp
		src/mongodb/MongoBulkOperation.cpp
		src/URI.cpp
		include/knowrob/graphs/xsd.cpp)
target_link_libraries(knowrob_qa
		${SWIPL_LIBRARIES}
		${MONGOC_LIBRARIES}
		${RAPTOR_LIBRARIES}
		spdlog::spdlog
		${GTEST_LIBRARIES})

add_executable(knowrob-terminal src/terminal.cpp)
target_link_libraries(knowrob-terminal
		Boost::program_options
		Boost::serialization
		knowrob_qa)
install(TARGETS knowrob-terminal RUNTIME DESTINATION bin)

##############
########## ROS1 block
##############

if (catkin_FOUND)
	# install into ROS workspace
	add_dependencies(mongo_kb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	add_dependencies(knowrob_qa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	add_dependencies(knowrob-terminal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

	## RViz marker ###
	add_library(marker_knowrob SHARED src/ros/marker/publisher.cpp)
	target_link_libraries(marker_knowrob
		${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	add_dependencies(marker_knowrob
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})
	
	## URDf ###
	add_library(urdf_parser SHARED src/ros/urdf/parser.cpp)
	target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	add_dependencies(urdf_parser
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS})

	## ROS Interface ###
	add_executable(knowrob-ros src/ROSInterface.cpp src/IterativeQAResultHandler.cpp)
	target_link_libraries(knowrob-ros
			Boost::program_options
			knowrob_qa
			${catkin_LIBRARIES})
	install(TARGETS knowrob-ros RUNTIME DESTINATION bin)
endif()

##############
########## Unit Testing
##############

# add an executable target for GTest.
# but the testing code is actually in the *knowrob_qa* library
# where gtest won't find them without using the "--no-as-needed"
# flag for the linker.
add_executable(all_gtests
		tests/gtests.cpp
		src/neems/test.cpp)
target_link_libraries(all_gtests
		-Wl,--whole-archive,--no-as-needed
		knowrob_qa
		-Wl,--no-whole-archive
		${GTEST_MAIN_LIBRARIES})

##############
##############

# TODO: look into installation
#install(DIRECTORY owl/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
#install(DIRECTORY prolog/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

install(DIRECTORY src/reasoner/prolog DESTINATION share
		PATTERN ".cpp" EXCLUDE)
