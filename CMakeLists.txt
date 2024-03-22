cmake_minimum_required(VERSION 3.5.0)
set (CMAKE_CXX_FLAGS "-Wswitch -std=c++0x -pthread ${CMAKE_CXX_FLAGS} -Wno-undef")
set (CMAKE_CXX_STANDARD 17)
if(POLICY CMP0148) 
	cmake_policy(SET CMP0148 OLD) 
endif()

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

find_package(Python COMPONENTS Interpreter Development)

# Construct Boost Python component name based on the found Python version
set(Boost_Python_VERSION "python${Python_VERSION_MAJOR}${Python_VERSION_MINOR}")
set(Boost_Python_COMPONENT "Boost::${Boost_Python_VERSION}")
message(STATUS "Using Boost Python component: ${Boost_Python_COMPONENT}")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.50 REQUIRED COMPONENTS program_options serialization ${Boost_Python_VERSION})

enable_testing()
find_package(GTest REQUIRED)

pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})

pkg_check_modules(RAPTOR REQUIRED raptor2)
include_directories(include ${RAPTOR_INCLUDE_DIRS})

pkg_check_modules(REDLAND REQUIRED redland)
include_directories(include ${REDLAND_INCLUDE_DIRS})

# install ontologies hosted on http servers.
# copy them into the workspace such the we can load them
# from a local source at runtime.
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
		${Boost_INCLUDE_DIRS}
		${Python_INCLUDE_DIRS})

# define some constants for the compilation of source files that
# are used to load project-relative files.
# TODO: needs to be defined differently when catkin is used?
add_definitions(-DKNOWROB_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\")
add_definitions(-DKNOWROB_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")
add_definitions(-DKNOWROB_BUILD_DIR=\"${CMAKE_BINARY_DIR}\")
add_definitions(-DBOOST_ALLOW_DEPRECATED_HEADERS)
# ignore BOOST deprecated headers
add_definitions(-DBOOST_ALLOW_DEPRECATED_HEADERS)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

##############
########## ROS1 block
##############
# NOTE: seems catkin_package must be called *before* any library is build.

find_package(catkin QUIET COMPONENTS
		roscpp roslib tests/urdf message_generation
		geometry_msgs message_generation json_prolog_msgs actionlib actionlib_msgs)

if (false)
	message(STATUS "Building with ROS1 support.")

	add_definitions(-DKNOWROB_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

	include_directories(${catkin_INCLUDE_DIRS})

	catkin_python_setup()

	# build messages
	add_action_files(
			DIRECTORY action
			FILES
			askall.action
			askone.action
			askincremental.action
			tell.action
	)

	add_message_files(
			FILES
			KeyValuePair.msg
			GraphQueryMessage.msg
			GraphAnswerMessage.msg
			EventToken.msg
	)

	generate_messages(DEPENDENCIES
			std_msgs
			geometry_msgs
			actionlib_msgs)

	catkin_package(
			CATKIN_DEPENDS roscpp roslib
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

add_library(knowrob SHARED
		src/knowrob.cpp
		src/ThreadPool.cpp
		src/Logger.cpp
		src/KnowledgeBase.cpp
        src/sources/DataSource.cpp
		src/URI.cpp
        src/TimePoint.cpp
		src/TimeInterval.cpp
		src/terms/Term.cpp
		src/terms/Variable.cpp
		src/terms/ListTerm.cpp
		src/terms/OptionList.cpp
        src/terms/Bindings.cpp
		src/terms/Unifier.cpp
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
		src/formulas/ModalOperator.cpp
        src/formulas/FirstOrderLiteral.cpp
		src/queries/Answer.cpp
		src/queries/TokenStream.cpp
		src/queries/TokenQueue.cpp
		src/queries/ConjunctiveBroadcaster.cpp
		src/queries/TokenBroadcaster.cpp
		src/formulas/DependencyGraph.cpp
		src/queries/QueryParser.cpp
		src/queries/QueryTree.cpp
		src/queries/TokenBuffer.cpp
        src/triples/GraphQuery.cpp
		src/queries/Query.cpp
		src/queries/RedundantAnswerFilter.cpp
		src/queries/QueryStage.cpp
		src/queries/QueryPipeline.cpp
		src/queries/NegationStage.cpp
		src/semweb/PrefixRegistry.cpp
		src/semweb/PrefixProbe.cpp
		src/semweb/xsd.cpp
		src/semweb/Vocabulary.cpp
		src/semweb/owl.cpp
		src/semweb/rdfs.cpp
		src/semweb/rdf.cpp
		src/semweb/Property.cpp
		src/semweb/Class.cpp
        src/triples/FramedTriplePattern.cpp
		src/semweb/ImportHierarchy.cpp
		src/reasoner/DefinedPredicate.cpp
		src/reasoner/ReasonerPlugin.cpp
		src/reasoner/Reasoner.cpp
		src/reasoner/ReasonerManager.cpp
        src/PropertyTree.cpp
        src/reasoner/prolog/PrologEngine.cpp
		src/reasoner/prolog/PrologReasoner.cpp
		src/reasoner/prolog/PrologTests.cpp
		src/reasoner/prolog/logging.cpp
        src/reasoner/prolog/ext/algebra.cpp
		src/reasoner/mongolog/MongologReasoner.cpp
		src/reasoner/mongolog/mongo_kb.cpp
		src/reasoner/mongolog/bson_pl.cpp
		src/reasoner/swrl/SWRLReasoner.cpp
		src/reasoner/esg/ESGReasoner.cpp
        src/backend/QueryableBackend.cpp
        src/backend/BackendManager.cpp
        src/backend/BackendPlugin.cpp
        src/sources/DataSourceHandler.cpp
        src/sources/OntologyFile.cpp
        src/sources/OntologyParser.cpp
        src/sources/DataFile.cpp
        src/backend/BackendTransaction.cpp
        src/backend/mongo/MongoInterface.cpp
        src/backend/mongo/Database.cpp
        src/backend/mongo/Collection.cpp
        src/backend/mongo/ChangeStream.cpp
        src/backend/mongo/Cursor.cpp
        src/backend/mongo/QueryWatch.cpp
        src/backend/mongo/MongoKnowledgeGraph.cpp
        src/backend/mongo/BulkOperation.cpp
        src/backend/mongo/Pipeline.cpp
        src/backend/mongo/TripleCursor.cpp
        src/backend/mongo/BindingsCursor.cpp
        src/backend/mongo/Connection.cpp
        src/backend/mongo/bson-helper.cpp
		src/queries/ModalStage.cpp
        src/triples/Perspective.cpp
		src/formulas/PredicateIndicator.cpp
		src/formulas/PredicateDescription.cpp
		src/reasoner/ReasonerModule.cpp
		src/queries/AnswerYes.cpp
		src/queries/AnswerNo.cpp
		src/queries/AnswerDontKnow.cpp
		src/queries/DisjunctiveBroadcaster.cpp
		src/queries/Token.cpp
		src/triples/GraphSelector.cpp
		src/queries/AnswerMerger.cpp
		src/py/utils.cpp
        src/triples/FramedTriple.cpp
        src/triples/TripleFormat.cpp
        src/semweb/OntologyLanguage.cpp
        src/sources/DataService.cpp
        src/reasoner/prolog/semweb.cpp
        src/reasoner/prolog/PrologBackend.cpp
        src/reasoner/prolog/PrologTerm.cpp
        src/sources/SPARQLService.cpp
        src/backend/redland/RaptorContainer.cpp
        src/alignment/GraphRenaming.cpp
        src/alignment/GraphRestructuring.cpp
        src/backend/redland/RedlandModel.cpp
        src/alignment/GraphTransformation.cpp
        src/terms/Atom.cpp
        src/terms/Atomic.cpp
        src/terms/Function.cpp
        src/terms/XSDAtomic.cpp
        src/terms/IRIAtom.cpp
        src/terms/Blank.cpp
        src/terms/RDFNode.cpp
        src/semweb/Resource.cpp
        src/terms/Numeric.cpp
        src/terms/String.cpp
        src/backend/DataBackend.cpp
        src/queries/QueryContext.cpp
        src/triples/SPARQLQuery.cpp
        src/triples/TripleContainer.cpp
        src/KnowRobError.cpp
        src/py/PythonError.cpp
        src/reification/ReifiedTriple.cpp
        src/reification/ReifiedQuery.cpp
        src/reification/ReificationContainer.cpp
        src/reification/UnReificationContainer.cpp
        src/backend/BackendInterface.cpp
        src/triples/GraphUnion.cpp
        src/triples/GraphSequence.cpp
        src/triples/GraphTerm.cpp
        src/backend/mongo/MongoTriple.cpp
        src/backend/mongo/MongoTriplePattern.cpp
        src/backend/mongo/MongoTerm.cpp
        src/backend/mongo/MongoTaxonomy.cpp
        src/backend/SPARQLBackend.cpp
        src/queries/parsers/strings.cpp
        src/queries/parsers/terms.cpp
        src/queries/parsers/formula.cpp)
set(KNOWROB_LIBRARIES
		${SWIPL_LIBRARIES}
		${MONGOC_LIBRARIES}
		${RAPTOR_LIBRARIES}
		${REDLAND_LIBRARIES}
		${Python_LIBRARIES}
		spdlog::spdlog
		${GTEST_LIBRARIES})
target_link_libraries(knowrob ${KNOWROB_LIBRARIES})

add_executable(knowrob-terminal src/terminal.cpp)
target_link_libraries(knowrob-terminal
		${KNOWROB_LIBRARIES}
		Boost::program_options
		Boost::serialization
		${Boost_Python_COMPONENT}
		knowrob)
install(TARGETS knowrob-terminal RUNTIME DESTINATION bin)

##############
########## Python block
##############

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

add_library(knowrob_py MODULE src/py/knowrob_module.cpp)
target_include_directories(knowrob_py PRIVATE
		${SWIPL_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${Boost_INCLUDE_DIRS}
		${PYTHON_INCLUDE_DIRS})
target_link_libraries(knowrob_py PRIVATE
		${KNOWROB_LIBRARIES}
		Boost::program_options
		Boost::serialization
		${Boost_Python_COMPONENT}
		knowrob)
# Rename the library to knowrob.so such that it can be loaded by the
# Python interpreter through the "import knowrob" statement.
# NOTE: The library needs to be copied into site-packages directory of Python, or
#       the PYTHONPATH needs to be set to include the build directory.
add_custom_command(TARGET knowrob_py POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_BINARY_DIR}/knowrob_py.so
		${CMAKE_BINARY_DIR}/knowrob.so
		COMMAND_EXPAND_LISTS)

##############
########## ROS1 block
##############

if (false)

	# install into ROS workspace
	#add_dependencies(mongo_kb ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	add_dependencies(knowrob ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	add_dependencies(knowrob-terminal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

	#	### TF messages ###
	#	add_library(tf_knowrob SHARED
	#		src/ros/tf/tf.cpp
	#		src/ros/tf/memory.cpp
	#		src/ros/tf/logger.cpp
	#		src/ros/tf/publisher.cpp
	#		src/ros/tf/republisher.cpp)
	#	target_link_libraries(tf_knowrob
	#		${SWIPL_LIBRARIES}
	#		${MONGOC_LIBRARIES}
	#		${catkin_LIBRARIES})
	#		#mongo_kb
	#	add_dependencies(tf_knowrob
	#		${${PROJECT_NAME}_EXPORTED_TARGETS}
	#		${catkin_EXPORTED_TARGETS})
	#
	#	## RViz marker ###
	#	add_library(marker_knowrob SHARED src/ros/marker/publisher.cpp)
	#	target_link_libraries(marker_knowrob
	#		${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	#	add_dependencies(marker_knowrob
	#		${${PROJECT_NAME}_EXPORTED_TARGETS}
	#		${catkin_EXPORTED_TARGETS})
	#
	#	## URDf ###
	#	add_library(urdf_parser SHARED src/ros/urdf/parser.cpp)
	#	target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
	#	add_dependencies(urdf_parser
	#		${${PROJECT_NAME}_EXPORTED_TARGETS}
	#		${catkin_EXPORTED_TARGETS})

	## ROS Interface ###
	add_executable(knowrob-ros
			src/ros/ROSInterface.cpp
	)
	add_dependencies(knowrob-ros
			${catkin_EXPORTED_TARGETS})
	target_link_libraries(knowrob-ros
			Boost::program_options
			knowrob
			${catkin_LIBRARIES})
	install(TARGETS knowrob-ros RUNTIME DESTINATION bin)
endif()

##############
########## Unit Testing
##############

# add an executable target for GTest.
# but the testing code is partly in the *knowrob* library
# where gtest won't find them without using the "--no-as-needed"
# flag for the linker.
add_executable(all_gtests
		tests/gtests.cpp
		src/neems/test.cpp
        tests/QueryParserTest.cpp
		tests/KnowledgeBaseTest.cpp
        tests/DataBackendTest.cpp
		tests/py/test-module.cpp)
add_dependencies(all_gtests knowrob_py)
target_link_libraries(all_gtests
		-Wl,--whole-archive,--no-as-needed
		knowrob
		-Wl,--no-whole-archive
		${Boost_Python_COMPONENT}
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
