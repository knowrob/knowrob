cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

# Make sure some specific warnings are enabled
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wswitch -Wno-undef -Wnon-virtual-dtor")
set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0148)
	cmake_policy(SET CMP0148 OLD)
endif()

option(CATKIN "Build as a catkin project." ON)

if(NOT PYTHON_MODULE_LIBDIR)
	# On non-debian systems, use "site-packages" instead.
	set(PYTHON_MODULE_LIBDIR "dist-packages")
endif()

option(KNOWROB_USE_STATIC_LIB "Link against static libraries" OFF)

# Read version from the package.xml file.
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml package_xml_str)

if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
	message(FATAL_ERROR "Could not parse project version from package.xml. Aborting.")
endif()

set(PROJECT_NAME "knowrob")
set(PROJECT_VERSION ${CMAKE_MATCH_1})
set(PROJECT_DESCRIPTION "A Knowledge Base System for Cognition-enabled Robots")
set(PROJECT_WEBSITE "https://github.com/knowrob/knowrob")
message(STATUS "KnowRob version: ${PROJECT_VERSION}")

project(knowrob
	VERSION ${PROJECT_VERSION}
	DESCRIPTION ${PROJECT_DESCRIPTION}
	HOMEPAGE_URL "https://github.com/knowrob/knowrob"
)

IF(NOT WIN32)
	INCLUDE(FindPkgConfig)
ENDIF()

set(SOMA_VERSION "2.1.0")

# include additional CMake macros
include("cmake/ontologies.cmake")

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(spdlog REQUIRED)
set(Python3_EXECUTABLE "/usr/bin/python" CACHE FILEPATH "Path to specific Python3 executable")
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Found Python3 include: ${Python3_INCLUDE_DIRS}")
message(STATUS "Found Python3 libraries: ${Python3_LIBRARIES}")
execute_process(
	COMMAND ${Python3_EXECUTABLE}-config --extension-suffix
	OUTPUT_VARIABLE PYTHON3_SUFFIX
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Construct Boost Python component name based on Python version
set(Boost_Python_VERSION "python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}")
set(Boost_Python_COMPONENT "Boost::${Boost_Python_VERSION}")
message(STATUS "Using Boost Python component: ${Boost_Python_COMPONENT}")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

if(DEFINED Boost_ROOT AND Boost_ROOT)
	# Enable Boost_ROOT usage properly
	if(POLICY CMP0074)
		cmake_policy(SET CMP0074 NEW)
	endif()

	# Set CMake's internal Boost root hint
	message(STATUS "Using user-defined Boost_ROOT: ${Boost_ROOT}")
	find_package(Boost REQUIRED COMPONENTS program_options serialization ${Boost_Python_VERSION})
else()
	find_package(Boost 1.50 REQUIRED COMPONENTS program_options serialization ${Boost_Python_VERSION})
endif()

enable_testing()
find_package(GTest REQUIRED)

pkg_check_modules(SWIPL REQUIRED swipl)
pkg_check_modules(RAPTOR2 REQUIRED raptor2)
pkg_check_modules(REDLAND REQUIRED redland)

link_directories(${SWIPL_LIBRARY_DIRS} ${RAPTOR2_LIBRARY_DIRS} ${REDLAND_LIBRARY_DIRS})

# install ontologies hosted on http servers.
# copy them into the workspace such the we can load them
# from a local source at runtime.
install_ontology(
	URL "http://www.ease-crc.org/ont/DUL.owl"
	VERSION "3.34")
install_ontology(
	URL "https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA.owl"
	VERSION "${SOMA_VERSION}")
install_ontology(
	URL "https://ease-crc.github.io/soma/owl/${SOMA_VERSION}/SOMA-HOME.owl"
	VERSION "${SOMA_VERSION}")

# install all ontologies files
install(DIRECTORY owl
	DESTINATION share/knowrob
	PATTERN "catalog-*.xml" EXCLUDE)

# define some constants for the compilation of source files that
# are used to load project-relative files.
add_definitions(-DKNOWROB_INSTALL_PREFIX=\"${CMAKE_INSTALL_PREFIX}\")
add_definitions(-DKNOWROB_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\")
add_definitions(-DKNOWROB_BUILD_DIR=\"${CMAKE_BINARY_DIR}\")

# ignore BOOST deprecated headers
add_definitions(-DBOOST_ALLOW_DEPRECATED_HEADERS)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# enable the implicit fmt ostream formatting
add_definitions(-DFMT_DEPRECATED_OSTREAM=1)

# #############
if(${KNOWROB_USE_STATIC_LIB})
	pkg_check_modules(BSON2 REQUIRED bson2-static)
	pkg_check_modules(MONGOC REQUIRED mongoc2-static)
	list(GET MONGOC_INCLUDE_DIRS 0 MONGOC_DIR)
	list(GET MONGOC_INCLUDE_DIRS 1 BSON2_DIR)
	set(MONGOC_INCLUDE_DIRS "${MONGOC_DIR};${MONGOC_DIR}/mongoc;${BSON2_DIR};${BSON2_DIR}/bson")
else()
	pkg_check_modules(MONGOC REQUIRED libmongoc-1.0)
endif()

# find_package(libmongoc-1.0 1.7 REQUIRED)
link_directories(${MONGOC_LIBRARY_DIRS})

# #############
# ######### Catkin
# #############
if(CATKIN)
	find_package(catkin QUIET)

	if(catkin_FOUND AND CATKIN_DEVEL_PREFIX)
		message(STATUS "Found catkin workspace at: ${CATKIN_DEVEL_PREFIX}")

		# make catkin happy
		set(spdlog_INCLUDE_DIRS "")
		set(GTest_INCLUDE_DIRS "")
		set(-lswipl_INCLUDE_DIRS "")

		catkin_package(
			INCLUDE_DIRS include
			LIBRARIES knowrob
			DEPENDS Boost -lswipl MONGOC RAPTOR2 REDLAND spdlog Python3 GTest EIGEN3
		)
	else()
		message(STATUS "No catkin workspace found.")
	endif()
endif()

# #############
# ### SHARED LIBRARY
# #############
set(knowrob_INCLUDE_DIRS
	include
	${SWIPL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${Python3_INCLUDE_DIRS}
	${MONGOC_INCLUDE_DIRS}
	${RAPTOR2_INCLUDE_DIRS}
	${REDLAND_INCLUDE_DIRS})
include_directories(${knowrob_INCLUDE_DIRS})

option(KNOWROB_USE_SUFFIX "Append fixed suffix to KnowRob module name" OFF)

if(KNOWROB_USE_STATIC_LIB)
	set(KNOWROB_LIBRARY_TYPE "STATIC")
else()
	set(KNOWROB_LIBRARY_TYPE "SHARED")
endif()

add_library(knowrob ${KNOWROB_LIBRARY_TYPE}
	src/knowrob.cpp
	src/ThreadPool.cpp
	src/Logger.cpp
	src/KnowledgeBase.cpp
	src/DataSource.cpp
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
	src/semweb/GraphQuery.cpp
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
	src/semweb/TriplePattern.cpp
	src/semweb/ImportHierarchy.cpp
	src/reasoner/Reasoner.cpp
	src/reasoner/ReasonerManager.cpp
	src/PropertyTree.cpp
	src/integration/prolog/PrologEngine.cpp
	src/integration/InterfaceUtils.cpp
	src/reasoner/prolog/PrologReasoner.cpp
	src/reasoner/prolog/PrologTests.cpp
	src/integration/prolog/logging.cpp
	src/integration/prolog/ext/algebra.cpp
	src/reasoner/mongolog/MongologReasoner.cpp
	src/reasoner/mongolog/mongo_kb.cpp
	src/reasoner/mongolog/bson_pl.cpp
	src/reasoner/swrl/SWRLReasoner.cpp
	src/reasoner/esg/ESGReasoner.cpp
	src/storage/QueryableStorage.cpp
	src/storage/StorageManager.cpp
	src/DataSourceHandler.cpp
	src/semweb/OntologyFile.cpp
	src/semweb/OntologyParser.cpp
	src/storage/Transaction.cpp
	src/storage/mongo/MongoInterface.cpp
	src/storage/mongo/Database.cpp
	src/storage/mongo/Collection.cpp
	src/storage/mongo/ChangeStream.cpp
	src/storage/mongo/Cursor.cpp
	src/storage/mongo/QueryWatch.cpp
	src/storage/mongo/MongoKnowledgeGraph.cpp
	src/storage/mongo/BulkOperation.cpp
	src/storage/mongo/Pipeline.cpp
	src/storage/mongo/TripleCursor.cpp
	src/storage/mongo/BindingsCursor.cpp
	src/storage/mongo/Connection.cpp
	src/storage/mongo/bson-helper.cpp
	src/queries/ModalStage.cpp
	src/semweb/Perspective.cpp
	src/formulas/PredicateIndicator.cpp
	src/queries/AnswerYes.cpp
	src/queries/AnswerNo.cpp
	src/queries/AnswerDontKnow.cpp
	src/queries/DisjunctiveBroadcaster.cpp
	src/queries/Token.cpp
	src/semweb/GraphSelector.cpp
	src/queries/AnswerMerger.cpp
	src/integration/python/utils.cpp
	src/semweb/Triple.cpp
	src/semweb/TripleFormat.cpp
	src/semweb/OntologyLanguage.cpp
	src/reasoner/prolog/semweb.cpp
	src/integration/prolog/PrologBackend.cpp
	src/integration/prolog/PrologTerm.cpp
	src/semweb/SPARQLService.cpp
	src/storage/redland/RaptorContainer.cpp
	src/semweb/GraphRenaming.cpp
	src/semweb/GraphRestructuring.cpp
	src/storage/redland/RedlandModel.cpp
	src/semweb/GraphTransformation.cpp
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
	src/storage/Storage.cpp
	src/queries/QueryContext.cpp
	src/semweb/SPARQLQuery.cpp
	src/semweb/TripleContainer.cpp
	src/KnowRobError.cpp
	src/integration/python/PythonError.cpp
	src/storage/ReifiedTriple.cpp
	src/storage/ReifiedQuery.cpp
	src/storage/ReificationContainer.cpp
	src/storage/UnReificationContainer.cpp
	src/storage/StorageInterface.cpp
	src/semweb/GraphUnion.cpp
	src/semweb/GraphSequence.cpp
	src/semweb/GraphTerm.cpp
	src/storage/mongo/MongoTriple.cpp
	src/storage/mongo/MongoTriplePattern.cpp
	src/storage/mongo/MongoTerm.cpp
	src/storage/mongo/MongoTaxonomy.cpp
	src/storage/SPARQLBackend.cpp
	src/queries/parsers/strings.cpp
	src/queries/parsers/terms.cpp
	src/queries/parsers/formula.cpp
	src/reasoner/GoalDrivenReasoner.cpp
	src/reasoner/DataDrivenReasoner.cpp
	src/reasoner/ReasonerEvent.cpp
	src/semweb/TransformedOntology.cpp
	src/reasoner/Goal.cpp
	src/formulas/SimpleConjunction.cpp
	src/queries/ConjunctiveQuery.cpp
	src/semweb/RDFIndicator.cpp
	src/integration/python/converter/shared_ptr.cpp
	src/semweb/GraphBuiltin.cpp
	src/reasoner/RDFGoal.cpp
	src/reasoner/RDFGoalReasoner.cpp
	src/reasoner/Computable.cpp
	src/storage/Observer.cpp
	src/storage/ObserverManager.cpp
	src/storage/ObserverJob.cpp
	src/queries/parsers/graph.cpp)

if(NOT KNOWROB_USE_STATIC_LIB)
	set(KNOWROB_LIBRARIES
		${SWIPL_LIBRARIES}
		${MONGOC_LIBRARIES}
		${RAPTOR2_LIBRARIES}
		${REDLAND_LIBRARIES}
		${Python3_LIBRARIES}
		spdlog::spdlog
		${GTEST_LIBRARIES})
else()
	pkg_check_modules(RAPTOR2 REQUIRED raptor2)
	pkg_check_modules(SPDLOG REQUIRED spdlog)
	pkg_check_modules(CURL REQUIRED libcurl)
	pkg_check_modules(GMP REQUIRED gmp)
	pkg_check_modules(YAJL REQUIRED yajl)
	pkg_check_modules(RASQAL REQUIRED rasqal)
	link_directories(${RAPTOR2_LIBRARY_DIRS} ${SPDLOG_LIBRARY_DIRS} ${CURL_LIBRARY_DIRS} ${GMP_LIBRARY_DIRS} ${YAJL_LIBRARY_DIRS} ${RASQAL_LIBRARY_DIRS})
	set(KNOWROB_LIBRARIES
		${SWIPL_STATIC_LIBRARIES}
		${MONGOC_DIR}/../../lib/libmongoc2.a
		libraptor2.a
		librdf.a
		libspdlog.a
		libcurl.a
		libcrypto.a
		libssl.a
		${MONGOC_DIR}/../../lib/libbson2.a
		librasqal.a
		gmp.a
		yajl_s.a
		${Python3_LIBRARIES}
		${GTEST_LIBRARIES}
		resolv
		zstd
		ltdl
		tinfo
		fmt
		dl
		pcre
		uuid
		xml2
		z
		lzma
		xslt)
endif()
set(OpenSSL_LIBRARIES "")  # Unset to avoid leakage
target_link_libraries(knowrob ${KNOWROB_LIBRARIES})
target_compile_options(knowrob PRIVATE -fPIC)

# install shared library and headers
if(KNOWROB_USE_SUFFIX)
	string(REPLACE ".so" "" PYTHON3_SUFFIX_BASE "${PYTHON3_SUFFIX}")
	message(STATUS "Installing knowrob with suffix ${PYTHON3_SUFFIX_BASE}")
	set_target_properties(knowrob PROPERTIES OUTPUT_NAME "knowrob${PYTHON3_SUFFIX_BASE}")
endif()

install(TARGETS knowrob
	PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/knowrob)

install(DIRECTORY include/knowrob
	DESTINATION include)

# Install Python and Prolog files into the share directory such that they can be loaded at runtime.
install(DIRECTORY src/
	DESTINATION share/knowrob
	FILES_MATCHING PATTERN "*.pl"
	PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY src/
	DESTINATION share/knowrob
	FILES_MATCHING PATTERN "*.py"
	PATTERN "__pycache__" EXCLUDE)

# Generate a pkg-config file for the knowrob library.
foreach(LIB ${KNOWROB_LIBRARIES})
	set(PRIVATE_LIBS "${PRIVATE_LIBS} -l${LIB}")
endforeach()

configure_file(knowrob.pc.in knowrob.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/knowrob.pc
	DESTINATION lib/pkgconfig)

add_executable(knowrob-terminal src/terminal.cpp)
target_link_libraries(knowrob-terminal
	${KNOWROB_LIBRARIES}
	Boost::program_options
	Boost::serialization
	${Boost_Python_COMPONENT}
	knowrob)
install(TARGETS knowrob-terminal RUNTIME DESTINATION bin)

# #############
# ######### Python module
# #############

# Without this, any build libraries automatically have names "lib{x}.so"
set(CMAKE_SHARED_MODULE_PREFIX "")

add_library(knowrob_py
	MODULE src/integration/python/knowrob_module.cpp
)
target_include_directories(knowrob_py PRIVATE
	${SWIPL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${PYTHON3_INCLUDE_DIRS})
target_link_libraries(knowrob_py PRIVATE
	${KNOWROB_LIBRARIES}
	Boost::program_options
	Boost::serialization
	${Boost_Python_COMPONENT}
	knowrob)

# Rename the library to knowrob.so such that it can be loaded by the
# Python interpreter through the "import knowrob" statement.
# NOTE: The library needs to be copied into site-packages directory of Python, or
# the PYTHONPATH needs to be set to include the build directory.
if(CATKIN AND catkin_FOUND AND CATKIN_DEVEL_PREFIX)
	add_custom_command(TARGET knowrob_py POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
		${CATKIN_DEVEL_PREFIX}/lib/knowrob_py.so
		${CMAKE_BINARY_DIR}/knowrob.so
		COMMAND_EXPAND_LISTS)

	# If catkin is used (e.g. in context of ROS) add library in the devel space too.
	add_custom_command(TARGET knowrob_py POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
		${CATKIN_DEVEL_PREFIX}/lib/knowrob_py.so
		${CATKIN_DEVEL_PREFIX}/lib/python3/dist-packages/knowrob.so
		COMMAND_EXPAND_LISTS)
else()
	add_custom_command(TARGET knowrob_py POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy
		${CMAKE_BINARY_DIR}/knowrob_py.so
		${CMAKE_BINARY_DIR}/knowrob.so
		COMMAND_EXPAND_LISTS)
endif()

# install the Python module into the site-packages directory
install(FILES ${CMAKE_BINARY_DIR}/knowrob.so
	DESTINATION lib/python/${PYTHON_MODULE_LIBDIR}
	RENAME knowrob${PYTHON3_SUFFIX})

# #############
# ######### Doxygen
# #############
find_package(Doxygen)

if(DOXYGEN_FOUND)
	set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/doxyfile.in)
	set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/doc/doxyfile)
	configure_file(${doxyfile_in} ${doxyfile} @ONLY)

	if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome.css")
		message(STATUS "Downloading \"doxygen-awesome.css\" into directory \"${CMAKE_CURRENT_BINARY_DIR}\".")
		file(DOWNLOAD
			https://raw.githubusercontent.com/jothepro/doxygen-awesome-css/v2.3.2/doxygen-awesome.css
			${CMAKE_CURRENT_BINARY_DIR}/doxygen-awesome.css)
	endif()

	add_custom_target(doc
		COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT "Generating API documentation with Doxygen"
		VERBATIM)
endif()

# #############
# ######### CPack
# #############
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION ${PROJECT_DESCRIPTION})
set(CPACK_PACKAGE_HOMEPAGE_URL ${PROJECT_WEBSITE})
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Daniel Beßler <danielb@uni-bremen.de>")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-program-options-dev, libboost-serialization-dev, libboost-python-dev, libeigen3-dev, libmongoc-dev, libraptor2-dev, librdf0-dev, libspdlog-dev, libgtest-dev, swi-prolog, python3-dev")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
include(CPack)

# #############
# ######### Unit Testings
# #############

# add an executable target for GTest.
# but the testing code is partly in the *knowrob* library
# where gtest won't find them without using the "--no-as-needed"
# flag for the linker.
add_executable(all_gtests
	tests/gtests.cpp
	tests/QueryParserTest.cpp
	tests/KnowledgeBaseTest.cpp
	tests/StorageTest.cpp
	tests/py/test-module.cpp
	tests/TokenBroadcasterTest.cpp
	tests/QueryTreeTest.cpp
	tests/DependencyGraphTest.cpp
	tests/DataSourceTest.cpp
	tests/ConjunctiveBroadcasterTest.cpp
	tests/UnifierTest.cpp
	tests/py/test-jupyter.cpp
	tests/py/PythonTests.cpp)
add_dependencies(all_gtests knowrob_py)

# add sub-directory for plugin libraries
add_subdirectory(tests/plugins)

if(KNOWROB_USE_STATIC_LIB)
	set(GTEST_MAIN_LIBRARIES "${GTEST_MAIN_LIBRARIES};${MONGOC_STATIC_LIBRARIES}")
endif()

target_link_libraries(all_gtests
	-Wl,--whole-archive,--no-as-needed
	knowrob
	-Wl,--no-whole-archive
	${Boost_Python_COMPONENT}
	${GTEST_MAIN_LIBRARIES})
