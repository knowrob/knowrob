
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_mongo)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++0x -pthread ${CMAKE_CXX_FLAGS}")

# find SWI Prolog libraries
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})

pkg_check_modules(MONGOC REQUIRED libmongoc-1.0)
# find_package(libmongoc-1.0 1.7 REQUIRED)
include_directories(${MONGOC_INCLUDE_DIRS})
link_directories(${MONGOC_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS rosprolog knowrob)

# TODO: would be nice to get rid of knowrob dependency
catkin_package(CATKIN_DEPENDS knowrob rosprolog)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mongo_kb
	SHARED
	src/mongo_kb.cpp
	src/bson_pl.cpp
	src/MongoException.cpp
	src/MongoInterface.cpp
	src/MongoDatabase.cpp
	src/MongoCollection.cpp
	src/MongoCursor.cpp)
target_link_libraries(mongo_kb
	${SWIPL_LIBRARIES}
	${MONGOC_LIBRARIES}
	${catkin_LIBRARIES})

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
