
cmake_minimum_required(VERSION 2.8.3)
project(urdfprolog)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS knowrob_common urdf xacro rosprolog)

catkin_package(
  CATKIN_DEPENDS knowrob_common urdf rosprolog
  LIBRARIES urdf_parser
)
catkin_add_plunit(rdf_urdf urdf_parser)

# find SWI-Prolog package configuration
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
link_directories(${SWIPL_LIBRARY_DIRS})
message(STATUS "SWIPL found.")

# build C++ library with predicates to parse URDF files
include_directories(${SWIPL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(urdf_parser SHARED src/urdf_parser.cpp)
target_link_libraries(urdf_parser ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})

# Xacro files
set(xacro_files_out "")
file(GLOB xacro_files_in ${CMAKE_CURRENT_SOURCE_DIR}/owl/*.xacro)
foreach(it ${xacro_files_in})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  set(expanded_file "${basepath}/${basename}.owl")
  xacro_add_xacro_file(${it} ${expanded_file})
  set(xacro_files_out ${xacro_files_out} ${expanded_file})
endforeach(it)
add_custom_target(srdl_media_files ALL DEPENDS ${xacro_files_out})
        
install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
