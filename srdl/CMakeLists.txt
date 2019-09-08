
cmake_minimum_required(VERSION 2.8.3)
project(srdl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS knowrob_common knowrob_actions urdf_prolog rosprolog rosowl)

catkin_package(
  CATKIN_DEPENDS knowrob_common urdf_prolog rosprolog rosowl
)

# find SWI-Prolog package configuration
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
message(STATUS "SWIPL found.")
        
install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
