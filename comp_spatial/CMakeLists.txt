cmake_minimum_required(VERSION 2.8.3)
project(comp_spatial)

find_package(catkin REQUIRED COMPONENTS rosprolog knowrob_common)

catkin_package(CATKIN_DEPENDS rosprolog knowrob_common)
catkin_add_plunit(knowrob/comp_spatial)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
