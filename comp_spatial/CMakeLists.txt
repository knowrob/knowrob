cmake_minimum_required(VERSION 2.8.3)
project(comp_spatial)

find_package(catkin REQUIRED COMPONENTS knowrob_common comp_temporal)

catkin_package(
    CATKIN_DEPENDS knowrob_common comp_temporal
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
