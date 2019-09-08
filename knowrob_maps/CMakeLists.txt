
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_maps)


find_package(catkin REQUIRED COMPONENTS knowrob_common knowrob_objects comp_spatial)

catkin_package(
    CATKIN_DEPENDS knowrob_common knowrob_objects comp_spatial
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
