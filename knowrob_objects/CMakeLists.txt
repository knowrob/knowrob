
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_objects)

find_package(catkin REQUIRED COMPONENTS knowrob_common)

catkin_package(
    CATKIN_DEPENDS knowrob_common tf_prolog
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
