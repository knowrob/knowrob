cmake_minimum_required(VERSION 2.8.3)
project(swrl_prolog)

find_package(catkin REQUIRED COMPONENTS knowrob_common)

catkin_package(
    CATKIN_DEPENDS knowrob_common
)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
