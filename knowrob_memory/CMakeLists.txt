cmake_minimum_required(VERSION 2.8.3)
project(knowrob_memory)

find_package(catkin REQUIRED COMPONENTS knowrob_common knowrob_mongo knowrob_objects knowrob_actions mongodb_log)

catkin_package(
    CATKIN_DEPENDS knowrob_common knowrob_mongo knowrob_objects knowrob_actions mongodb_log
)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
