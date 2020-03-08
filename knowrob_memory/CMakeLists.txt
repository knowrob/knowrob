cmake_minimum_required(VERSION 2.8.3)
project(knowrob_memory)

find_package(catkin REQUIRED COMPONENTS knowrob knowrob_mongo mongodb_log)

catkin_package(
    CATKIN_DEPENDS knowrob knowrob_mongo mongodb_log
)
catkin_add_plunit(
    knowrob/beliefstate
    knowrob/memory
)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
