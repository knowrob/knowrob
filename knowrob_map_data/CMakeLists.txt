
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_map_data)


find_package(catkin REQUIRED COMPONENTS knowrob_map_tools)

catkin_package(
    DEPENDS knowrob_map_tools
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
