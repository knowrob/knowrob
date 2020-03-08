
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_vis)

find_package(catkin REQUIRED COMPONENTS data_vis_msgs knowrob)

catkin_package(CATKIN_DEPENDS knowrob)

find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
