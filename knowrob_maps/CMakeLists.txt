
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_maps)


find_package(catkin REQUIRED COMPONENTS comp_spatial)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS comp_spatial rospy
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(PROGRAMS
  scripts/urdf_to_sem
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
