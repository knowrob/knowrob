
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_vis)

find_package(catkin REQUIRED rosjava_build_tools data_vis_msgs knowrob_common knowrob_mongo)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
    DEPENDS knowrob_common knowrob_mongo
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

install(DIRECTORY html/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/html)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(DIRECTORY schema/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/schema)
