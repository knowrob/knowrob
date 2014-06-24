
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_srdl)

find_package(catkin REQUIRED rosjava_build_tools knowrob_common urdf)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)

catkin_package(
  DEPENDS knowrob_common urdf
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
