
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_mongo)

find_package(catkin REQUIRED rosjava_build_tools tf_prolog rosprolog)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
  DEPENDS tf_prolog
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

install(FILES knowrob_mongo/build/classpath.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/build )
