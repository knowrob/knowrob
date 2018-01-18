
cmake_minimum_required(VERSION 2.8.3)
project(tfjava)

find_package(catkin REQUIRED COMPONENTS rosprolog rosjava_build_tools)

catkin_rosjava_setup(publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
    CATKIN_DEPENDS rosprolog
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)

install(FILES tfjava/build/classpath.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/build )
