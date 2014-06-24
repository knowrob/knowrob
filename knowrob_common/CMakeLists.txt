
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_common)


find_package(catkin REQUIRED COMPONENTS rosprolog rosjava_build_tools)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
    DEPENDS rosprolog
)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})
