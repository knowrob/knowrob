
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_common)

find_package(catkin REQUIRED COMPONENTS rosprolog rosowl rosjava_build_tools)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)

catkin_rosjava_setup(installDist publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
    CATKIN_DEPENDS rosprolog
)

include_directories(include ${SWIPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(eigen SHARED src/eigen.cpp)
target_link_libraries(eigen ${SWIPL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(eigen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(FILES knowrob_common/build/classpath.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/build )
