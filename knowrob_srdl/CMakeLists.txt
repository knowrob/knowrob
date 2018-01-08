
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_srdl)

find_package(catkin REQUIRED COMPONENTS rosjava_build_tools knowrob_common knowrob_actions urdf xacro)

catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository writeClasspath)

catkin_package(
  CATKIN_DEPENDS knowrob_common urdf
)

# Xacro files
set(xacro_files_out "")
file(GLOB xacro_files_in ${CMAKE_CURRENT_SOURCE_DIR}/owl/*.xacro)
foreach(it ${xacro_files_in})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME_WE)
  set(expanded_file "${basepath}/${basename}.owl")
  xacro_add_xacro_file(${it} ${expanded_file})
  set(xacro_files_out ${xacro_files_out} ${expanded_file})
endforeach(it)
add_custom_target(srdl_media_files ALL DEPENDS ${xacro_files_out})

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/org/knowrob/${PROJECT_NAME})

FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/)
        
install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
  
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(FILES knowrob_srdl/build/classpath.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/build )
