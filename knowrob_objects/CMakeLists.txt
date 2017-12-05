
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_objects)

find_package(catkin REQUIRED COMPONENTS knowrob_common message_generation)

catkin_python_setup()

add_service_files(FILES DirtyObject.srv)
generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS knowrob_common)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)
  
install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
