
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_actions)

find_package(catkin REQUIRED COMPONENTS knowrob_common knowrob_objects)

catkin_package(
    CATKIN_DEPENDS knowrob_common knowrob_objects
)
catkin_add_plunit(
  knowrob/action_effects
  knowrob/action_execution
  knowrob/action_parser
  knowrob/event_graph
  knowrob/task_planning
)

install(DIRECTORY owl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/owl)

install(DIRECTORY prolog/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/prolog)
