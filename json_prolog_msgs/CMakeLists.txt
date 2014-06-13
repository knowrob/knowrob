
cmake_minimum_required(VERSION 2.8.3)
project(json_prolog_msgs)

find_package(catkin REQUIRED roscpp rospy std_msgs message_generation)

add_service_files(
  FILES PrologFinish.srv  PrologNextSolution.srv  PrologQuery.srv
)
## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS # TODO add dependencies
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

