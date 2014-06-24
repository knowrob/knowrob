
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_objects)

find_package(catkin REQUIRED COMPONENTS knowrob_common)

catkin_package(
    DEPENDS knowrob_common tf_prolog
)