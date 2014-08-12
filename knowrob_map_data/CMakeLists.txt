
cmake_minimum_required(VERSION 2.8.3)
project(knowrob_map_data)


find_package(catkin REQUIRED COMPONENTS knowrob_map_tools)

catkin_package(
    DEPENDS knowrob_map_tools
)