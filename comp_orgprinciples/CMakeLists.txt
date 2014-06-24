
cmake_minimum_required(VERSION 2.8.3)
project(comp_orgprinciples)

find_package(catkin REQUIRED COMPONENTS rosprolog knowrob_common)

catkin_package(
    DEPENDS rosprolog knowrob_common
)