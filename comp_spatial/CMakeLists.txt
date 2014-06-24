cmake_minimum_required(VERSION 2.8.3)
project(comp_spatial)

find_package(catkin REQUIRED COMPONENTS knowrob_common comp_temporal)

catkin_package(
    DEPENDS knowrob_common comp_temporal
)