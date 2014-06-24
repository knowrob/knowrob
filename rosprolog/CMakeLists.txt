
cmake_minimum_required(VERSION 2.8.3)
project(rosprolog)

find_package(catkin REQUIRED COMPONENTS catkin)

catkin_package(
    DEPENDS swi-prolog swi-prolog-semweb swi-prolog-java
)