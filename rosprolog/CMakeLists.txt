# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(rosprolog)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS catkin)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS swi-prolog swi-prolog-clib swi-prolog-http swi-prolog-odbc swi-prolog-semweb swi-prolog-sgml swi-prolog-xpce
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)