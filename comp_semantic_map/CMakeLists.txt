# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(comp_semantic_map)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS ias_knowledge_base comp_spatial comp_temporal ias_prolog_addons)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS ias_knowledge_base comp_spatial comp_temporal ias_prolog_addons
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)