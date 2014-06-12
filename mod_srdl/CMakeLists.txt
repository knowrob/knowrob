# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mod_srdl)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS knowrob_common urdf)


# TODO: Build urdf converter (converter from urdf to srdl)
# add_jar_dir(lib)
# add_java_source_dir( ${PROJECT_SOURCE_DIR}/src )
# rospack_add_java_executable( urdf_to_owl org.roboearth.wp5.io.URDF2SRDL)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS knowrob_common urdf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)