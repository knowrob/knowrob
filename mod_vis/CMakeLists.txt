# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mod_vis)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS knowrob_cad_parser ias_knowledge_base)


add_service_files(
  FILES CommVisSetLeftImg.srv CommVisSetReqText.srv CommVisSetResText.srv CommVisSetRightImg.srv
)

generate_messages()

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS knowrob_cad_parser ias_knowledge_base
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)