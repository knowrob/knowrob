cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

add_classpath(${ias_knowledge_base_PACKAGE_PATH}/lib/classifier_interface.jar)
add_classpath(${ias_knowledge_base_PACKAGE_PATH}/lib/mallet-deps.jar)
add_classpath(${ias_knowledge_base_PACKAGE_PATH}/lib/mallet-dist.jar)
add_classpath(${ias_knowledge_base_PACKAGE_PATH}/lib/weka_fipm.jar)
