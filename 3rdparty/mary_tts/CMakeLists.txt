cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)



# set( MOD_DIALOG_PATH ${PROJECT_SOURCE_DIR} )
# 
# file(GLOB inputs src/edu/tum/cs/ias/knowrob/mod_dialog/*.in)
# 
# foreach( _in ${inputs} )
#   string(REPLACE ".in" "" _out ${_in})
#   configure_file( ${_in} ${_out} )
# endforeach( _in )

# copy config file to binary tree
# configure_file(${PROJECT_SOURCE_DIR}/src/edu/tum/cs/ias/knowrob/mod_dialog/jsgf.config.xml ${PROJECT_SOURCE_DIR}/bin/edu/tum/cs/ias/knowrob/mod_dialog/jsgf.config.xml COPYONLY)

add_classpath(${PROJECT_SOURCE_DIR}/bin)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/mary/java)
add_jar_dir(build)
add_jar_dir(lib)
add_jar_dir(mary/java)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(MaryROSClient edu.tum.cs.ias.mary_tts.MaryROSClient)

# rospack_add_java_executable(SpeechRecognitionNode edu.tum.cs.ias.knowrob.mod_dialog.SpeechRecognitionNode)
