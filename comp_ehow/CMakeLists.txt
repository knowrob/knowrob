cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rosbuild_init()

rosbuild_include(rosprolog utils)

rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(JAVA_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/bin)



#####################################################################
# replace placeholders with global paths in OWL and Prolog files

rosbuild_find_ros_package(opencyc)
rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
rosbuild_find_ros_package(comp_ehow)
set( LOCAL_PACKAGE_PATH ${comp_ehow_PACKAGE_PATH} )

fix_ros_destination_path(OWL_PATH_PREFIX)
fix_ros_destination_path(LOCAL_PACKAGE_PATH)


file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


file(GLOB inputs etc/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


file(GLOB inputs prolog/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )


file(GLOB inputs src/instruction/configuration/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs src/instruction/syntaxparser/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )

file(GLOB inputs src/instruction/wordnet/*.in)
foreach( _in ${inputs} )
  string(REPLACE ".in" "" _out ${_in})
  configure_file( ${_in} ${_out} )
endforeach( _in )




#####################################################################
# class path for compilation time
#
add_classpath(${opencyc_PACKAGE_PATH}/opencyc-2.0/api/java/build/OpenCyc.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/antlr-2.7.5.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/arq-extra.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/arq.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/axis.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/commons-collections.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/commons-discovery.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/commons-lang-2.0.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/commons-logging-1.1.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/concurrent.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/edtftpj-1.5.2.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/edu.mit.jwi_2.1.3.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/ekitspell.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/icu4j_3_4.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/instruction_factory.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/iri.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jakarta-oro-2.0.4.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jaxrpc.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jcalendar.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jdom.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jena.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jep-2.4.0.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/jpl.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/json.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/junit.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/kazuki.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/lax.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.12.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/looks-2.1.3.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/lucene-core-2.3.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/orphanNodesAlg.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi-bin.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/owlsyntax.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/protege-owl.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/protege.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/stanford-parser-2007-08-19.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/stax-api-1.0.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/swrl-jess-bridge.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/unicode_panel.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/vowlidator.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/wstx-asl-3.0.0.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xercesImpl-2.7.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xml-apis-2.7.1.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xml-datatype-validator-20011001.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xmlparserv2.jar)
add_classpath(${PROJECT_SOURCE_DIR}/lib/xschema.jar)


#####################################################################
# class path for generating run-time scripts
#
add_runtime_classpath(${opencyc_PACKAGE_PATH}/opencyc-2.0/api/java/build/OpenCyc.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/antlr-2.7.5.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/arq-extra.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/arq.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/axis.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/commons-collections.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/commons-discovery.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/commons-lang-2.0.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/commons-logging-1.1.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/concurrent.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/edtftpj-1.5.2.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/edu.mit.jwi_2.1.3.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/ekitspell.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/icu4j_3_4.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/instruction_factory.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/iri.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jakarta-oro-2.0.4.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jaxrpc.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jcalendar.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jdom.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jena.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jep-2.4.0.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/jpl.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/json.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/junit.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/kazuki.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/lax.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/log4j-1.2.12.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/looks-2.1.3.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/lucene-core-2.3.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/orphanNodesAlg.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/owlapi-bin.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/owlsyntax.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/protege-owl.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/protege.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/stanford-parser-2007-08-19.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/stax-api-1.0.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/swrl-jess-bridge.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/unicode_panel.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/vowlidator.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/wstx-asl-3.0.0.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xercesImpl-2.7.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xml-apis-2.7.1.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xml-datatype-validator-20011001.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xmlparserv2.jar)
add_runtime_classpath(${PROJECT_SOURCE_DIR}/lib/xschema.jar)

add_java_source_dir(${PROJECT_SOURCE_DIR}/src)

rospack_add_java_executable(ehow_importer instruction.ros.EhowImportService)
