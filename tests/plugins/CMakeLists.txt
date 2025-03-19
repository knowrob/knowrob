
add_library(lpn_reasoner SHARED LPNReasoner.cpp)
target_link_libraries(lpn_reasoner knowrob)
add_dependencies(all_gtests lpn_reasoner)
