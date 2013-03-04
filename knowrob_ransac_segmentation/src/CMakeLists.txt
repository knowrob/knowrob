cmake_minimum_required(VERSION 2.4.6)

include_directories(./)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")


#Get source files
file(GLOB ransac_SOURCES_GfxTL "GfxTL/*.cpp")
file(GLOB ransac_SOURCES_MiscLib "MiscLib/*.cpp")
file(GLOB ransac_SOURCES "*.cpp")
#don't build main into library
file(GLOB ransac_to_remove "ransac.cpp")
list(REMOVE_ITEM ransac_SOURCES ${ransac_to_remove})

#build as ransac_segmentation library
rosbuild_add_library(ransac_segmentation ${ransac_SOURCES} ${ransac_SOURCES_GfxTL} ${ransac_SOURCES_MiscLib})
set_target_properties(ransac_segmentation PROPERTIES COMPILE_FLAGS -w)

#now build ransac main
rosbuild_add_executable(ransac_main ransac.cpp)
target_link_libraries(ransac_main ransac_segmentation)
