cmake_minimum_required(VERSION 2.8)


project(loadlist)

add_subdirectory(src/sttl)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#include( ../CMakePluginTpl.cmake )

#set dependencies to necessary libraries (see qPCV for an example)
add_executable(loadlist src/loadtrans.cpp)
target_link_libraries( ${PROJECT_NAME} STTL )
target_link_libraries (loadlist ${PCL_LIBRARIES})
#include_directories( ${STTL_INCLUDE_DIR} )

get_target_property(sttl_includes STTL INTERFACE_INCLUDE_DIRECTORIES)
INCLUDE_DIRECTORIES ( ${sttl_includes} )
MESSAGE("sttl_includes: ${sttl_includes} ")
