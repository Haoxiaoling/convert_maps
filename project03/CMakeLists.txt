cmake_minimum_required(VERSION 2.8)


project(graphviz)

add_subdirectory(src/sttl)


#include( ../CMakePluginTpl.cmake )

#set dependencies to necessary libraries (see qPCV for an example)
add_executable(graphviz src/graphviz.cpp)
target_link_libraries( ${PROJECT_NAME} STTL )
#include_directories( ${STTL_INCLUDE_DIR} )

get_target_property(sttl_includes STTL INTERFACE_INCLUDE_DIRECTORIES)
INCLUDE_DIRECTORIES ( ${sttl_includes} )
MESSAGE("sttl_includes: ${sttl_includes} ")
