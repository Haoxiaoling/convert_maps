cmake_minimum_required(VERSION 2.8)

# CloudCompare 'Transforms' plugin
    project( STTL )
    
    set( CMAKE_AUTOMOC OFF )

  if (APPLE)

    find_package( JsonCpp REQUIRED )

	SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
	IF( NOT EIGEN3_INCLUDE_DIR )
	    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
	ENDIF()
	INCLUDE_DIRECTORIES ( ${EIGEN3_INCLUDE_DIR} )
    MESSAGE("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR} ")

  else()

    find_package(Eigen3 REQUIRED)
    

# pkg-config jsoncpp --libs --libs

    include_directories(${EIGEN3_INCLUDE_DIR})

    SET(JsonCpp_includes "/usr/include/jsoncpp")
    
    MESSAGE("JsonCpp_includes: ${JsonCpp_includes} ")
    MESSAGE("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR} ")

    SET(jsoncpp_lib_static "jsoncpp" )
    
#    /lib/x86_64-linux-gnu/libjsoncpp.a
    
    SET(CMAKE_CXX_FLAGS "-std=c++11 -fPIC" )
    
  endif() 


    include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )


    file( GLOB header_list *.h )
    file( GLOB source_list *.cpp )

    add_library( ${PROJECT_NAME} STATIC ${header_list} ${source_list} )

    #target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )

  if (APPLE)
    get_target_property(JsonCpp_includes jsoncpp_lib_static  INTERFACE_INCLUDE_DIRECTORIES)
    set(jsoncpp_lib_static "jsoncpp_lib_static")
  endif()
  
  
    INCLUDE_DIRECTORIES ( ${JsonCpp_includes} )
    
    set_target_properties(${PROJECT_NAME} PROPERTIES 
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR};${CMAKE_CURRENT_SOURCE_DIR}"
    )

    #set dependencies to necessary libraries (see qPCV for an example)
    target_link_libraries( ${PROJECT_NAME} ${jsoncpp_lib_static} )
    #include_directories( ${LIB1_INCLUDE_DIR} )
