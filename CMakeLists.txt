cmake_minimum_required(VERSION 2.8)
option(BUILD_FORCE_32 "Set to ON to enable 32 bit compilation" OFF)


IF( ($ENV{CHAOS32}) OR (BUILD_FORCE_32) )
    MESSAGE(STATUS "Enabling 32 bit Compilation")
    set (CMAKE_C_FLAGS "-m32")
    set (CMAKE_CXX_FLAGS "-m32")
    set (CMAKE_LINK_FLAGS "-m32")
ENDIF()

if( DEFINED ENV{CHAOS_FRAMEWORK} ) 
    MESSAGE(STATUS "Using $ENV{CHAOS_FRAMEWORK}")
      set (PROJECT_SOURCE_DIR $ENV{CHAOS_FRAMEWORK})
ELSE()
    MESSAGE(FATAL_ERROR "Please set CHAOS_FRAMEWORK environment")
ENDIF()

set (CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/usr/local)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)
     
INCLUDE_DIRECTORIES(. ../../  ${PROJECT_SOURCE_DIR}/usr/local/include)
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/usr/local/lib)

ADD_DEFINITIONS(-g -O2 -Wall -DDEBUG)

set(Ocem_src Ocem/OcemCore.cpp)
#### put your libcommon_PowerSupply LIBRARY files here
###

ADD_LIBRARY(common_powersupply SHARED ${PowerSupply_src} ${Ocem_src})
TARGET_LINK_LIBRARIES(common_powersupply common_serial common_debug)

SET(PowerSupplyOcem_test_src test/ocem_test.cpp )
###
ADD_EXECUTABLE(test_Ocem ${PowerSupplyOcem_test_src})


TARGET_LINK_LIBRARIES(test_Ocem common_powersupply common_serial common_debug boost_program_options)

FILE(GLOB ocem_src Ocem/*.h)    
INSTALL(FILES core/AbstractPowerSupply.h DESTINATION include/common/powersupply/core) 
INSTALL(FILES powersupply.h DESTINATION include/common/powersupply/) 
INSTALL(FILES ${ocem_src} DESTINATION include/common/powersupply/Ocem ) 

INSTALL_TARGETS(/bin common_powersupply)
INSTALL_TARGETS(/bin test_Ocem)
