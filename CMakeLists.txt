cmake_minimum_required(VERSION 2.6)
option(BUILD_FORCE_32 "Set to ON to enable 32 bit compilation" OFF)


IF( (DEFINED ENV{CHAOS32}) OR (BUILD_FORCE_32) )
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
#   -DOCEM_CORE_DEBUG  -DOCEM_CORE_DEBUG -DOCEM_CORE_DEBUG -DOCEM_CORE_DEBUG
ADD_DEFINITIONS(-g  -O2  -Wall -DPOWER_SUPPLY_DEFAULT_TIMEOUT=4000)

set(power_src Ocem/OcemE642X.cpp Simulator/SimPSupply.cpp)

#### put your libcommon_PowerSupply LIBRARY files here
###

ADD_LIBRARY(common_powersupply SHARED ${PowerSupply_src} ${power_src})
TARGET_LINK_LIBRARIES(common_powersupply common_serial common_debug boost_thread boost_system boost_regex)

SET(PowerSupplyOcem_test_src test/ocem_test.cpp )
###
ADD_EXECUTABLE(test_ocem ${PowerSupplyOcem_test_src})
ADD_EXECUTABLE(test_ocem_array test/ocem_array.cpp)


TARGET_LINK_LIBRARIES(test_ocem common_powersupply common_serial common_debug boost_program_options boost_thread boost_system boost_regex)
TARGET_LINK_LIBRARIES(test_ocem_array common_powersupply common_serial common_debug boost_program_options boost_thread boost_system boost_regex)

FILE(GLOB ocem_src Ocem/*.h)    
INSTALL(FILES core/AbstractPowerSupply.h DESTINATION include/common/powersupply/core) 
INSTALL(FILES powersupply.h DESTINATION include/common/powersupply/) 
INSTALL(FILES Ocem/OcemE642X.h DESTINATION include/common/powersupply/Ocem ) 
INSTALL(FILES Simulator/SimPSupply.h DESTINATION include/common/powersupply/Simulator ) 

INSTALL_TARGETS(/lib common_powersupply)
INSTALL_TARGETS(/bin test_ocem)
INSTALL_TARGETS(/bin test_ocem_array)
