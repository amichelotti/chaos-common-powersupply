cmake_minimum_required(VERSION 2.6)
project(common_powersupply_ocem)

IF (DEFINED ENV{CHAOS_BUNDLE} )
  include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
ELSE()
  MESSAGE(STATUS "STANDALONE, environment CHAOS_BUNDLE defined")
  IF (NOT CMAKE_INSTALL_PREFIX) 
    MESSAGE(fatal "you must add -DCMAKE_INSTALL_PREFIX=<dir>")
  ELSE()
    MESSAGE(STATUS "install prefix ${CMAKE_INSTALL_PREFIX}")
  ENDIF()
  INCLUDE_DIRECTORIES(. ../../../../)
  LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/build ${CMAKE_INSTALL_PREFIX}/lib)
ENDIF()


#     -DOCEM_CORE_DEBUG
ADD_DEFINITIONS(-DSIMPSUPPLY_DEBUG      -DOCEM_CORE_DEBUG -DPOWER_SUPPLY_DEFAULT_TIMEOUT=10000)

set(power_src OcemE642X.cpp)
IF(FrameworkLib)
SET(libs chaos_common chaos_uitoolkit ${FrameworkLib})
ELSE()
SET(libs boost_program_options boost_thread boost_system boost_regex boost_atomic pthread)
ENDIF()

IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(${PROJECT_NAME} STATIC ${PowerSupply_src} ${power_src})
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${PowerSupply_src} ${power_src})
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_serial_ocem  common_powersupply common_serial common_debug ${libs})

SET(PowerSupplyOcem_test_src test/ocem_test.cpp )

ADD_EXECUTABLE(test_ocem ${PowerSupplyOcem_test_src})
ADD_EXECUTABLE(test_ocem_array test/ocem_array.cpp)

SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(test_ocem ${PROJECT_NAME} common_serial_ocem  common_powersupply common_serial common_debug ${libs})
TARGET_LINK_LIBRARIES(test_ocem_array ${PROJECT_NAME} common_serial_ocem  common_powersupply common_serial common_debug ${libs})

FILE(GLOB ocem_src *.h)    
 

INSTALL(FILES OcemE642X.h DESTINATION include/common/powersupply/models/Ocem ) 
INSTALL_TARGETS(/lib ${PROJECT_NAME})

INSTALL_TARGETS(/bin test_ocem)
INSTALL_TARGETS(/bin test_ocem_array)
