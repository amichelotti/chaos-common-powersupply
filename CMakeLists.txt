cmake_minimum_required(VERSION 2.6)
include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)

#   -DOCEM_CORE_DEBUG  -DOCEM_CORE_DEBUG -DOCEM_CORE_DEBUG -DOCEM_CORE_DEBUG
ADD_DEFINITIONS(-DSIMPSUPPLY_DEBUG -DPOWER_SUPPLY_DEFAULT_TIMEOUT=4000)

set(power_src Ocem/OcemE642X.cpp Simulator/SimPSupply.cpp)

IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(common_powersupply STATIC ${PowerSupply_src} ${power_src})
ELSE()
  ADD_LIBRARY(common_powersupply SHARED ${PowerSupply_src} ${power_src})
ENDIF()

TARGET_LINK_LIBRARIES(common_powersupply common_serial common_debug boost_thread boost_system boost_regex)

SET(PowerSupplyOcem_test_src test/ocem_test.cpp )

ADD_EXECUTABLE(test_ocem ${PowerSupplyOcem_test_src})
ADD_EXECUTABLE(test_ocem_array test/ocem_array.cpp)


TARGET_LINK_LIBRARIES(test_ocem common_powersupply common_serial common_debug boost_program_options boost_thread boost_system boost_regex pthread)
TARGET_LINK_LIBRARIES(test_ocem_array common_powersupply common_serial common_debug boost_program_options boost_thread boost_system boost_regex pthread)

FILE(GLOB ocem_src Ocem/*.h)    
INSTALL(FILES core/AbstractPowerSupply.h DESTINATION include/common/powersupply/core) 
INSTALL(FILES powersupply.h DESTINATION include/common/powersupply/) 
INSTALL(FILES Ocem/OcemE642X.h DESTINATION include/common/powersupply/Ocem ) 
INSTALL(FILES Simulator/SimPSupply.h DESTINATION include/common/powersupply/Simulator ) 

INSTALL_TARGETS(/lib common_powersupply)

INSTALL_TARGETS(/bin test_ocem)
INSTALL_TARGETS(/bin test_ocem_array)
