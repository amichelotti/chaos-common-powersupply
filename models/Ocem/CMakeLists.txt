cmake_minimum_required(VERSION 2.6)
project(common_powersupply_ocem)

#     -DOCEM_CORE_DEBUG
ADD_DEFINITIONS(-DPOWER_SUPPLY_DEFAULT_TIMEOUT=10000)

set(power_src OcemE642X.cpp)
IF(FrameworkLib)
SET(libs chaos_uitoolkit chaos_common ${FrameworkLib})
ELSE()
SET(libs boost_program_options boost_thread boost_system boost_regex boost_atomic pthread)
ENDIF()

ADD_LIBRARY(${PROJECT_NAME} STATIC ${PowerSupply_src} ${power_src})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_serial_ocem  common_powersupply common_debug ${libs})

SET(PowerSupplyOcem_test_src test/ocem_test.cpp )

ADD_EXECUTABLE(test_ocem ${PowerSupplyOcem_test_src})
# ADD_EXECUTABLE(test_ocem_array test/ocem_array.cpp)

SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)
TARGET_LINK_LIBRARIES(test_ocem ${PROJECT_NAME} common_serial_ocem  common_powersupply common_debug ${libs})
# TARGET_LINK_LIBRARIES(test_ocem_array ${PROJECT_NAME} common_serial_ocem  common_powersupply common_serial common_debug ${libs})

FILE(GLOB ocem_src *.h)    
 

INSTALL(FILES OcemE642X.h DESTINATION include/common/powersupply/models/Ocem ) 
INSTALL_TARGETS(/lib ${PROJECT_NAME})

INSTALL_TARGETS(/bin test_ocem)
# INSTALL_TARGETS(/bin test_ocem_array)
