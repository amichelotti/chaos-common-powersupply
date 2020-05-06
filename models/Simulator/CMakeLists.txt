cmake_minimum_required(VERSION 2.6)
project(common_powersupply_simulator)
#     -DOCEM_CORE_DEBUG
ADD_DEFINITIONS(-DSIMPSUPPLY_DEBUG -DPOWER_SUPPLY_DEFAULT_TIMEOUT=4000)

set(power_src SimPSupply.cpp)
IF(FrameworkLib)
SET(libs chaos_common ${FrameworkLib})
ELSE()
SET(libs boost_program_options boost_thread boost_system boost_regex boost_atomic pthread)
ENDIF()

ADD_LIBRARY(${PROJECT_NAME} STATIC ${PowerSupply_src} ${power_src})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common_powersupply common_debug ${libs})


SET(DRIVERS_LIB ${DRIVERS_LIB} ${PROJECT_NAME} PARENT_SCOPE)

FILE(GLOB src *.h)    
 
INSTALL(FILES ${src} DESTINATION include/common/powersupply/models/Simulator) 
#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

