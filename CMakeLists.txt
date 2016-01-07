cmake_minimum_required(VERSION 2.6)
project(common_powersupply)
IF (DEFINED ENV{CHAOS_BUNDLE} )
include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
add_definitions(-DCHAOS)
ENDIF()

SET(core_src core/AbstractPowerSupply.cpp)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})
INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(FILES core/AbstractPowerSupply.h DESTINATION include/common/powersupply/core)
INSTALL(FILES powersupply.h DESTINATION include/common/powersupply/) 
add_subdirectory(models)





