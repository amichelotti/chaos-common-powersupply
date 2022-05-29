cmake_minimum_required(VERSION 2.9)
project(common_powersupply)
find_package(chaos MODULE REQUIRED)
include_directories(${chaos_INCLUDE_DIRS})

ADD_DEFINITIONS(-fPIC)
SET(core_src core/AbstractPowerSupply.cpp)
include_directories(../../ ${CMAKE_INSTALL_PREFIX}/include)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${core_src})
#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
INSTALL(FILES core/AbstractPowerSupply.h DESTINATION include/common/powersupply/core)
INSTALL(FILES powersupply.h DESTINATION include/common/powersupply/) 
add_subdirectory(models)





