cmake_minimum_required(VERSION 2.6)
project(hazemeyer_powersupply)
ADD_DEFINITIONS(-DDEBUG)

IF(FrameworkLib)
SET(libs chaos_common ${FrameworkLib})
ELSE()
SET(libs pthread boost_system boost_regex )
ENDIF()

IF(BUILD_FORCE_STATIC)
  ADD_LIBRARY(${PROJECT_NAME} STATIC ./AL250PowerSupply.cpp)
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ./AL250PowerSupply.cpp)
ENDIF()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} HZLib  common_powersupply common_debug ${libs})

###
ADD_EXECUTABLE(A250Test test.cpp)
TARGET_LINK_LIBRARIES(A250Test ${MODBUSLIB} ${PROJECT_NAME} HZLib common_powersupply common_modbus modbus common_debug ${libs} )
INSTALL_TARGETS(/lib ${PROJECT_NAME})