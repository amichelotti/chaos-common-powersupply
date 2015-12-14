cmake_minimum_required(VERSION 2.6)
project(hazemeyer_powersupply)
IF (DEFINED ENV{CHAOS_BUNDLE} )
  include($ENV{CHAOS_BUNDLE}/tools/project_template/CMakeChaos.txt)
ELSE()
  MESSAGE(STATUS "STANDALONE, environment CHAOS_BUNDLE defined")
  IF (NOT CMAKE_INSTALL_PREFIX) 
    MESSAGE(fatal "you must add -DCMAKE_INSTALL_PREFIX=<dir>")
  ELSE()
    MESSAGE(STATUS "install prefix ${CMAKE_INSTALL_PREFIX}")
  ENDIF()
  INCLUDE_DIRECTORIES(. ../..)
  LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/build ${CMAKE_INSTALL_PREFIX}/lib)
ENDIF()



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

TARGET_LINK_LIBRARIES(${PROJECT_NAME} HZLib  ${libs})

###
ADD_EXECUTABLE(A250Test test.cpp)
TARGET_LINK_LIBRARIES(A250Test ${MODBUSLIB} ${PROJECT_NAME} HZLib common_modbus modbus ${libs} )
INSTALL_TARGETS(/lib ${PROJECT_NAME})