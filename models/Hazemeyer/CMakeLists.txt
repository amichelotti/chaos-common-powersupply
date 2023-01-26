cmake_minimum_required(VERSION 3.0)
project(hazemeyer_powersupply)
ADD_DEFINITIONS(-DDEBUG -fPIC)

IF(FrameworkLib)
SET(libs chaos_common ${FrameworkLib})
ELSE()
SET(libs pthread boost_system  )
ENDIF()


ADD_LIBRARY(${PROJECT_NAME} STATIC ./AL250PowerSupply.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} HZLib  common_powersupply common_debug ${libs})

###
ADD_EXECUTABLE(A250Test test.cpp)
TARGET_LINK_LIBRARIES(A250Test ${MODBUSLIB} ${PROJECT_NAME} HZLib common_powersupply common_modbus modbus common_debug ${libs} )
#INSTALL_TARGETS(/lib ${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME}
        DESTINATION "lib"
        PERMISSIONS OWNER_WRITE OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)