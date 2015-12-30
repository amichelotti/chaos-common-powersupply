cmake_minimum_required(VERSION 2.6)
project(powersupply_models)
add_subdirectory(Ocem)
add_subdirectory(Hazemeyer)
add_subdirectory(Simulator)

SET(DRIVERS_LIB ${DRIVERS_LIB} PARENT_SCOPE)
