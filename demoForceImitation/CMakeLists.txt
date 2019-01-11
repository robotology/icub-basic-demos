# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(demoForceImitation)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers src/robot_interfaces.h)
set(sources src/robot_interfaces.cpp src/main.cpp)
source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

add_executable(${PROJECT_NAME} ${headers} ${sources})

target_compile_definitions(${PROJECT_NAME} PRIVATE ${GTK_DEFINITIONS})

target_link_libraries(${PROJECT_NAME} ctrlLib
                                      skinDynLib
                                      ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

if(NOT BUILD_BUNDLE)
    icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
