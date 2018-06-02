# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME demoForceImitation)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers src/robot_interfaces.h)
set(sources src/robot_interfaces.cpp src/main.cpp)
source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

add_definitions(${GTK_DEFINITIONS})

include_directories(${iCubDev_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${skinDynLib_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${headers} ${sources})

target_link_libraries(${PROJECTNAME} ctrlLib
                                     skinDynLib
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

if(NOT BUILD_BUNDLE)
    icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
