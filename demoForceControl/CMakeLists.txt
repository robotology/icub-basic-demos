# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME demoForceControl)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(GTK2 COMPONENTS gtk gtkmm REQUIRED)
if(WIN32 AND GTK2_FOUND)
    list(REMOVE_ITEM GTK2_LIBRARIES ${FREETYPE_LIBRARY} ${GTK2_PANGOXFT_LIBRARY})
endif()

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers src/robot_interfaces.h)
set(sources src/robot_interfaces.cpp src/main.cpp)
source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

add_definitions(${GTK_DEFINITIONS})

include_directories(${iCubDev_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    ${GtkPlus_INCLUDE_DIRS}
                    ${GTK2_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${headers} ${sources})

target_link_libraries(${PROJECTNAME} ctrlLib
                                     ${GtkPlus_LIBRARIES}
                                     ${GTK2_LIBRARIES}
                                     ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

icubcontrib_add_uninstall_target()

# app
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

