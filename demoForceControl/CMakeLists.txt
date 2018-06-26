# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(demoForceControl)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

find_package(Qt5 COMPONENTS Core Widgets Gui REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

file(GLOB demoForceControl_SRCS src/*.cpp)
file(GLOB demoForceControl_HDRS src/*.h)
file(GLOB demoForceControl_QRC_FILES src/*.qrc)
file(GLOB demoForceControl_UI_FILES src/*.ui)
  
qt5_add_resources(demoForceControl_QRC_GEN_SRCS ${demoForceControl_QRC_FILES})
qt5_wrap_ui(demoForceControl_UI_GEN_SRCS ${demoForceControl_UI_FILES})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(headers src/robot_interfaces.h src/mainwindow.h)
set(sources src/robot_interfaces.cpp src/main.cpp src/mainwindow.cpp)
source_group("Header Files" FILES ${headers})
source_group("Source Files" FILES ${sources})

add_executable(${PROJECT_NAME} WIN32 ${headers} ${sources} ${demoForceControl_UI_GEN_SRCS})

target_link_libraries(${PROJECT_NAME} ctrlLib
                                      ${YARP_LIBRARIES}
                                      Qt5::Core
                                      Qt5::Widgets
                                      Qt5::Gui)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

if(NOT BUILD_BUNDLE)
  icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
