# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(demoRedBall)

find_package(YARP 3.5.1 REQUIRED)
find_package(ICUB 2.0.0 REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(sources src/main.cpp)
source_group("Source Files" FILES ${sources})

add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} ctrlLib iKin ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# world
find_package(GAZEBO QUIET)
if (GAZEBO_FOUND)
    add_library(${PROJECT_NAME}-world SHARED src/world.cpp)
    target_compile_definitions(${PROJECT_NAME}-world PRIVATE _USE_MATH_DEFINES)
    target_include_directories(${PROJECT_NAME}-world PRIVATE ${GAZEBO_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME}-world PRIVATE ${GAZEBO_LIBRARY_DIRS})
    target_link_libraries(${PROJECT_NAME}-world PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES})
    install(TARGETS ${PROJECT_NAME}-world LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)
    add_subdirectory(gazebo)
endif()

if(NOT BUILD_BUNDLE)
    icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.m)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

