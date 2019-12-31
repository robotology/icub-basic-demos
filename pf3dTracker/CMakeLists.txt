# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(pf3dTracker)

find_package(ICUBcontrib REQUIRED)
find_package(YARP 3.3.0 REQUIRED COMPONENTS os sig cv)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
find_package(OpenCV REQUIRED)

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/iCub/*.hpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

if(NOT BUILD_BUNDLE)
  icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini
               ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.txt
               ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.csv)
file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/models/*.bmp
                 ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/models/*.csv
                 ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/models/*.py)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.template)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${models}  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/models)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
