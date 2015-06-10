# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME demoYoga)
project(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})

include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(sources src/main.cpp)
source_group("Source Files" FILES ${sources})

include_directories(${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${sources})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

if(NOT BUILD_BUNDLE)
    icubcontrib_add_uninstall_target()
endif()

# app
file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
file(GLOB yoga      ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/yoga/*.ini)
file(GLOB crf       ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/crf/*.ini)
file(GLOB hannover  ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/hannover/*.ini)
file(GLOB two-hands ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/two-hands/*.ini)

yarp_install(FILES ${conf}      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})
yarp_install(FILES ${yoga}      DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/yoga)
yarp_install(FILES ${crf}       DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/crf)
yarp_install(FILES ${hannover}  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/hannover)
yarp_install(FILES ${two-hands} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/two-hands)
