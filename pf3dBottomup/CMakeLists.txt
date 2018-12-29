# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(pf3dBottomup)

find_package(ICUBcontrib REQUIRED)
find_package(YARP 3.1.100 REQUIRED COMPONENTS OS sig cv)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
find_package(OpenCV REQUIRED)

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

set(folder_source src/main.cpp
                  src/cvfloodfill2.cpp
                  src/pf3dBottomup.cpp
                  src/ScaleSpace.cpp
                  src/FastGauss.cpp
                  src/IIRFilt.cpp
                  src/IIRGausDeriv.cpp)
set(folder_header include/iCub/FastGauss.h
                  include/iCub/ScaleSpace.h
                  include/iCub/IIRFilt.h
                  include/iCub/IIRGausDeriv.h
                  include/iCub/pf3dBottomup.hpp)

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
file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/*.ini)
file(GLOB models ${CMAKE_CURRENT_SOURCE_DIR}/app/conf/models/*.bmp)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME})
yarp_install(FILES ${models}  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECTNAME}/models)
