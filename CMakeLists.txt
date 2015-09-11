# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
project(icub-basic-demos)
set(BUILD_BUNDLE ON)

option(BUILD_FORCECONTROL "Build Force Control Demo (GTK is required)" OFF)
option(BUILD_FORCEIMITATION "Build Force Imitation Demo" ON)
option(BUILD_REDBALL "Build Red-Ball Demo" ON)
option(BUILD_YOGA "Build Yoga Demo" ON)

find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()

if (BUILD_FORCECONTROL)
    add_subdirectory(demoForceControl)
endif()

if (BUILD_FORCEIMITATION)
    add_subdirectory(demoForceImitation)
endif()

if (BUILD_REDBALL)
    add_subdirectory(demoRedBall)
    add_subdirectory(pf3dTracker)
    add_subdirectory(pf3dBottomup)
endif()

if (BUILD_YOGA)
    add_subdirectory(demoYoga)
endif()

icubcontrib_add_uninstall_target()
