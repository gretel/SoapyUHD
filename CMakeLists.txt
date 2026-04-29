# Copyright 2012-2015 Free Software Foundation, Inc.
# Copyright 2015-2016 Josh Blum
#
# This file is part of SoapyUHD support modules
#
# SoapyUHD is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# SoapyUHD is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.10...3.30)
project(SoapyUHD CXX C)
enable_testing()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_UHD_SUPPORT   "Build the SoapySDR-side module exposing UHD devices to SoapySDR"   ON)
option(BUILD_SOAPY_SUPPORT "Build the UHD-side module exposing Soapy devices to UHD"          ON)

if(NOT BUILD_UHD_SUPPORT AND NOT BUILD_SOAPY_SUPPORT)
    message(FATAL_ERROR "Both BUILD_UHD_SUPPORT and BUILD_SOAPY_SUPPORT are OFF -- nothing to build.")
endif()

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

########################################################################
# Dependencies
########################################################################
find_package(SoapySDR "0.7" NO_MODULE REQUIRED)
find_package(UHD 4.0 NO_MODULE REQUIRED)

if (NOT UHD_ROOT)
    get_filename_component(UHD_ROOT "${UHD_INCLUDE_DIRS}/.." ABSOLUTE)
endif()
message(STATUS "UHD root directory: ${UHD_ROOT}")
message(STATUS "UHD include directories: ${UHD_INCLUDE_DIRS}")
message(STATUS "UHD libraries: ${UHD_LIBRARIES}")
message(STATUS "UHD version: ${UHD_VERSION}")

set(UHD_FEATURE_DEFS "")

#
# UHD's public headers transitively #include <boost/format.hpp> and
# others from its own .ipp files. UHDConfig.cmake's find_dependency(Boost)
# populates Boost_INCLUDE_DIRS as a side effect; pull it into our
# include paths so the UHD headers compile.
#
set(SOAPY_UHD_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SoapySDR_INCLUDE_DIRS}
    ${UHD_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

########################################################################
# Build a Soapy module to support UHD devices
########################################################################
if(BUILD_UHD_SUPPORT)
    SOAPY_SDR_MODULE_UTIL(
        TARGET uhdSupport
        SOURCES SoapyUHDDevice.cpp
        LIBRARIES
            ${UHD_LIBRARIES}
    )
    target_include_directories(uhdSupport PRIVATE ${SOAPY_UHD_INCLUDE_DIRS})
    target_compile_definitions(uhdSupport PRIVATE ${UHD_FEATURE_DEFS})
endif()

########################################################################
# Build a UHD module to support Soapy devices
########################################################################
if(BUILD_SOAPY_SUPPORT)
    add_library(soapySupport MODULE UHDSoapyDevice.cpp)
    target_include_directories(soapySupport PRIVATE ${SOAPY_UHD_INCLUDE_DIRS})
    target_compile_definitions(soapySupport PRIVATE ${UHD_FEATURE_DEFS})
    target_link_libraries(soapySupport
        ${UHD_LIBRARIES}
        ${SoapySDR_LIBRARIES})
    install(TARGETS soapySupport
        DESTINATION ${UHD_ROOT}/lib${LIB_SUFFIX}/uhd/modules
    )
endif()

########################################################################
# rpath setup - http://www.cmake.org/Wiki/CMake_RPATH_handling
########################################################################
# use, i.e. don't skip the full RPATH for the build tree
option(CMAKE_SKIP_BUILD_RPATH "skip rpath build" FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
option(CMAKE_BUILD_WITH_INSTALL_RPATH "build with install rpath" FALSE)

# the RPATH to be used when installing, but only if it's not a system directory
option(CMAKE_AUTOSET_INSTALL_RPATH TRUE)
if(CMAKE_AUTOSET_INSTALL_RPATH)
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")
ENDIF("${isSystemDir}" STREQUAL "-1")
endif(CMAKE_AUTOSET_INSTALL_RPATH)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
option(CMAKE_INSTALL_RPATH_USE_LINK_PATH "build with automatic rpath" TRUE)

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif()

########################################################################
# Print Summary
########################################################################
MESSAGE(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")

########################################################################
# uninstall target
########################################################################
add_custom_target(uninstall
    "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

