# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# PROJECT_VER must be defined before project() so esp_app_desc.version uses it.
file(READ "${CMAKE_SOURCE_DIR}/build_counter.txt" BUILD_COUNTER_RAW)
string(STRIP "${BUILD_COUNTER_RAW}" BUILD_COUNTER)
if("${BUILD_COUNTER}" STREQUAL "")
    set(BUILD_COUNTER 0)
endif()
math(EXPR BUILD_COUNTER_NEXT "${BUILD_COUNTER} + 1")
set(PROJECT_VER "0.0.1.${BUILD_COUNTER_NEXT}")

# ---------------------------------------------------------------------------
# External dependency: vscp-firmware (common VSCP firmware sources)
#
# Fetched at configure time into ${CMAKE_BINARY_DIR}/_deps/vscp_firmware-src.
# To build against a local checkout instead, configure with
#   -DFETCHCONTENT_SOURCE_DIR_VSCP_FIRMWARE=/path/to/vscp-firmware
# ---------------------------------------------------------------------------
include(FetchContent)

set(VSCP_FIRMWARE_GIT_TAG "bdee7be0f86b4154a45b0cf025d9c5b482f25d50"
    CACHE STRING "vscp-firmware git tag/branch/commit to fetch")

FetchContent_Declare(
    vscp_firmware
    GIT_REPOSITORY https://github.com/grodansparadis/vscp-firmware.git
    GIT_TAG        ${VSCP_FIRMWARE_GIT_TAG}
    # Sources are compiled by the main component; don't add_subdirectory() it.
    SOURCE_SUBDIR  _no_cmake_
)
FetchContent_MakeAvailable(vscp_firmware)

# Used by main/CMakeLists.txt
set(VSCP_FIRMWARE_DIR "${vscp_firmware_SOURCE_DIR}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(vscp_smart_p1)

set(BUILD_COUNTER_HEADER ${CMAKE_BINARY_DIR}/build_counter.h)

add_custom_command(
    OUTPUT ${BUILD_COUNTER_HEADER}
    COMMAND ${CMAKE_COMMAND} -E env python ${CMAKE_SOURCE_DIR}/tools/increment_build.py
    DEPENDS ${CMAKE_SOURCE_DIR}/build_counter.txt
    COMMENT "Incrementing build counter"
)

add_custom_target(build_counter ALL DEPENDS ${BUILD_COUNTER_HEADER})

add_dependencies(${PROJECT_NAME}.elf build_counter)

include_directories(${CMAKE_BINARY_DIR})
