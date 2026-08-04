cmake_minimum_required(VERSION 3.16)
project(p1parser C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Build as a static library so it's easy to link into an embedded
# firmware project (STM32CubeIDE / Makefile / other CMake project)
# via add_subdirectory() + target_link_libraries().
add_library(p1parser STATIC
    p1_parser.c
)

target_include_directories(p1parser PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_options(p1parser PRIVATE
    -Wall -Wextra -Wpedantic
)

# Optional host-side demo, built by default when this is the top-level
# project (i.e. not when pulled in via add_subdirectory() from a
# firmware project that doesn't want the demo built).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(P1PARSER_IS_TOP_LEVEL ON)
else()
    set(P1PARSER_IS_TOP_LEVEL OFF)
endif()

option(P1PARSER_BUILD_EXAMPLE "Build the example_main demo executable" ${P1PARSER_IS_TOP_LEVEL})

if(P1PARSER_BUILD_EXAMPLE)
    add_executable(p1demo example_main.c)
    target_link_libraries(p1demo PRIVATE p1parser)
endif()
