cmake_minimum_required(VERSION 3.16)
project(vscp_ws_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(OpenSSL REQUIRED)

add_executable(test_vscp_binary test_vscp_binary.c)

target_compile_definitions(test_vscp_binary PRIVATE _POSIX_C_SOURCE=200809L)
target_compile_options(test_vscp_binary PRIVATE -Wall -Wextra)
target_link_libraries(test_vscp_binary PRIVATE OpenSSL::Crypto)
