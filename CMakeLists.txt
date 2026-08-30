cmake_minimum_required(VERSION 3.16)
project(fuzzylib
    VERSION 0.1.0
    DESCRIPTION "Modular C++ fuzzy logic library (Mamdani -> TSK/Sugeno) blendable with PID, GA, NN, and HDC"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(FUZZYLIB_BUILD_EXAMPLES "Build example applications" ON)
option(FUZZYLIB_BUILD_TESTS "Build unit tests" ON)
option(FUZZYLIB_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

add_library(fuzzylib STATIC
    src/inference/mamdani.cpp
    src/inference/sugeno.cpp
    src/ga/genetic_algorithm.cpp
    src/nn/feedforward.cpp
    src/hdc/hypervector.cpp
)
add_library(fuzzylib::fuzzylib ALIAS fuzzylib)

target_include_directories(fuzzylib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_options(fuzzylib PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>
    $<$<AND:$<BOOL:${FUZZYLIB_WARNINGS_AS_ERRORS}>,$<CXX_COMPILER_ID:GNU,Clang,AppleClang>>:-Werror>
)

if(FUZZYLIB_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(FUZZYLIB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
