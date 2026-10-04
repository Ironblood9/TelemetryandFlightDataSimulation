cmake_minimum_required(VERSION 3.28)

project(TelemetryAndFlightDataSimulation
        VERSION 0.1.0
        DESCRIPTION "Real-time flight dynamics and telemetry simulation with a WPF ground station"
        HOMEPAGE_URL "https://github.com/<owner>/TelemetryandFlightDataSimulation"
        LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Global build settings
# ---------------------------------------------------------------------------
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (single-config generators)" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ProjectOptions)

# `enable_testing()` must run in the top-level directory before any
# `add_test()`/`gtest_add_tests()` call so that the top level CTestTestfile.cmake
# pulls the subdirectories in.
if(FSE_BUILD_TESTS)
    enable_testing()
endif()

# ---------------------------------------------------------------------------
# Components
# ---------------------------------------------------------------------------
add_subdirectory(FlightSimulationEngine)