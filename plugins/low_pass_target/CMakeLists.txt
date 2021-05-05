cmake_minimum_required(VERSION 3.12)
project(LowPassTarget VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use -fPIC even if statically compiled
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Scenario COMPONENTS Gazebo REQUIRED)

# ==========
# Dependency
# ==========

find_package(iir QUIET)

option(SGP_USE_SYSTEM_IIR
    "Use system-installed IIR rather than a private copy"
    ${iir_FOUND})

if(SGP_USE_SYSTEM_IIR AND NOT ${iir_FOUND})
    message(FATAL_ERROR "Failed to find system iir")
endif()

if(NOT ${SGP_USE_SYSTEM_IIR})

    include(FetchContent)

    FetchContent_Declare(
        iir
        GIT_REPOSITORY https://github.com/berndporr/iir1)

    FetchContent_GetProperties(iir)

    if(NOT iir_POPULATED)
        FetchContent_Populate(iir)

        add_subdirectory(${iir_SOURCE_DIR}
                         ${iir_BINARY_DIR}
                         EXCLUDE_FROM_ALL)
    endif()

endif()

# =============
# LowPassTarget
# =============

add_library(LowPassTarget SHARED
    LowPassTarget.h
    LowPassTarget.cpp)

target_link_libraries(LowPassTarget
    PRIVATE
    iir::iir_static
    ScenarioGazebo::ScenarioGazebo)

target_include_directories(LowPassTarget PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

install(
    TARGETS LowPassTarget
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
