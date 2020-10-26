cmake_minimum_required(VERSION 3.12)
project(ActuationDelay VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Scenario COMPONENTS Gazebo REQUIRED)

# ==============
# ActuationDelay
# ==============

add_library(ActuationDelay SHARED
    ActuationDelay.h
    ActuationDelay.cpp)

target_link_libraries(ActuationDelay
    PRIVATE
    ScenarioGazebo::ScenarioGazebo)

target_include_directories(ActuationDelay PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

install(
    TARGETS ActuationDelay
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
