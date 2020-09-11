cmake_minimum_required(VERSION 3.12)
project(ActuationDelay VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter Development QUIET)

if(${Python3_FOUND})
    execute_process(COMMAND ${Python3_EXECUTABLE}
        -c "import scenario, pathlib; print(pathlib.Path(scenario.__file__).parent)"
        OUTPUT_VARIABLE Scenario_PREFIX_PATH)
    string(STRIP ${Scenario_PREFIX_PATH} Scenario_PREFIX_PATH)
    list(APPEND CMAKE_PREFIX_PATH ${Scenario_PREFIX_PATH})
endif()

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
