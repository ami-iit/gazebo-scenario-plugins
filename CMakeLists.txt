cmake_minimum_required(VERSION 3.12)
project(gazebo-scenario-plugins VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(plugins/actuation_delay)
add_subdirectory(plugins/low_pass_target)
