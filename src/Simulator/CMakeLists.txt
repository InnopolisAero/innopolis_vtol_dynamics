# Copyright (C) 2024 Dmitry Ponomarev <ponomarevda96@gmail.com>
# Distributed under the terms of the GPL v3 license, available in the file LICENSE.

cmake_minimum_required(VERSION 3.22)
project(UavDynamics VERSION 1.0 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include(../DronecanInterface/dronecan_interface.cmake)
add_definitions(-DLIBPARAMS_PARAMS_DIR="${CMAKE_CURRENT_BINARY_DIR}")

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ../UavDynamics/include
    ../MavlinkSimInterface/include
    ../mavlink_c_library_v2/
    ../RosInterface/include
    ../websocketpp
    ../nlohmann_json/include
    ../ArdupilotInterface/include
    ${DRONECAN_INTERFACE_HEADERS}
)

add_executable(UavDynamics
    src/main.cpp
    ../MavlinkSimInterface/src/mavlink_sim_interface.cpp
    ${DRONECAN_INTERFACE_SOURCES}
)


target_link_libraries(UavDynamics PRIVATE
    yaml-cpp
)

install(TARGETS UavDynamics DESTINATION usr/bin)
# install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION usr/include)
