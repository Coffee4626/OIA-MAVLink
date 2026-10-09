cmake_minimum_required(VERSION 3.16)
project(OIA_MAVLink_bridge CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(
    OIA_MAVLink_bridge
    src/drone_bridge.cpp
)

target_include_directories(OIA_MAVLink_bridge PRIVATE
    third_party/mavlink
)

find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)

target_link_libraries(OIA_MAVLink_bridge PRIVATE
    Threads::Threads
    mosquittopp
    nlohmann_json::nlohmann_json
)