cmake_minimum_required(VERSION 2.8.11)
project(ROPOD_CCU)
set (CMAKE_CXX_STANDARD 11)

find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)

include_directories(include)

add_executable(ccu
    src/config/config_file_reader.cpp
    src/ccu_manager.cpp
    src/ccu.cpp
)

target_link_libraries(ccu zmq jsoncpp yaml-cpp)
