cmake_minimum_required(VERSION 2.8.11)
project(ROPOD_CCU)
set (CMAKE_CXX_STANDARD 11)

find_package(PkgConfig)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

include_directories(include
  /opt/ropod_common/include
)

add_executable(ccu
    src/config/config_file_reader.cpp
    src/ccu_manager.cpp
    src/ccu.cpp
)

target_link_libraries(ccu
  zmq
  jsoncpp
  yaml-cpp
)

target_compile_features(ccu PRIVATE cxx_range_for)
