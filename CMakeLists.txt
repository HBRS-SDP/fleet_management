cmake_minimum_required(VERSION 2.8.11)
project(ROPOD_CCU)
set (CMAKE_CXX_STANDARD 11)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})


########################################################################
# LIBZMQ dependency
########################################################################
find_package(libzmq REQUIRED)
IF (LIBZMQ_FOUND)
    include_directories(${LIBZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${LIBZMQ_LIBRARIES})
ELSE (LIBZMQ_FOUND)
    message( FATAL_ERROR "libzmq not found." )
ENDIF (LIBZMQ_FOUND)

########################################################################
# CZMQ dependency
########################################################################
find_package(czmq REQUIRED)
IF (CZMQ_FOUND)
    include_directories(${CZMQ_INCLUDE_DIRS})
    list(APPEND LIBS ${CZMQ_LIBRARIES})
ELSE (CZMQ_FOUND)
    message( FATAL_ERROR "czmq not found." )
ENDIF (CZMQ_FOUND)

########################################################################
# Zyre dependency
########################################################################
find_package(zyre REQUIRED)
IF (ZYRE_FOUND)
    include_directories(${ZYRE_INCLUDE_DIRS})
    list(APPEND LIBS ${ZYRE_LIBRARIES})
ELSE (ZYRE_FOUND)
    message( FATAL_ERROR "zyre not found." )
ENDIF (ZYRE_FOUND)

file(GLOB ZYRE_WRAPPER_SRC src/extern/zyre/*.cpp)

include_directories(include
  /opt/ropod_common/include
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(ccu_cli
    src/config/config_file_reader.cpp
    ${ZYRE_WRAPPER_SRC}
    src/ccu_manager.cpp
    src/ccu_cli.cpp
)

target_link_libraries(ccu_cli
  zmq
  jsoncpp
  yaml-cpp
  ${LIBS}
)

##################
# Required for Qt
##################
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

message(STATUS "Checking for module 'Qt5Widgets'")
find_package(Qt5 COMPONENTS Widgets REQUIRED)
if (Qt5Widgets_FOUND)
    message(STATUS "  Found Qt5Widgets, version ${Qt5Widgets_VERSION}")

    add_executable(ccu_gui
      src/config/config_file_reader.cpp
      src/ccu_manager_gui.cpp
      ${ZYRE_WRAPPER_SRC}
      window.cpp
      ccu_gui.cpp
    )

    target_link_libraries(ccu_gui
      zmq
      jsoncpp
      yaml-cpp
      ${LIBS}
      Qt5::Widgets
    )
else (Qt5Widgets_FOUND)
    message(STATUS "  Qt5Widgets not found; the GUI will not be built")
endif(Qt5Widgets_FOUND)

target_compile_features(ccu_cli PRIVATE cxx_range_for)
