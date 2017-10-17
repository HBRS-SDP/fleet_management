cmake_minimum_required(VERSION 2.8.11)
project(ROPOD_CCU)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)

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

include_directories(include
  /opt/ropod_common/include
)

add_executable(ccu
    src/config/config_file_reader.cpp
    src/extern/zyre/exception.cpp
    src/extern/zyre/event.cpp
    src/extern/zyre/node.cpp
    src/ccu_manager.cpp
    src/ccu.cpp
)

target_link_libraries(ccu
  zmq
  jsoncpp
  yaml-cpp
  ${LIBS}
)

target_compile_features(ccu PRIVATE cxx_range_for)
