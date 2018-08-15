cmake_minimum_required(VERSION 2.8.11)
project(ROPOD_CCU)
set (CMAKE_CXX_STANDARD 11)


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig)
pkg_check_modules(yaml_cpp REQUIRED yaml-cpp)

pkg_check_modules(LIBMONGOCXX REQUIRED libmongocxx)
pkg_check_modules(LIBBSONCXX REQUIRED libbsoncxx)

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
    /opt/ropod/ropod_common/zyre_communicator/include
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

link_directories(
    ${LIBMONGOCXX_LIBRARY_DIRS}
    ${LIBBSONCXX_LIBRARY_DIRS}
)

add_library(cculib
    /opt/ropod/ropod_common/zyre_communicator/src/ZyreBaseCommunicator.cpp
    src/db/ccu_store.cpp
    src/path_planner.cpp
    src/resource_manager.cpp
    src/task_manager.cpp
    src/task_planner.cpp
)

target_link_libraries(cculib
    zmq
    jsoncpp
    yaml-cpp
    ${LIBMONGOCXX_LIBRARIES}
    ${LIBBSONCXX_LIBRARIES}
    ${LIBS}
)

set_target_properties(cculib PROPERTIES LINKER_LANGUAGE CXX)


add_executable(ccu
    src/ccu.cpp
    src/config/config_file_reader.cpp
)

target_link_libraries(ccu
    cculib
)

add_subdirectory (test)

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
  ${LIBMONGOCXX_LIBRARIES}
  ${LIBBSONCXX_LIBRARIES}
  ${LIBS}
)

target_compile_features(ccu_cli PRIVATE cxx_range_for)

add_executable(mobidik_experiment
    src/config/config_file_reader.cpp
    ${ZYRE_WRAPPER_SRC}
    src/ccu_manager.cpp
    src/experiments/mobidik_elevator.cpp
    src/experiments/mobidik_elevator_main.cpp
)

target_link_libraries(mobidik_experiment
  zmq
  jsoncpp
  yaml-cpp
  ${LIBS}
)

target_compile_features(mobidik_experiment PRIVATE cxx_range_for)
