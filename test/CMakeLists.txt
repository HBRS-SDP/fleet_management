include_directories(include
        test/include)

add_executable(ccu_tests
    src/ccu_test.cpp
    ../src/config/config_file_reader.cpp
)

target_link_libraries(ccu_tests
    cculib
)

add_executable(task_request_test
    src/task_request_test.cpp
)
target_link_libraries(task_request_test
    cculib
)

add_executable(elevator_request_test
        src/elevator_request_test.cpp)

target_link_libraries(elevator_request_test
        cculib)

