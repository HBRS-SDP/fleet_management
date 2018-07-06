add_executable(ccu_tests
    src/ccu_test.cpp
    ../src/config/config_file_reader.cpp
)

target_link_libraries(ccu_tests
    cculib
)
