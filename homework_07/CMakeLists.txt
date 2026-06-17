add_library(
    homework_07_mission src/analytical_solver.cpp src/file_config_loader.cpp
                        src/json_target_provider.cpp src/factory.cpp src/mission_processor.cpp)

target_include_directories(homework_07_mission PUBLIC include)
target_include_directories(homework_07_mission SYSTEM PUBLIC third_party)

target_compile_features(homework_07_mission PUBLIC cxx_std_20)
target_compile_options(homework_07_mission PRIVATE -Wall -Wextra -Wpedantic)

add_executable(homework_07_cli src/main.cpp)

target_link_libraries(homework_07_cli PRIVATE homework_07_mission)
target_compile_features(homework_07_cli PRIVATE cxx_std_20)
target_compile_options(homework_07_cli PRIVATE -Wall -Wextra -Wpedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(homework_07_tests tests/homework_07_tests.cpp)

    target_link_libraries(homework_07_tests PRIVATE homework_07_mission GTest::gtest_main)
    target_compile_features(homework_07_tests PRIVATE cxx_std_20)
    target_compile_options(homework_07_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(homework_07_tests
                               PRIVATE HOMEWORK_07_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

    gtest_discover_tests(homework_07_tests DISCOVERY_MODE PRE_TEST)
endif()
