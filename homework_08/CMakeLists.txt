add_library(
    homework_08_mission
    src/MissionProcessor.cpp src/solvers/AnalyticalSolver.cpp src/providers/JsonTargetProvider.cpp
    src/config/FileConfigLoader.cpp src/config/ComponentFactory.cpp)

target_include_directories(homework_08_mission PUBLIC include)
target_include_directories(homework_08_mission SYSTEM PUBLIC third_party)

target_compile_features(homework_08_mission PUBLIC cxx_std_20)
target_compile_options(homework_08_mission PRIVATE -Wall -Wextra -Wpedantic)

add_executable(homework_08_cli src/main.cpp)

target_link_libraries(homework_08_cli PRIVATE homework_08_mission)
target_compile_features(homework_08_cli PRIVATE cxx_std_20)
target_compile_options(homework_08_cli PRIVATE -Wall -Wextra -Wpedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(homework_08_tests tests/homework_08_tests.cpp)

    target_link_libraries(homework_08_tests PRIVATE homework_08_mission GTest::gtest_main)
    target_compile_features(homework_08_tests PRIVATE cxx_std_20)
    target_compile_options(homework_08_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(homework_08_tests
                               PRIVATE HOMEWORK_08_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

    gtest_discover_tests(homework_08_tests DISCOVERY_MODE PRE_TEST)
endif()
