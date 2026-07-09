add_library(
    homework_10_mission
    src/MissionProcessor.cpp
    src/solvers/AnalyticalSolver.cpp
    src/solvers/TableSolver.cpp
    src/providers/ThreadSafeTargetProvider.cpp
    src/config/FileConfigLoader.cpp
    src/config/ComponentFactory.cpp
    src/drone/DroneState.cpp
    src/drone/DronePhysics.cpp)

target_include_directories(homework_10_mission PUBLIC include)
target_include_directories(homework_10_mission SYSTEM PUBLIC third_party)

target_compile_features(homework_10_mission PUBLIC cxx_std_20)
target_compile_options(homework_10_mission PRIVATE -Wall -Wextra -Wpedantic)

add_executable(homework_10_cli src/main.cpp)

target_link_libraries(homework_10_cli PRIVATE homework_10_mission)
target_compile_features(homework_10_cli PRIVATE cxx_std_20)
target_compile_options(homework_10_cli PRIVATE -Wall -Wextra -Wpedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(homework_10_tests tests/homework_10_tests.cpp)

    target_link_libraries(homework_10_tests PRIVATE homework_10_mission GTest::gtest_main)
    target_compile_features(homework_10_tests PRIVATE cxx_std_20)
    target_compile_options(homework_10_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_definitions(homework_10_tests
                               PRIVATE HOMEWORK_10_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

    gtest_discover_tests(homework_10_tests DISCOVERY_MODE PRE_TEST)
endif()
