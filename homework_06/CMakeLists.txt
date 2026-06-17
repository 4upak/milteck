add_library(ballistics src/ballistics.cpp)

target_include_directories(ballistics PUBLIC include)
target_compile_features(ballistics PUBLIC cxx_std_20)
target_compile_options(ballistics PRIVATE -Wall -Wextra -Wpedantic)

add_executable(ballistics_cli src/main.cpp)

target_link_libraries(ballistics_cli PRIVATE ballistics)
target_compile_features(ballistics_cli PRIVATE cxx_std_20)
target_compile_options(ballistics_cli PRIVATE -Wall -Wextra -Wpedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(ballistics_tests tests/ballistics_tests.cpp)

    target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main)
    target_compile_features(ballistics_tests PRIVATE cxx_std_20)
    target_compile_options(ballistics_tests PRIVATE -Wall -Wextra -Wpedantic)

    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
endif()
