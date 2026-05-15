add_library(telemetry src/telemetry.cpp)

target_include_directories(telemetry PUBLIC include)
target_compile_features(telemetry PUBLIC cxx_std_20)
target_compile_options(telemetry PRIVATE -Wall -Wextra -Wpedantic)

add_executable(telemetry_check src/main.cpp)

target_link_libraries(telemetry_check PRIVATE telemetry)
target_compile_features(telemetry_check PRIVATE cxx_std_20)
target_compile_options(telemetry_check PRIVATE -Wall -Wextra -Wpedantic)
