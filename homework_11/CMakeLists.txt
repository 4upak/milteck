add_library(
    homework_11_mission
    src/MissionProcessor.cpp
    src/io/UartPort.cpp
    src/io/GpioController.cpp)

target_include_directories(homework_11_mission PUBLIC include)
target_compile_features(homework_11_mission PUBLIC cxx_std_20)
target_compile_options(homework_11_mission PRIVATE -Wall -Wextra -Wpedantic)

target_compile_definitions(homework_11_mission PRIVATE _DEFAULT_SOURCE=1)

include(CheckIncludeFileCXX)
check_include_file_cxx(gpiod.h HOMEWORK11_HAS_GPIOD_H)
if(HOMEWORK11_HAS_GPIOD_H)
  target_compile_definitions(homework_11_mission PRIVATE HOMEWORK11_HAS_GPIOD_H=1)
  target_link_libraries(homework_11_mission PUBLIC gpiod)
endif()

add_executable(homework_11_cli src/main.cpp)
target_link_libraries(homework_11_cli PRIVATE homework_11_mission)
target_compile_features(homework_11_cli PRIVATE cxx_std_20)
target_compile_options(homework_11_cli PRIVATE -Wall -Wextra -Wpedantic)
