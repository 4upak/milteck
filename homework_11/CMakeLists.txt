add_library(
    homework_11_mission
    src/MissionProcessor.cpp
    src/io/UartPort.cpp
    src/io/GpioController.cpp)

target_include_directories(homework_11_mission PUBLIC include)
target_compile_features(homework_11_mission PUBLIC cxx_std_20)
target_compile_options(homework_11_mission PRIVATE -Wall -Wextra -Wpedantic)

target_compile_definitions(homework_11_mission PRIVATE _DEFAULT_SOURCE=1)

# The GpioController uses the libgpiod v2 API (gpiod_line_settings_new,
# gpiod_chip_request_lines, ...). A bare header check is not enough because
# Raspberry Pi OS often ships libgpiod 1.x, whose header exists but lacks the
# v2 symbols. Prefer pkg-config to require >= 2.0; fall back to a header check
# only with a loud warning.
find_package(PkgConfig QUIET)
set(HOMEWORK11_GPIOD_FOUND FALSE)

if(PkgConfig_FOUND)
  pkg_check_modules(GPIOD QUIET libgpiod>=2.0)
  if(GPIOD_FOUND)
    set(HOMEWORK11_GPIOD_FOUND TRUE)
    target_compile_definitions(homework_11_mission PRIVATE HOMEWORK11_HAS_GPIOD_H=1)
    target_include_directories(homework_11_mission PRIVATE ${GPIOD_INCLUDE_DIRS})
    target_link_directories(homework_11_mission PRIVATE ${GPIOD_LIBRARY_DIRS})
    target_link_libraries(homework_11_mission PUBLIC ${GPIOD_LIBRARIES})
    message(STATUS "homework_11: using libgpiod ${GPIOD_VERSION}")
  endif()
endif()

if(NOT HOMEWORK11_GPIOD_FOUND)
  message(WARNING
    "homework_11: libgpiod >= 2.0 not found. Building WITHOUT GPIO support: "
    "START/DROP will be no-ops and GpioController::open() will throw at runtime. "
    "Install libgpiod-dev (>= 2.0) to enable GPIO.")
endif()

add_executable(homework_11_cli src/main.cpp)
target_link_libraries(homework_11_cli PRIVATE homework_11_mission)
target_compile_features(homework_11_cli PRIVATE cxx_std_20)
target_compile_options(homework_11_cli PRIVATE -Wall -Wextra -Wpedantic)
