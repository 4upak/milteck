cmake_minimum_required(VERSION 3.20)

project(miltech_homeworks LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/homework_04/CMakeLists.txt")
  add_subdirectory(homework_04)
endif()

add_subdirectory(homework_05)
add_subdirectory(homework_06)
add_subdirectory(homework_07)
add_subdirectory(homework_08)
add_subdirectory(homework_09)
