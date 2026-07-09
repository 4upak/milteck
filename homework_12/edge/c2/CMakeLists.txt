cmake_minimum_required(VERSION 3.20)
project(homework_12_c2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(c2_service
    src/main.cpp
    src/fc_link.cpp
    src/c2_controller.cpp)

target_compile_options(c2_service PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(c2_service PRIVATE src)
