cmake_minimum_required(VERSION 3.16)

project(homework_04 LANGUAGES CXX)

add_executable(ugv_odometry src/main.cpp)

target_compile_features(ugv_odometry PRIVATE cxx_std_17)
target_compile_options(ugv_odometry PRIVATE -Wall -Wextra -Wpedantic)
