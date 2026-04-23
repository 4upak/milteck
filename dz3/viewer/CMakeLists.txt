cmake_minimum_required(VERSION 3.16)
project(drone_viewer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# raylib installed via Homebrew on Apple Silicon lives under /opt/homebrew/opt/raylib
if(NOT DEFINED RAYLIB_ROOT)
    if(EXISTS "/opt/homebrew/opt/raylib")
        set(RAYLIB_ROOT "/opt/homebrew/opt/raylib")
    elseif(EXISTS "/usr/local/opt/raylib")
        set(RAYLIB_ROOT "/usr/local/opt/raylib")
    endif()
endif()

find_package(raylib 4.0 QUIET PATHS "${RAYLIB_ROOT}")

add_executable(drone_viewer viewer.cpp)

if(raylib_FOUND)
    target_link_libraries(drone_viewer PRIVATE raylib)
else()
    target_include_directories(drone_viewer PRIVATE "${RAYLIB_ROOT}/include")
    target_link_directories(drone_viewer PRIVATE "${RAYLIB_ROOT}/lib")
    target_link_libraries(drone_viewer PRIVATE raylib)
endif()

if(APPLE)
    target_link_libraries(drone_viewer PRIVATE
        "-framework CoreVideo"
        "-framework IOKit"
        "-framework Cocoa"
        "-framework GLUT"
        "-framework OpenGL"
    )
endif()

target_compile_options(drone_viewer PRIVATE -Wall -Wextra -O2)
