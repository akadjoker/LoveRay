cmake_minimum_required(VERSION 3.16)
project(LoveRay VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(LOVERAY_SANITIZE "Build with address/undefined sanitizers (Debug only)" OFF)

if(EMSCRIPTEN)
    # OpenGL/X11 are provided by the browser; raylib's CMake handles the rest.
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()

# Third-party libraries, all vendored and built as static libraries.
add_subdirectory(vendor/box2d)
add_subdirectory(vendor/lua)
add_subdirectory(vendor/raylib)

# The engine itself: produces the `love` executable in <repo>/bin.
add_subdirectory(src)
