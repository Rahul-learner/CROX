cmake_minimum_required(VERSION 3.10)
project(GS_Monitor)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This is a standard PC application, NO Pico SDK!
add_executable(GS_Monitor main.cpp)

# For running background threads (useful for serial reading)
find_package(Threads REQUIRED)
target_link_libraries(GS_Monitor Threads::Threads)
