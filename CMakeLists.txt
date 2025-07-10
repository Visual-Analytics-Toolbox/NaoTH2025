cmake_minimum_required(VERSION 4.0)
project(NaoTH2025 LANGUAGES CXX)

# Set C++ standard and compiler options
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

add_executable(NaoRobot platforms/NaoRobot/main.cpp)