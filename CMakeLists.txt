cmake_minimum_required(VERSION 4.0)
project(NaoTH2025 LANGUAGES CXX)

# Set C++ standard and compiler options
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

if (MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -pedantic -Werror)
endif()

add_executable(NaoRobot platforms/NaoRobot/main.cpp)