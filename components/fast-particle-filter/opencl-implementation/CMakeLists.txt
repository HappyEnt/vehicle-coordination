cmake_minimum_required(VERSION 3.15)

project(opencl_cmake VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(vadd src/main.cpp)
target_compile_features(vadd PRIVATE cxx_auto_type)

find_package(OpenCL REQUIRED)
target_link_libraries(vadd OpenCL::OpenCL)
