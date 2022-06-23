cmake_minimum_required(VERSION 3.15)

# set the project name
project(benchmark-utils)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3") # for development use no optimization

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find boost
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system REQUIRED)

add_library(BenchmarkUtils MeasurementGenerator.hpp MeasurementGenerator.cpp)
target_link_libraries(BenchmarkUtils Boost::boost)

# add the executable
add_executable(mock-data-generator main.cpp)
target_link_libraries(mock-data-generator BenchmarkUtils)
# target_include_directories(mock-data-generator SYSTEM PUBLIC Boost::headers)
